#include "proto_binary.h"

#include <string.h>

#include "cobs.h"
#include "crc32.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "app_config.h"
#include "kia_err.h"
#include "kinematics.h"
#include "svc_motion.h"
#include "svc_ota.h"
#include "svc_safety.h"
#include "svc_storage.h"

static const char *TAG = "proto_bin";

/* Wire frame (pre-COBS):
 *   [u8 op | u8 flags | u16 seq | u32 crc32 | u8 body[]]
 * crc32 = CRC32-ISO over (op | flags | seq | body). */
#define HDR_LEN 8

typedef struct {
    uint8_t raw[KIA_COMM_FRAME_MAX];
    size_t  n;
} acc_t;

static acc_t              s_acc[COMM_TRANSPORT__COUNT];
static SemaphoreHandle_t  s_tx_mtx;
static uint16_t           s_async_seq;

static inline void put_u16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static inline void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}
static inline uint16_t get_u16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static inline uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline float get_f32(const uint8_t *p)
{
    uint32_t u = get_u32(p);
    float    f;
    memcpy(&f, &u, 4);
    return f;
}
static inline void put_f32(uint8_t *p, float f)
{
    uint32_t u;
    memcpy(&u, &f, 4);
    put_u32(p, u);
}

int proto_binary_send(comm_transport_id_t t, uint8_t op, uint16_t seq, const void *body, size_t body_n)
{
    if (body_n + HDR_LEN > KIA_COMM_FRAME_MAX) return -1;
    if (!s_tx_mtx) s_tx_mtx = xSemaphoreCreateMutex();
    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);

    static uint8_t frame[KIA_COMM_FRAME_MAX];
    static uint8_t enc[KIA_COMM_FRAME_MAX + 4];
    frame[0] = op;
    frame[1] = 0;
    put_u16(frame + 2, seq);
    if (body && body_n) memcpy(frame + HDR_LEN, body, body_n);

    /* CRC over [op|flags|seq|body], skip the crc slot. */
    uint8_t tmp[KIA_COMM_FRAME_MAX];
    tmp[0] = frame[0]; tmp[1] = frame[1]; tmp[2] = frame[2]; tmp[3] = frame[3];
    if (body && body_n) memcpy(tmp + 4, body, body_n);
    uint32_t crc = kia_crc32(tmp, 4 + body_n);
    put_u32(frame + 4, crc);

    size_t enc_n = kia_cobs_encode(frame, HDR_LEN + body_n, enc, sizeof enc);
    int    rc   = -1;
    if (enc_n > 0) {
        if (t == COMM_TRANSPORT__COUNT) {
            rc = svc_comm_broadcast(enc, enc_n) == ESP_OK ? (int)enc_n : -1;
        } else {
            rc = svc_comm_send(t, enc, enc_n) == ESP_OK ? (int)enc_n : -1;
        }
    }
    xSemaphoreGive(s_tx_mtx);
    return rc;
}

int proto_binary_broadcast(uint8_t op, uint16_t seq, const void *body, size_t body_n)
{
    return proto_binary_send(COMM_TRANSPORT__COUNT, op, seq, body, body_n);
}

int proto_binary_ack(comm_transport_id_t t, uint16_t seq)
{
    return proto_binary_send(t, OP_ACK, seq, NULL, 0);
}

int proto_binary_nack(comm_transport_id_t t, uint16_t seq, kia_nack_t reason)
{
    uint8_t b = (uint8_t)reason;
    return proto_binary_send(t, OP_NACK, seq, &b, 1);
}

/* === Dispatchers (per opcode) ============================================ */

static kia_nack_t map_err(esp_err_t e)
{
    switch (e) {
        case ESP_OK:                 return (kia_nack_t)0;
        case ESP_ERR_INVALID_ARG:    return NACK_BAD_ARG;
        case ESP_ERR_INVALID_STATE:  return NACK_BAD_STATE;
        case ESP_ERR_NO_MEM:         return NACK_NOMEM;
        case KIA_ERR_LIMIT:          return NACK_LIMIT;
        case KIA_ERR_UNREACHABLE:    return NACK_UNREACHABLE;
        case KIA_ERR_FAULT_ACTIVE:   return NACK_FAULT;
        default:                     return NACK_INTERNAL;
    }
}

static void on_ping(comm_transport_id_t t, uint16_t seq, const uint8_t *b, size_t n)
{
    KIA_UNUSED(b); KIA_UNUSED(n);
    uint8_t body[3] = {KIA_FW_VERSION_MAJOR, KIA_FW_VERSION_MINOR, KIA_FW_VERSION_PATCH};
    proto_binary_send(t, OP_PONG, seq, body, sizeof body);
}

static void on_get_status(comm_transport_id_t t, uint16_t seq, const uint8_t *b, size_t n)
{
    KIA_UNUSED(b); KIA_UNUSED(n);
    kin_joints_t j; float g; kin_pose_t p;
    svc_motion_get_joints(&j, &g);
    svc_motion_get_pose(&p);

    uint8_t body[60];
    uint8_t *q = body;
    put_f32(q, j.base_deg);     q += 4;
    put_f32(q, j.shoulder_deg); q += 4;
    put_f32(q, j.elbow_deg);    q += 4;
    put_f32(q, j.wrist_deg);    q += 4;
    put_f32(q, g);              q += 4;
    put_f32(q, p.x_mm);         q += 4;
    put_f32(q, p.y_mm);         q += 4;
    put_f32(q, p.z_mm);         q += 4;
    put_f32(q, p.pitch_deg);    q += 4;
    put_u32(q, (uint32_t)svc_safety_last_current_ma()); q += 4;
    *q++ = (uint8_t)svc_safety_last_fault();
    *q++ = (uint8_t)svc_motion_get_source();
    *q++ = (uint8_t)svc_motion_is_idle();
    *q++ = 0;
    proto_binary_send(t, OP_STATUS, seq, body, q - body);
}

static void on_set_joint(comm_transport_id_t t, uint16_t seq, const uint8_t *b, size_t n)
{
    if (n < 9) { proto_binary_nack(t, seq, NACK_BAD_LEN); return; }
    uint8_t idx = b[0];
    float   deg = get_f32(b + 1);
    float   dur = get_f32(b + 5);
    esp_err_t e = svc_motion_set_joint_target(idx, deg, dur);
    if (e == ESP_OK) proto_binary_ack(t, seq);
    else proto_binary_nack(t, seq, map_err(e));
}

static void on_set_joints(comm_transport_id_t t, uint16_t seq, const uint8_t *b, size_t n)
{
    if (n < 24) { proto_binary_nack(t, seq, NACK_BAD_LEN); return; }
    kin_joints_t j = {
        .base_deg     = get_f32(b),
        .shoulder_deg = get_f32(b + 4),
        .elbow_deg    = get_f32(b + 8),
        .wrist_deg    = get_f32(b + 12),
    };
    float grip = get_f32(b + 16);
    float dur  = get_f32(b + 20);
    esp_err_t e = svc_motion_set_joint_targets(&j, grip, dur);
    if (e == ESP_OK) proto_binary_ack(t, seq);
    else proto_binary_nack(t, seq, map_err(e));
}

static void on_set_xyz(comm_transport_id_t t, uint16_t seq, const uint8_t *b, size_t n)
{
    if (n < 20) { proto_binary_nack(t, seq, NACK_BAD_LEN); return; }
    kin_pose_t p = {
        .x_mm      = get_f32(b),
        .y_mm      = get_f32(b + 4),
        .z_mm      = get_f32(b + 8),
        .pitch_deg = get_f32(b + 12),
    };
    float dur = get_f32(b + 16);
    esp_err_t e = svc_motion_set_cartesian_target(&p, dur);
    if (e == ESP_OK) proto_binary_ack(t, seq);
    else proto_binary_nack(t, seq, map_err(e));
}

static void on_simple(comm_transport_id_t t, uint16_t seq, esp_err_t e)
{
    if (e == ESP_OK) proto_binary_ack(t, seq);
    else proto_binary_nack(t, seq, map_err(e));
}

typedef void (*op_fn_t)(comm_transport_id_t, uint16_t, const uint8_t *, size_t);

static void on_home(comm_transport_id_t t, uint16_t s, const uint8_t *b, size_t n)  { KIA_UNUSED(b);KIA_UNUSED(n); on_simple(t, s, svc_motion_home()); }
static void on_hold(comm_transport_id_t t, uint16_t s, const uint8_t *b, size_t n)  { KIA_UNUSED(b);KIA_UNUSED(n); on_simple(t, s, svc_motion_hold()); }
static void on_arm(comm_transport_id_t t, uint16_t s, const uint8_t *b, size_t n)   { KIA_UNUSED(b);KIA_UNUSED(n); on_simple(t, s, svc_safety_arm()); }
static void on_clear(comm_transport_id_t t, uint16_t s, const uint8_t *b, size_t n) { KIA_UNUSED(b);KIA_UNUSED(n); on_simple(t, s, svc_safety_clear()); }
static void on_save(comm_transport_id_t t, uint16_t s, const uint8_t *b, size_t n)  { KIA_UNUSED(b);KIA_UNUSED(n); on_simple(t, s, svc_storage_save_calibration()); }
static void on_load(comm_transport_id_t t, uint16_t s, const uint8_t *b, size_t n)  { KIA_UNUSED(b);KIA_UNUSED(n); on_simple(t, s, svc_storage_load_calibration()); }
static void on_fact(comm_transport_id_t t, uint16_t s, const uint8_t *b, size_t n)  { KIA_UNUSED(b);KIA_UNUSED(n); on_simple(t, s, svc_storage_factory_reset()); }

static void on_ota_begin(comm_transport_id_t t, uint16_t s, const uint8_t *b, size_t n)
{
    if (n < 10) { proto_binary_nack(t, s, NACK_BAD_LEN); return; }
    uint32_t total = get_u32(b);
    uint16_t chunk = get_u16(b + 4);
    uint32_t crc   = get_u32(b + 6);
    on_simple(t, s, svc_ota_begin(t, total, chunk, crc));
}

static void on_ota_chunk(comm_transport_id_t t, uint16_t s, const uint8_t *b, size_t n)
{
    if (n < 5) { proto_binary_nack(t, s, NACK_BAD_LEN); return; }
    uint32_t off = get_u32(b);
    on_simple(t, s, svc_ota_chunk(t, off, b + 4, n - 4));
}

static void on_ota_end(comm_transport_id_t t, uint16_t s, const uint8_t *b, size_t n)
{
    KIA_UNUSED(b); KIA_UNUSED(n);
    on_simple(t, s, svc_ota_end(t));
}

static void on_ota_abort(comm_transport_id_t t, uint16_t s, const uint8_t *b, size_t n)
{
    KIA_UNUSED(b); KIA_UNUSED(n);
    on_simple(t, s, svc_ota_abort());
}

typedef struct { uint8_t op; op_fn_t fn; } entry_t;
static const entry_t TABLE[] = {
    {OP_PING,         on_ping},
    {OP_GET_STATUS,   on_get_status},
    {OP_SET_JOINT,    on_set_joint},
    {OP_SET_JOINTS,   on_set_joints},
    {OP_SET_XYZ,      on_set_xyz},
    {OP_HOME,         on_home},
    {OP_HOLD,         on_hold},
    {OP_ARM,          on_arm},
    {OP_CLEAR_FAULT,  on_clear},
    {OP_SAVE_CAL,     on_save},
    {OP_LOAD_CAL,     on_load},
    {OP_FACTORY_RST,  on_fact},
    {OP_OTA_BEGIN,    on_ota_begin},
    {OP_OTA_CHUNK,    on_ota_chunk},
    {OP_OTA_END,      on_ota_end},
    {OP_OTA_ABORT,    on_ota_abort},
};

static void handle_frame(comm_transport_id_t t, const uint8_t *dec, size_t n)
{
    if (n < HDR_LEN) return;
    uint8_t  op    = dec[0];
    uint8_t  flags = dec[1];
    uint16_t seq   = get_u16(dec + 2);
    uint32_t crc   = get_u32(dec + 4);
    size_t   body_n = n - HDR_LEN;

    uint8_t tmp[KIA_COMM_FRAME_MAX];
    tmp[0] = op; tmp[1] = flags; tmp[2] = dec[2]; tmp[3] = dec[3];
    memcpy(tmp + 4, dec + HDR_LEN, body_n);
    if (kia_crc32(tmp, 4 + body_n) != crc) {
        proto_binary_nack(t, seq, NACK_INTERNAL);
        return;
    }

    for (size_t i = 0; i < KIA_ARRAY_LEN(TABLE); ++i) {
        if (TABLE[i].op == op) {
            TABLE[i].fn(t, seq, dec + HDR_LEN, body_n);
            return;
        }
    }
    proto_binary_nack(t, seq, NACK_BAD_OP);
}

void proto_binary_feed(comm_transport_id_t t, const uint8_t *data, size_t n)
{
    acc_t *a = &s_acc[t];
    for (size_t i = 0; i < n; ++i) {
        uint8_t b = data[i];
        if (b == 0x00) {
            if (a->n) {
                uint8_t dec[KIA_COMM_FRAME_MAX];
                a->raw[a->n++] = 0x00;
                size_t dl      = kia_cobs_decode(a->raw, a->n, dec, sizeof dec);
                if (dl) handle_frame(t, dec, dl);
                a->n = 0;
            }
        } else if (a->n < KIA_COMM_FRAME_MAX - 1) {
            a->raw[a->n++] = b;
        } else {
            a->n = 0;
        }
    }
}

int proto_binary_telemetry_broadcast(uint16_t seq)
{
    kin_joints_t j; float g; kin_pose_t p;
    svc_motion_get_joints(&j, &g);
    svc_motion_get_pose(&p);

    uint8_t body[44];
    uint8_t *q = body;
    put_f32(q, j.base_deg);     q += 4;
    put_f32(q, j.shoulder_deg); q += 4;
    put_f32(q, j.elbow_deg);    q += 4;
    put_f32(q, j.wrist_deg);    q += 4;
    put_f32(q, g);              q += 4;
    put_f32(q, p.x_mm);         q += 4;
    put_f32(q, p.y_mm);         q += 4;
    put_f32(q, p.z_mm);         q += 4;
    put_f32(q, p.pitch_deg);    q += 4;
    put_u32(q, (uint32_t)svc_safety_last_current_ma()); q += 4;
    *q++ = (uint8_t)svc_safety_last_fault();
    *q++ = (uint8_t)svc_motion_get_source();
    *q++ = (uint8_t)svc_motion_is_idle();
    *q++ = 0;
    return proto_binary_broadcast(OP_TELEMETRY, seq ? seq : ++s_async_seq, body, q - body);
}
