#include "proto_text.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "app_config.h"
#include "app_calibration.h"
#include "kia_err.h"
#include "kinematics.h"
#include "svc_motion.h"
#include "svc_safety.h"
#include "svc_storage.h"

static const char *TAG = "proto_text";

#define LINE_MAX 192

typedef struct {
    char     buf[LINE_MAX];
    size_t   n;
} line_acc_t;

static line_acc_t  s_acc[COMM_TRANSPORT__COUNT];
static SemaphoreHandle_t s_mtx;

static int reply(comm_transport_id_t t, const char *fmt, ...)
{
    char    out[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out, sizeof out - 2, fmt, ap);
    va_end(ap);
    if (n < 0) return n;
    if (n > (int)sizeof out - 2) n = sizeof out - 2;
    out[n++] = '\n';
    svc_comm_send(t, (uint8_t *)out, n);
    return n;
}

typedef int (*cmd_fn_t)(comm_transport_id_t t, char *args);

static int cmd_ping(comm_transport_id_t t, char *a)
{
    KIA_UNUSED(a);
    return reply(t, "PONG fw=%d.%d.%d", KIA_FW_VERSION_MAJOR, KIA_FW_VERSION_MINOR, KIA_FW_VERSION_PATCH);
}

static int cmd_status(comm_transport_id_t t, char *a)
{
    KIA_UNUSED(a);
    kin_joints_t j;
    float        g;
    kin_pose_t   p;
    svc_motion_get_joints(&j, &g);
    svc_motion_get_pose(&p);
    return reply(t,
                 "STATUS J=[%.2f,%.2f,%.2f,%.2f,%.2f] XYZP=[%.1f,%.1f,%.1f,%.1f] I=%ld FAULT=%d SRC=%d",
                 j.base_deg, j.shoulder_deg, j.elbow_deg, j.wrist_deg, g, p.x_mm, p.y_mm, p.z_mm,
                 p.pitch_deg, (long)svc_safety_last_current_ma(), svc_safety_last_fault(),
                 svc_motion_get_source());
}

static int cmd_j(comm_transport_id_t t, char *a)
{
    int   id;
    float deg;
    float dur = 1.0f;
    int   n = sscanf(a, "%d %f %f", &id, &deg, &dur);
    if (n < 2) return reply(t, "ERR proto");
    esp_err_t e = svc_motion_set_joint_target((uint8_t)id, deg, dur);
    return reply(t, e == ESP_OK ? "OK" : "ERR 0x%x", e);
}

static int cmd_xyz(comm_transport_id_t t, char *a)
{
    kin_pose_t p = {0};
    float      dur = 2.0f;
    int        n   = sscanf(a, "%f %f %f %f %f", &p.x_mm, &p.y_mm, &p.z_mm, &p.pitch_deg, &dur);
    if (n < 4) return reply(t, "ERR proto");
    esp_err_t e = svc_motion_set_cartesian_target(&p, dur);
    return reply(t, e == ESP_OK ? "OK" : "ERR 0x%x", e);
}

static int cmd_mode(comm_transport_id_t t, char *a)
{
    while (*a == ' ') ++a;
    if      (!strncasecmp(a, "MANUAL", 6)) svc_motion_set_source(MOTION_SRC_MANUAL_POT);
    else if (!strncasecmp(a, "APP", 3))    svc_motion_set_source(MOTION_SRC_HOLD);
    else if (!strncasecmp(a, "HOLD", 4))   svc_motion_set_source(MOTION_SRC_HOLD);
    else return reply(t, "ERR mode");
    return reply(t, "OK");
}

static int cmd_home(comm_transport_id_t t, char *a) { KIA_UNUSED(a); return reply(t, svc_motion_home() == ESP_OK ? "OK" : "ERR"); }
static int cmd_hold(comm_transport_id_t t, char *a) { KIA_UNUSED(a); svc_motion_hold(); return reply(t, "OK"); }
static int cmd_arm(comm_transport_id_t t, char *a)  { KIA_UNUSED(a); return reply(t, svc_safety_arm() == ESP_OK ? "OK" : "ERR"); }
static int cmd_clear(comm_transport_id_t t, char *a){ KIA_UNUSED(a); return reply(t, svc_safety_clear() == ESP_OK ? "OK" : "ERR"); }
static int cmd_save(comm_transport_id_t t, char *a) { KIA_UNUSED(a); return reply(t, svc_storage_save_calibration() == ESP_OK ? "OK" : "ERR"); }
static int cmd_cal(comm_transport_id_t t, char *a)
{
    /* Forms:
     *   CAL J<i> ZERO|MIN|MAX
     *   CAL P<i> MIN|MAX
     *   CAL GEOM <base_h> <upper> <fore> <tool>
     *   CAL FACTORY
     *   CAL LOAD
     */
    while (*a == ' ') ++a;
    if (!strncasecmp(a, "GEOM", 4)) {
        float bh, up, fo, to;
        if (sscanf(a + 4, "%f %f %f %f", &bh, &up, &fo, &to) != 4) return reply(t, "ERR cal proto");
        return reply(t, app_calibration_set_geometry(bh, up, fo, to) == ESP_OK ? "OK" : "ERR");
    }
    if (!strncasecmp(a, "FACTORY", 7)) {
        return reply(t, svc_storage_factory_reset() == ESP_OK ? "OK reboot to apply" : "ERR");
    }
    if (!strncasecmp(a, "LOAD", 4)) {
        return reply(t, svc_storage_load_calibration() == ESP_OK ? "OK" : "ERR");
    }
    if ((a[0] == 'J' || a[0] == 'j') && isdigit((unsigned char)a[1])) {
        int   idx = a[1] - '0';
        char *op  = a + 2;
        while (*op == ' ') ++op;
        app_cal_servo_op_t k;
        if      (!strncasecmp(op, "ZERO", 4)) k = APP_CAL_SERVO_ZERO;
        else if (!strncasecmp(op, "MIN", 3))  k = APP_CAL_SERVO_MIN;
        else if (!strncasecmp(op, "MAX", 3))  k = APP_CAL_SERVO_MAX;
        else return reply(t, "ERR cal op");
        return reply(t, app_calibration_servo((uint8_t)idx, k) == ESP_OK ? "OK" : "ERR");
    }
    if ((a[0] == 'P' || a[0] == 'p') && isdigit((unsigned char)a[1])) {
        int   idx = a[1] - '0';
        char *op  = a + 2;
        while (*op == ' ') ++op;
        app_cal_pot_op_t k;
        if      (!strncasecmp(op, "MIN", 3)) k = APP_CAL_POT_MIN;
        else if (!strncasecmp(op, "MAX", 3)) k = APP_CAL_POT_MAX;
        else return reply(t, "ERR cal op");
        return reply(t, app_calibration_pot((uint8_t)idx, k) == ESP_OK ? "OK" : "ERR");
    }
    return reply(t, "ERR cal proto");
}

static int cmd_help(comm_transport_id_t t, char *a)
{
    KIA_UNUSED(a);
    return reply(t, "CMDS: PING STATUS J<i,deg[,dur]> XYZ<x,y,z,pitch[,dur]> "
                     "MODE<MANUAL|APP|HOLD> HOME HOLD ARM CLEAR SAVE "
                     "CAL<J<i>{ZERO|MIN|MAX}|P<i>{MIN|MAX}|GEOM bh up fo to|LOAD|FACTORY> "
                     "QPUSH<J|XYZ ...> QCLEAR HELP");
}

typedef struct { const char *name; cmd_fn_t fn; } cmd_t;
static const cmd_t TABLE[] = {
    {"PING", cmd_ping},     {"STATUS", cmd_status}, {"J", cmd_j},        {"XYZ", cmd_xyz},
    {"MODE", cmd_mode},     {"HOME", cmd_home},     {"HOLD", cmd_hold},  {"ARM", cmd_arm},
    {"CLEAR", cmd_clear},   {"SAVE", cmd_save},     {"CAL", cmd_cal},    {"HELP", cmd_help},
};

static void dispatch(comm_transport_id_t t, char *line)
{
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == 0) return;
    char *sp = strpbrk(line, " \t");
    char *args = "";
    if (sp) { *sp = 0; args = sp + 1; }
    for (char *p = line; *p; ++p) *p = (char)toupper((unsigned char)*p);

    for (size_t i = 0; i < KIA_ARRAY_LEN(TABLE); ++i) {
        if (!strcmp(line, TABLE[i].name)) {
            TABLE[i].fn(t, args);
            return;
        }
    }
    reply(t, "ERR cmd=%s", line);
}

void proto_text_feed(comm_transport_id_t t, const uint8_t *data, size_t n)
{
    if (!s_mtx) s_mtx = xSemaphoreCreateMutex();
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    line_acc_t *a = &s_acc[t];
    for (size_t i = 0; i < n; ++i) {
        char c = (char)data[i];
        if (c == '\r' || c == '\n') {
            if (a->n) {
                a->buf[a->n] = 0;
                dispatch(t, a->buf);
                a->n = 0;
            }
        } else if (a->n + 1 < LINE_MAX) {
            a->buf[a->n++] = c;
        } else {
            a->n = 0; /* line too long → drop */
        }
    }
    xSemaphoreGive(s_mtx);
}
