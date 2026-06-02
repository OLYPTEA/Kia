#include "svc_comm.h"

#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "app_config.h"
#include "app_events.h"
#include "esp_event.h"
#include "kia_err.h"

static const char *TAG = "tr_ble";

/* === Custom 128-bit UUIDs (Kia Arm Control service) =====================
 *   Service : K-I-A-0-...  Use a stable random base UUID — fixed here.
 * ======================================================================= */
/* 7d3b... (LE byte order) */
static const ble_uuid128_t SVC_UUID =
    BLE_UUID128_INIT(0x47, 0x49, 0x4B, 0x5F, 0x41, 0x52, 0x4D, 0x21,
                     0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
static const ble_uuid128_t CHR_RX_UUID = /* host -> device */
    BLE_UUID128_INIT(0x47, 0x49, 0x4B, 0x5F, 0x41, 0x52, 0x4D, 0x21,
                     0x00, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
static const ble_uuid128_t CHR_TX_UUID = /* device -> host (notify) */
    BLE_UUID128_INIT(0x47, 0x49, 0x4B, 0x5F, 0x41, 0x52, 0x4D, 0x21,
                     0x00, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00);

static uint16_t  s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t  s_tx_handle;
static uint8_t   s_own_addr_type;
static bool      s_subscribed;

static int gatt_rx(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_chr_def chrs[] = {
    {
        .uuid       = &CHR_RX_UUID.u,
        .access_cb  = gatt_rx,
        .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid       = &CHR_TX_UUID.u,
        .access_cb  = NULL,
        .flags      = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_tx_handle,
    },
    {0},
};

static const struct ble_gatt_svc_def svcs[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &SVC_UUID.u,
        .characteristics = chrs,
    },
    {0},
};

static int gatt_rx(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    KIA_UNUSED(conn); KIA_UNUSED(attr); KIA_UNUSED(arg);
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
    const uint8_t *data = OS_MBUF_DATA(ctxt->om, const uint8_t *);
    uint16_t       len  = OS_MBUF_PKTLEN(ctxt->om);
    svc_comm_on_rx(COMM_TRANSPORT_BLE, data, len);
    return 0;
}

static int ble_send(comm_transport_id_t t, const uint8_t *buf, size_t n)
{
    KIA_UNUSED(t);
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_subscribed) return -1;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(buf, n);
    if (!om) return -1;
    int rc = ble_gatts_notify_custom(s_conn_handle, s_tx_handle, om);
    return rc == 0 ? (int)n : -1;
}

static void advertise(void);

static int gap_event(struct ble_gap_event *e, void *arg)
{
    KIA_UNUSED(arg);
    switch (e->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (e->connect.status == 0) {
                s_conn_handle = e->connect.conn_handle;
                ESP_LOGI(TAG, "connected");
                esp_event_post(KIA_EVENT_BASE, KIA_EVT_LINK_UP, NULL, 0, 0);
            } else {
                advertise();
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "disconnect reason=%d", e->disconnect.reason);
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            s_subscribed  = false;
            esp_event_post(KIA_EVENT_BASE, KIA_EVT_LINK_DOWN, NULL, 0, 0);
            advertise();
            break;
        case BLE_GAP_EVENT_SUBSCRIBE:
            if (e->subscribe.attr_handle == s_tx_handle) {
                s_subscribed = e->subscribe.cur_notify;
                ESP_LOGI(TAG, "TX subscribed=%d", s_subscribed);
            }
            break;
        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU=%d", e->mtu.value);
            break;
        default: break;
    }
    return 0;
}

static void advertise(void)
{
    struct ble_hs_adv_fields fields = {0};
    fields.flags                    = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present    = 1;
    fields.tx_pwr_lvl               = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    const char *name                = ble_svc_gap_device_name();
    fields.name                     = (const uint8_t *)name;
    fields.name_len                 = strlen(name);
    fields.name_is_complete         = 1;
    fields.uuids128                 = (ble_uuid128_t *)&SVC_UUID;
    fields.num_uuids128             = 1;
    fields.uuids128_is_complete     = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc) ESP_LOGW(TAG, "adv_set_fields rc=%d", rc);

    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event, NULL);
    if (rc) ESP_LOGW(TAG, "adv_start rc=%d", rc);
}

static void on_reset(int reason) { ESP_LOGW(TAG, "ble reset %d", reason); }

static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    advertise();
}

static void host_task(void *arg)
{
    KIA_UNUSED(arg);
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t transport_ble_init(void)
{
    esp_err_t e = nimble_port_init();
    if (e != ESP_OK) { ESP_LOGE(TAG, "nimble init: 0x%x", e); return e; }

    ble_hs_cfg.reset_cb   = on_reset;
    ble_hs_cfg.sync_cb    = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    KIA_RET(ble_gatts_count_cfg(svcs));
    KIA_RET(ble_gatts_add_svcs(svcs));
    KIA_RET(ble_svc_gap_device_name_set("KiaArm"));

    svc_comm_register_sender(COMM_TRANSPORT_BLE, ble_send);

    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "BLE up");
    return ESP_OK;
}
