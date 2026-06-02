#include "svc_ota.h"

#include <string.h>

#include "esp_app_format.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "app_events.h"
#include "crc32.h"
#include "kia_err.h"

static const char *TAG = "svc_ota";

static struct {
    svc_ota_state_t        state;
    svc_ota_session_t      sess;
    esp_ota_handle_t       hnd;
    const esp_partition_t *part;
    uint32_t               crc_running;
    SemaphoreHandle_t      mtx;
    comm_transport_id_t    owner;
    int64_t                t_started_us;
} S;

static void broadcast_progress(void)
{
    /* Broadcast a generic event; host queries via OP_GET_STATUS or telemetry. */
    esp_event_post(KIA_EVENT_BASE, KIA_EVT_OTA_BEGIN + (S.state == OTA_IDLE ? 1 : 0),
                   &S.sess, sizeof S.sess, 0);
}

esp_err_t svc_ota_init(void)
{
    if (!S.mtx) S.mtx = xSemaphoreCreateMutex();
    return S.mtx ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t svc_ota_begin(comm_transport_id_t t, uint32_t total, uint16_t chunk_sz, uint32_t crc_exp)
{
    if (!S.mtx) svc_ota_init();
    xSemaphoreTake(S.mtx, portMAX_DELAY);
    esp_err_t rc = ESP_OK;

    if (S.state != OTA_IDLE && S.state != OTA_FAILED) { rc = ESP_ERR_INVALID_STATE; goto out; }
    if (total == 0 || chunk_sz == 0)                   { rc = ESP_ERR_INVALID_ARG;   goto out; }

    S.part = esp_ota_get_next_update_partition(NULL);
    if (!S.part) { rc = ESP_ERR_NOT_FOUND; goto out; }

    rc = esp_ota_begin(S.part, total, &S.hnd);
    if (rc != ESP_OK) goto out;

    S.sess.total_bytes    = total;
    S.sess.chunk_size     = chunk_sz;
    S.sess.received_bytes = 0;
    S.sess.crc32_expected = crc_exp;
    S.crc_running         = 0;
    S.owner               = t;
    S.state               = OTA_RECEIVING;
    S.t_started_us        = esp_log_timestamp() * 1000LL;
    ESP_LOGI(TAG, "OTA begin total=%lu chunk=%u part=%s", (unsigned long)total, chunk_sz, S.part->label);
    esp_event_post(KIA_EVENT_BASE, KIA_EVT_OTA_BEGIN, NULL, 0, 0);
out:
    xSemaphoreGive(S.mtx);
    return rc;
}

esp_err_t svc_ota_chunk(comm_transport_id_t t, uint32_t offset, const uint8_t *data, size_t n)
{
    xSemaphoreTake(S.mtx, portMAX_DELAY);
    esp_err_t rc = ESP_OK;
    if (S.state != OTA_RECEIVING)        { rc = ESP_ERR_INVALID_STATE; goto out; }
    if (t != S.owner)                    { rc = ESP_ERR_INVALID_STATE; goto out; }
    if (offset != S.sess.received_bytes) { rc = ESP_ERR_INVALID_ARG;   goto out; }
    if (offset + n > S.sess.total_bytes) { rc = ESP_ERR_INVALID_SIZE;  goto out; }

    rc = esp_ota_write(S.hnd, data, n);
    if (rc != ESP_OK) { S.state = OTA_FAILED; goto out; }

    S.crc_running          = kia_crc32_update(S.crc_running, data, n);
    S.sess.received_bytes += n;

    if ((S.sess.received_bytes & 0x3FFF) == 0) broadcast_progress();
out:
    xSemaphoreGive(S.mtx);
    return rc;
}

esp_err_t svc_ota_end(comm_transport_id_t t)
{
    xSemaphoreTake(S.mtx, portMAX_DELAY);
    esp_err_t rc = ESP_OK;
    if (S.state != OTA_RECEIVING)                       { rc = ESP_ERR_INVALID_STATE; goto out; }
    if (t != S.owner)                                   { rc = ESP_ERR_INVALID_STATE; goto out; }
    if (S.sess.received_bytes != S.sess.total_bytes)    { rc = ESP_ERR_INVALID_SIZE;  goto out; }
    if (S.sess.crc32_expected && S.crc_running != S.sess.crc32_expected) {
        ESP_LOGE(TAG, "CRC mismatch: got %08lx exp %08lx",
                 (unsigned long)S.crc_running, (unsigned long)S.sess.crc32_expected);
        rc = KIA_ERR_CRC;
        goto out;
    }
    S.state = OTA_FINALISING;
    rc = esp_ota_end(S.hnd);
    if (rc != ESP_OK) { S.state = OTA_FAILED; goto out; }
    rc = esp_ota_set_boot_partition(S.part);
    if (rc != ESP_OK) { S.state = OTA_FAILED; goto out; }
    ESP_LOGI(TAG, "OTA done — reboot to apply");
    esp_event_post(KIA_EVENT_BASE, KIA_EVT_OTA_DONE, NULL, 0, 0);
    S.state = OTA_IDLE;
out:
    if (rc != ESP_OK) S.state = OTA_FAILED;
    xSemaphoreGive(S.mtx);
    return rc;
}

esp_err_t svc_ota_abort(void)
{
    xSemaphoreTake(S.mtx, portMAX_DELAY);
    if (S.state == OTA_RECEIVING || S.state == OTA_FINALISING) {
        esp_ota_abort(S.hnd);
    }
    memset(&S.sess, 0, sizeof S.sess);
    S.state       = OTA_IDLE;
    S.crc_running = 0;
    xSemaphoreGive(S.mtx);
    return ESP_OK;
}

svc_ota_state_t svc_ota_get_state(void)        { return S.state; }
uint32_t        svc_ota_received_bytes(void)   { return S.sess.received_bytes; }
uint32_t        svc_ota_total_bytes(void)      { return S.sess.total_bytes; }
