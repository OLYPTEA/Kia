#include "svc_comm.h"

#include "esp_log.h"

#include "kia_err.h"

esp_err_t transport_usb_cdc_init(void);
esp_err_t transport_ble_init(void);

static const char *TAG = "svc_comm";

esp_err_t svc_comm_init(void)
{
    KIA_RET(transport_usb_cdc_init());
    esp_err_t ble = transport_ble_init();
    if (ble != ESP_OK) ESP_LOGW(TAG, "BLE init failed 0x%x (USB-only mode)", ble);
    ESP_LOGI(TAG, "comm up");
    return ESP_OK;
}
