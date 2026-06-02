#include "svc_storage.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "app_config.h"
#include "drv_pot.h"
#include "drv_servo.h"
#include "kia_err.h"

static const char *TAG = "svc_storage";

#define KEY_SERVO_FMT "sv%d"
#define KEY_POT_FMT   "pt%d"

esp_err_t svc_storage_init(void)
{
    return svc_storage_load_calibration();
}

esp_err_t svc_storage_save_calibration(void)
{
    nvs_handle_t h;
    KIA_RET(nvs_open(KIA_NVS_NS_CAL, NVS_READWRITE, &h));
    for (uint8_t i = 0; i < DRV_SERVO_COUNT; ++i) {
        drv_servo_cal_t c;
        KIA_RET(drv_servo_get_calibration(i, &c));
        char k[8];
        snprintf(k, sizeof k, KEY_SERVO_FMT, i);
        nvs_set_blob(h, k, &c, sizeof c);
    }
    for (uint8_t i = 0; i < DRV_POT_COUNT; ++i) {
        drv_pot_cal_t c;
        KIA_RET(drv_pot_get_calibration(i, &c));
        char k[8];
        snprintf(k, sizeof k, KEY_POT_FMT, i);
        nvs_set_blob(h, k, &c, sizeof c);
    }
    esp_err_t e = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "calibration saved (%d)", e);
    return e;
}

esp_err_t svc_storage_load_calibration(void)
{
    nvs_handle_t h;
    esp_err_t    e = nvs_open(KIA_NVS_NS_CAL, NVS_READONLY, &h);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "no calibration in NVS (first boot)");
        return ESP_OK;
    }
    KIA_RET(e);

    for (uint8_t i = 0; i < DRV_SERVO_COUNT; ++i) {
        drv_servo_cal_t c;
        size_t sz = sizeof c;
        char k[8]; snprintf(k, sizeof k, KEY_SERVO_FMT, i);
        if (nvs_get_blob(h, k, &c, &sz) == ESP_OK && sz == sizeof c) {
            drv_servo_set_calibration(i, &c);
        }
    }
    for (uint8_t i = 0; i < DRV_POT_COUNT; ++i) {
        drv_pot_cal_t c;
        size_t sz = sizeof c;
        char k[8]; snprintf(k, sizeof k, KEY_POT_FMT, i);
        if (nvs_get_blob(h, k, &c, &sz) == ESP_OK && sz == sizeof c) {
            drv_pot_set_calibration(i, c);
        }
    }
    nvs_close(h);
    return ESP_OK;
}

esp_err_t svc_storage_factory_reset(void)
{
    nvs_handle_t h;
    KIA_RET(nvs_open(KIA_NVS_NS_CAL, NVS_READWRITE, &h));
    esp_err_t e = nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
    return e;
}
