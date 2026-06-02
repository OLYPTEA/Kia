#include <stdio.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "app_config.h"
#include "app_events.h"

#include "app_mode_fsm.h"
#include "drv_buttons.h"
#include "drv_buzzer.h"
#include "drv_ina219.h"
#include "drv_led.h"
#include "drv_pot.h"
#include "drv_servo.h"
#include "kia_err.h"
#include "svc_comm.h"
#include "svc_motion.h"
#include "svc_ota.h"
#include "svc_safety.h"
#include "svc_storage.h"
#include "svc_telemetry.h"

ESP_EVENT_DEFINE_BASE(KIA_EVENT_BASE);

static const char *TAG = "main";

static void on_kia_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    (void)base;
    (void)data;
    switch (id) {
        case KIA_EVT_ESTOP_TRIGGERED: ESP_LOGW(TAG, "ESTOP TRIGGERED"); break;
        case KIA_EVT_ESTOP_CLEARED: ESP_LOGI(TAG, "ESTOP cleared"); break;
        case KIA_EVT_FAULT_RAISED: ESP_LOGE(TAG, "FAULT raised"); break;
        case KIA_EVT_FAULT_CLEARED: ESP_LOGI(TAG, "Fault cleared"); break;
        case KIA_EVT_MODE_CHANGED: ESP_LOGI(TAG, "Mode changed"); break;
        case KIA_EVT_OVERCURRENT: SP_LOGW(TAG, "OVERCURRENT"); break;
        default: break;
    }
}

static void boot_banner(void)
{
    ESP_LOGI(TAG, "Kia Arm Controller fw v%d.%d.%d", KIA_FW_VERSION_MAJOR, KIA_FW_VERSION_MINOR,
             KIA_FW_VERSION_PATCH);
    ESP_LOGI(TAG, "Build: %s %s", __DATE__, __TIME__);
}

void app_main(void)
{
    boot_banner();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(KIA_EVENT_BASE, ESP_EVENT_ANY_ID, on_kia_event, NULL));

    KIA_CHK(svc_storage_init());
    KIA_CHK(drv_led_init());
    drv_led_set_mode_color(DRV_LED_COLOR_BOOT);

    KIA_CHK(drv_buzzer_init());
    KIA_CHK(drv_buttons_init());
    KIA_CHK(drv_servo_init());
    KIA_CHK(drv_pot_init());
    KIA_CHK(drv_ina219_init());

    KIA_CHK(svc_safety_init());      /* must come before motion */
    KIA_CHK(svc_motion_init());
    KIA_CHK(app_mode_fsm_init());
    KIA_CHK(svc_ota_init());
    KIA_CHK(svc_telemetry_init());
    KIA_CHK(svc_comm_init());

    drv_buzzer_beep_ok();
    drv_led_set_mode_color(DRV_LED_COLOR_READY);

    ESP_ERROR_CHECK(esp_event_post(KIA_EVENT_BASE, KIA_EVT_BOOT_DONE, NULL, 0, portMAX_DELAY));
    ESP_LOGI(TAG, "Boot complete");

    /* app_main can return on IDF v5+; idle is handled by tasks. */
}
