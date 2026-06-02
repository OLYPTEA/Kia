#include "svc_safety.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_events.h"
#include "drv_buzzer.h"
#include "drv_ina219.h"
#include "drv_led.h"
#include "drv_servo.h"
#include "kia_err.h"

static const char *TAG = "svc_safety";

static struct {
    safety_fault_t fault;
    int32_t        last_i_ma;
    int32_t        avg_i_ma_q8;
    int64_t        avg_started_us;
    TaskHandle_t   task;
    bool           ready;
    bool           armed;
} S;

static void apply_lockdown(void)
{
    drv_servo_enable(false);
    drv_led_set_fault(true);
    drv_led_set_mode_color(DRV_LED_COLOR_FAULT);
}

static void task(void *arg)
{
    KIA_UNUSED(arg);
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    const TickType_t period = pdMS_TO_TICKS(1000 / KIA_SAFETY_RATE_HZ);
    TickType_t       wake   = xTaskGetTickCount();
    drv_ina219_sample_t s;

    while (true) {
        esp_task_wdt_reset();

        if (drv_ina219_read(&s) == ESP_OK) {
            S.last_i_ma = s.current_ma;
            /* Q8 EMA on current, ~250 ms window @ 200 Hz */
            int32_t a   = S.avg_i_ma_q8;
            int32_t x   = s.current_ma << 8;
            S.avg_i_ma_q8 = a + ((x - a) >> 6);

            if (s.current_ma > KIA_SAFETY_I_LIMIT_MA) {
                svc_safety_trip(SAFETY_OVERCURRENT);
                esp_event_post(KIA_EVENT_BASE, KIA_EVT_OVERCURRENT, NULL, 0, 0);
            } else if (s.bus_mv > 0 && s.bus_mv < KIA_SAFETY_V_BUS_MIN_MV) {
                svc_safety_trip(SAFETY_UNDERVOLT);
            } else if ((S.avg_i_ma_q8 >> 8) > KIA_SAFETY_I_AVG_LIMIT_MA) {
                svc_safety_trip(SAFETY_OVERCURRENT);
            }
        }

        vTaskDelayUntil(&wake, period);
    }
}

static void on_estop(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    KIA_UNUSED(arg); KIA_UNUSED(base); KIA_UNUSED(data);
    if (id == KIA_EVT_ESTOP_TRIGGERED) {
        svc_safety_trip(SAFETY_ESTOP);
    }
}

esp_err_t svc_safety_init(void)
{
    if (S.ready) return ESP_OK;
    memset(&S, 0, sizeof(S));
    S.fault = SAFETY_OK;

    ESP_ERROR_CHECK(esp_event_handler_register(KIA_EVENT_BASE, KIA_EVT_ESTOP_TRIGGERED, on_estop, NULL));

    BaseType_t ok = xTaskCreatePinnedToCore(task, "safety", KIA_STACK_SAFETY, NULL, KIA_PRIO_SAFETY,
                                            &S.task, KIA_CORE_RT);
    KIA_RET_IF(ok != pdPASS, ESP_ERR_NO_MEM);
    S.ready = true;
    ESP_LOGI(TAG, "safety supervisor up @ %d Hz", KIA_SAFETY_RATE_HZ);
    return ESP_OK;
}

esp_err_t svc_safety_arm(void)
{
    if (S.fault != SAFETY_OK) return KIA_ERR_FAULT_ACTIVE;
    drv_servo_enable(true);
    S.armed = true;
    return ESP_OK;
}

esp_err_t svc_safety_trip(safety_fault_t f)
{
    if (S.fault == SAFETY_OK) {
        S.fault = f;
        apply_lockdown();
        kia_evt_fault_t fault = {.code = f};
        esp_event_post(KIA_EVENT_BASE, KIA_EVT_FAULT_RAISED, &fault, sizeof fault, 0);
        ESP_LOGE(TAG, "TRIP cause=%d I=%ld mA", f, (long)S.last_i_ma);
    }
    return ESP_OK;
}

esp_err_t svc_safety_clear(void)
{
    if (S.fault == SAFETY_ESTOP) {
        return KIA_ERR_FAULT_ACTIVE; /* must be released by HW button */
    }
    S.fault = SAFETY_OK;
    drv_led_set_fault(false);
    drv_led_set_mode_color(DRV_LED_COLOR_READY);
    esp_event_post(KIA_EVENT_BASE, KIA_EVT_FAULT_CLEARED, NULL, 0, 0);
    return ESP_OK;
}

bool           svc_safety_is_ok(void)        { return S.fault == SAFETY_OK; }
safety_fault_t svc_safety_last_fault(void)   { return S.fault; }
int32_t        svc_safety_last_current_ma(void) { return S.last_i_ma; }
