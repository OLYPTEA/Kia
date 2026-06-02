#include "app_mode_fsm.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_events.h"
#include "drv_buttons.h"
#include "drv_led.h"
#include "kia_err.h"
#include "svc_motion.h"
#include "svc_safety.h"

static const char *TAG = "mode_fsm";

static struct {
    app_mode_t   cur;
    TaskHandle_t task;
    bool         ready;
} S;

static void enter(app_mode_t next)
{
    if (next == S.cur) return;
    app_mode_t prev = S.cur;
    S.cur = next;
    ESP_LOGI(TAG, "%d -> %d", prev, next);

    switch (next) {
        case APP_MODE_MANUAL:
            drv_led_set_mode_color(DRV_LED_COLOR_MANUAL);
            svc_safety_arm();
            svc_motion_set_source(MOTION_SRC_MANUAL_POT);
            break;
        case APP_MODE_APP:
            drv_led_set_mode_color(DRV_LED_COLOR_APP);
            svc_safety_arm();
            svc_motion_set_source(MOTION_SRC_HOLD);
            break;
        case APP_MODE_CALIB:
            drv_led_set_mode_color(DRV_LED_COLOR_CAL);
            svc_motion_hold();
            break;
        case APP_MODE_FAULT:
            drv_led_set_mode_color(DRV_LED_COLOR_FAULT);
            svc_motion_hold();
            break;
        case APP_MODE_ESTOP:
            drv_led_set_mode_color(DRV_LED_COLOR_ESTOP);
            svc_motion_hold();
            break;
        case APP_MODE_IDLE:
        default:
            drv_led_set_mode_color(DRV_LED_COLOR_READY);
            svc_motion_hold();
            break;
    }
    kia_evt_mode_t evt = {.prev = prev, .next = next};
    esp_event_post(KIA_EVENT_BASE, KIA_EVT_MODE_CHANGED, &evt, sizeof evt, 0);
}

static void on_evt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    KIA_UNUSED(arg); KIA_UNUSED(base); KIA_UNUSED(data);
    switch (id) {
        case KIA_EVT_ESTOP_TRIGGERED: enter(APP_MODE_ESTOP); break;
        case KIA_EVT_ESTOP_CLEARED:   if (S.cur == APP_MODE_ESTOP) enter(APP_MODE_IDLE); break;
        case KIA_EVT_FAULT_RAISED:    if (S.cur != APP_MODE_ESTOP) enter(APP_MODE_FAULT); break;
        case KIA_EVT_FAULT_CLEARED:   if (S.cur == APP_MODE_FAULT) enter(APP_MODE_IDLE); break;
        default: break;
    }
}

static void task(void *arg)
{
    KIA_UNUSED(arg);
    bool last_mode_btn = drv_buttons_get_state(DRV_BTN_MODE);
    while (true) {
        if (S.cur == APP_MODE_IDLE || S.cur == APP_MODE_MANUAL || S.cur == APP_MODE_APP) {
            bool mb = drv_buttons_get_state(DRV_BTN_MODE);
            if (mb != last_mode_btn) {
                enter(mb ? APP_MODE_MANUAL : APP_MODE_APP);
                last_mode_btn = mb;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t app_mode_fsm_init(void)
{
    if (S.ready) return ESP_OK;
    memset(&S, 0, sizeof(S));
    ESP_ERROR_CHECK(esp_event_handler_register(KIA_EVENT_BASE, ESP_EVENT_ANY_ID, on_evt, NULL));
    BaseType_t ok = xTaskCreatePinnedToCore(task, "mode_fsm", KIA_STACK_FSM, NULL, KIA_PRIO_MODE_FSM,
                                            &S.task, KIA_CORE_IO);
    KIA_RET_IF(ok != pdPASS, ESP_ERR_NO_MEM);
    S.ready = true;
    enter(APP_MODE_IDLE);
    return ESP_OK;
}

app_mode_t app_mode_fsm_current(void) { return S.cur; }

esp_err_t app_mode_fsm_request(app_mode_t target)
{
    if (S.cur == APP_MODE_ESTOP && target != APP_MODE_ESTOP) return KIA_ERR_FAULT_ACTIVE;
    enter(target);
    return ESP_OK;
}
