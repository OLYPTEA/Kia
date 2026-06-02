#include "drv_buttons.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_config.h"
#include "app_events.h"
#include "esp_event.h"
#include "kia_err.h"

#define DEBOUNCE_US 8000

static const char *TAG = "drv_btn";

typedef struct {
    int          gpio;
    bool         active_low;
    bool         pulled_up;
    bool         last_raw;
    bool         state;
    int64_t      last_edge_us;
    drv_btn_cb_t cb;
    void        *user;
} btn_t;

static struct {
    btn_t            b[DRV_BTN__COUNT];
    QueueHandle_t    q;
    TaskHandle_t     task;
    bool             ready;
} S;

static IRAM_ATTR void isr_handler(void *arg)
{
    drv_btn_id_t id = (drv_btn_id_t)(uintptr_t)arg;
    BaseType_t   hp = pdFALSE;
    xQueueSendFromISR(S.q, &id, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static void task(void *arg)
{
    KIA_UNUSED(arg);
    drv_btn_id_t id;
    while (xQueueReceive(S.q, &id, portMAX_DELAY) == pdTRUE) {
        btn_t  *b   = &S.b[id];
        int64_t now = esp_timer_get_time();
        if ((now - b->last_edge_us) < DEBOUNCE_US) continue;
        b->last_edge_us = now;
        int     lvl     = gpio_get_level(b->gpio);
        bool    raw     = b->active_low ? (lvl == 0) : (lvl != 0);
        if (raw != b->state) {
            b->state = raw;
            if (b->cb) b->cb(id, raw, b->user);
            if (id == DRV_BTN_ESTOP) {
                esp_event_post(KIA_EVENT_BASE,
                               raw ? KIA_EVT_ESTOP_TRIGGERED : KIA_EVT_ESTOP_CLEARED, NULL, 0, 0);
            }
        }
    }
}

esp_err_t drv_buttons_init(void)
{
    if (S.ready) return ESP_OK;
    memset(&S, 0, sizeof(S));

    S.b[DRV_BTN_ESTOP] = (btn_t){.gpio = KIA_GPIO_ESTOP, .active_low = true, .pulled_up = true};
    S.b[DRV_BTN_MODE]  = (btn_t){.gpio = KIA_GPIO_MODE_SEL, .active_low = true, .pulled_up = true};

    for (int i = 0; i < DRV_BTN__COUNT; ++i) {
        gpio_config_t c = {
            .pin_bit_mask = 1ULL << S.b[i].gpio,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = S.b[i].pulled_up,
            .pull_down_en = !S.b[i].pulled_up,
            .intr_type    = GPIO_INTR_ANYEDGE,
        };
        KIA_RET(gpio_config(&c));
        S.b[i].state = (gpio_get_level(S.b[i].gpio) == 0) == S.b[i].active_low;
    }

    S.q = xQueueCreate(8, sizeof(drv_btn_id_t));
    KIA_RET_IF(!S.q, ESP_ERR_NO_MEM);

    KIA_RET(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    for (int i = 0; i < DRV_BTN__COUNT; ++i)
        KIA_RET(gpio_isr_handler_add(S.b[i].gpio, isr_handler, (void *)(uintptr_t)i));

    BaseType_t ok = xTaskCreatePinnedToCore(task, "btn", 3072, NULL, 10, &S.task, KIA_CORE_IO);
    KIA_RET_IF(ok != pdPASS, ESP_ERR_NO_MEM);

    S.ready = true;
    ESP_LOGI(TAG, "buttons up (E-STOP=%d MODE=%d)", S.b[DRV_BTN_ESTOP].state, S.b[DRV_BTN_MODE].state);
    return ESP_OK;
}

esp_err_t drv_buttons_register(drv_btn_id_t id, drv_btn_cb_t cb, void *user)
{
    KIA_RET_IF(id >= DRV_BTN__COUNT, ESP_ERR_INVALID_ARG);
    S.b[id].cb   = cb;
    S.b[id].user = user;
    return ESP_OK;
}

bool drv_buttons_get_state(drv_btn_id_t id)
{
    if (id >= DRV_BTN__COUNT) return false;
    return S.b[id].state;
}
