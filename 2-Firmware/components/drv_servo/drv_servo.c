#include "drv_servo.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "app_config.h"
#include "kia_err.h"

static const char *TAG = "drv_servo";

typedef struct {
    mcpwm_cmpr_handle_t cmpr;
    mcpwm_gen_handle_t  gen;
    drv_servo_cal_t     cal;
    float               cur_deg;
} servo_chan_t;

static struct {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t  oper[3];
    servo_chan_t         chan[DRV_SERVO_COUNT];
    SemaphoreHandle_t    mtx;
    bool                 enabled;
    bool                 ready;
} S;

static const int s_gpio[DRV_SERVO_COUNT] = {
    KIA_GPIO_PWM_BASE, KIA_GPIO_PWM_SHOULDER, KIA_GPIO_PWM_ELBOW, KIA_GPIO_PWM_WRIST, KIA_GPIO_PWM_GRIP,
};

static const drv_servo_cal_t s_default_cal[DRV_SERVO_COUNT] = {
    [KIA_J_BASE]     = {500, 2500, -90, 90, 0, false},
    [KIA_J_SHOULDER] = {500, 2500, 0, 180, -90, false},
    [KIA_J_ELBOW]    = {500, 2500, -135, 135, 0, false},
    [KIA_J_WRIST]    = {500, 2500, -90, 90, 0, false},
    [KIA_J_GRIP]     = {500, 2500, 0, 90, 0, false},
};

static uint32_t deg_to_pulse(uint8_t idx, float deg)
{
    const drv_servo_cal_t *c = &S.chan[idx].cal;
    float                  a = c->inverted ? -deg : deg;
    a += c->offset_deg;
    a            = kia_clampf(a, c->angle_min_deg, c->angle_max_deg);
    float t      = (a - c->angle_min_deg) / (c->angle_max_deg - c->angle_min_deg);
    float pulse  = (float)c->pulse_min_us + t * (float)(c->pulse_max_us - c->pulse_min_us);
    return (uint32_t)pulse;
}

esp_err_t drv_servo_init(void)
{
    if (S.ready) return ESP_OK;
    memset(&S, 0, sizeof(S));

    gpio_config_t en = {
        .pin_bit_mask = 1ULL << KIA_GPIO_SERVO_EN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = 0,
        .pull_down_en = 1,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    KIA_RET(gpio_config(&en));
    gpio_set_level(KIA_GPIO_SERVO_EN, 0);

    S.mtx = xSemaphoreCreateMutex();
    KIA_RET_IF(!S.mtx, ESP_ERR_NO_MEM);

    mcpwm_timer_config_t tcfg = {
        .group_id     = 0,
        .clk_source   = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = KIA_SERVO_RESOLUTION_HZ,
        .period_ticks  = KIA_SERVO_RESOLUTION_HZ / KIA_SERVO_PWM_FREQ_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    KIA_RET(mcpwm_new_timer(&tcfg, &S.timer));

    for (int i = 0; i < 3; ++i) {
        mcpwm_operator_config_t ocfg = {.group_id = 0};
        KIA_RET(mcpwm_new_operator(&ocfg, &S.oper[i]));
        KIA_RET(mcpwm_operator_connect_timer(S.oper[i], S.timer));
    }

    for (int i = 0; i < DRV_SERVO_COUNT; ++i) {
        S.chan[i].cal     = s_default_cal[i];
        S.chan[i].cur_deg = 0;

        mcpwm_comparator_config_t ccfg = {.flags.update_cmp_on_tez = true};
        KIA_RET(mcpwm_new_comparator(S.oper[i / 2], &ccfg, &S.chan[i].cmpr));

        mcpwm_generator_config_t gcfg = {.gen_gpio_num = s_gpio[i]};
        KIA_RET(mcpwm_new_generator(S.oper[i / 2], &gcfg, &S.chan[i].gen));

        KIA_RET(mcpwm_comparator_set_compare_value(S.chan[i].cmpr, KIA_SERVO_PULSE_NEUTRAL_US));

        KIA_RET(mcpwm_generator_set_action_on_timer_event(
            S.chan[i].gen, MCPWM_GEN_TIMER_EVENT_ACTION(
                               MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        KIA_RET(mcpwm_generator_set_action_on_compare_event(
            S.chan[i].gen, MCPWM_GEN_COMPARE_EVENT_ACTION(
                               MCPWM_TIMER_DIRECTION_UP, S.chan[i].cmpr, MCPWM_GEN_ACTION_LOW)));
    }

    KIA_RET(mcpwm_timer_enable(S.timer));
    KIA_RET(mcpwm_timer_start_stop(S.timer, MCPWM_TIMER_START_NO_STOP));

    S.ready = true;
    ESP_LOGI(TAG, "MCPWM %d channels initialised", DRV_SERVO_COUNT);
    return ESP_OK;
}

esp_err_t drv_servo_enable(bool on)
{
    KIA_RET_IF(!S.ready, ESP_ERR_INVALID_STATE);
    gpio_set_level(KIA_GPIO_SERVO_EN, on ? 1 : 0);
    S.enabled = on;
    return ESP_OK;
}

bool drv_servo_is_enabled(void) { return S.enabled; }

esp_err_t drv_servo_set_calibration(uint8_t idx, const drv_servo_cal_t *cal)
{
    KIA_RET_IF(idx >= DRV_SERVO_COUNT || !cal, ESP_ERR_INVALID_ARG);
    xSemaphoreTake(S.mtx, portMAX_DELAY);
    S.chan[idx].cal = *cal;
    xSemaphoreGive(S.mtx);
    return ESP_OK;
}

esp_err_t drv_servo_get_calibration(uint8_t idx, drv_servo_cal_t *out)
{
    KIA_RET_IF(idx >= DRV_SERVO_COUNT || !out, ESP_ERR_INVALID_ARG);
    xSemaphoreTake(S.mtx, portMAX_DELAY);
    *out = S.chan[idx].cal;
    xSemaphoreGive(S.mtx);
    return ESP_OK;
}

esp_err_t drv_servo_set_angle(uint8_t idx, float deg)
{
    KIA_RET_IF(idx >= DRV_SERVO_COUNT, ESP_ERR_INVALID_ARG);
    KIA_RET_IF(!S.ready, ESP_ERR_INVALID_STATE);
    uint32_t us = deg_to_pulse(idx, deg);
    esp_err_t e = mcpwm_comparator_set_compare_value(S.chan[idx].cmpr, us);
    if (e == ESP_OK) S.chan[idx].cur_deg = deg;
    return e;
}

float drv_servo_get_angle(uint8_t idx)
{
    if (idx >= DRV_SERVO_COUNT) return 0;
    return S.chan[idx].cur_deg;
}

esp_err_t drv_servo_set_pulse_us(uint8_t idx, uint32_t us)
{
    KIA_RET_IF(idx >= DRV_SERVO_COUNT, ESP_ERR_INVALID_ARG);
    KIA_RET_IF(us < KIA_SERVO_PULSE_MIN_US || us > KIA_SERVO_PULSE_MAX_US, ESP_ERR_INVALID_ARG);
    return mcpwm_comparator_set_compare_value(S.chan[idx].cmpr, us);
}

esp_err_t drv_servo_home_all(void)
{
    for (uint8_t i = 0; i < DRV_SERVO_COUNT; ++i) {
        const drv_servo_cal_t *c   = &S.chan[i].cal;
        float                  mid = 0.5f * (c->angle_min_deg + c->angle_max_deg) - c->offset_deg;
        KIA_RET(drv_servo_set_angle(i, mid));
    }
    return ESP_OK;
}
