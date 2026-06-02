#include "drv_buzzer.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "kia_err.h"

#define BUZ_TIMER   LEDC_TIMER_3
#define BUZ_CHAN    LEDC_CHANNEL_7
#define BUZ_SPEED   LEDC_LOW_SPEED_MODE

static bool s_ready;

esp_err_t drv_buzzer_init(void)
{
    if (s_ready) return ESP_OK;
    ledc_timer_config_t t = {
        .speed_mode      = BUZ_SPEED,
        .timer_num       = BUZ_TIMER,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz         = 2000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    KIA_RET(ledc_timer_config(&t));

    ledc_channel_config_t c = {
        .gpio_num   = KIA_GPIO_BUZZER,
        .speed_mode = BUZ_SPEED,
        .channel    = BUZ_CHAN,
        .timer_sel  = BUZ_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    KIA_RET(ledc_channel_config(&c));
    s_ready = true;
    return ESP_OK;
}

esp_err_t drv_buzzer_tone(uint32_t freq_hz, uint32_t duration_ms)
{
    KIA_RET_IF(!s_ready, ESP_ERR_INVALID_STATE);
    if (freq_hz == 0) {
        ledc_set_duty(BUZ_SPEED, BUZ_CHAN, 0);
        ledc_update_duty(BUZ_SPEED, BUZ_CHAN);
        return ESP_OK;
    }
    ledc_set_freq(BUZ_SPEED, BUZ_TIMER, freq_hz);
    ledc_set_duty(BUZ_SPEED, BUZ_CHAN, 512); /* 50% on 10-bit */
    ledc_update_duty(BUZ_SPEED, BUZ_CHAN);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    ledc_set_duty(BUZ_SPEED, BUZ_CHAN, 0);
    ledc_update_duty(BUZ_SPEED, BUZ_CHAN);
    return ESP_OK;
}

esp_err_t drv_buzzer_beep_ok(void)
{
    drv_buzzer_tone(2200, 60);
    vTaskDelay(pdMS_TO_TICKS(40));
    return drv_buzzer_tone(2800, 80);
}

esp_err_t drv_buzzer_beep_err(void)
{
    drv_buzzer_tone(900, 120);
    vTaskDelay(pdMS_TO_TICKS(40));
    return drv_buzzer_tone(700, 180);
}

esp_err_t drv_buzzer_beep_fault(void)
{
    for (int i = 0; i < 3; ++i) {
        drv_buzzer_tone(1200, 100);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    return ESP_OK;
}
