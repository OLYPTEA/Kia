#include "drv_pot.h"

#include <string.h>

#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "kia_err.h"

static const char *TAG = "drv_pot";

static const adc_channel_t s_chans[DRV_POT_COUNT] = {
    ADC_CHANNEL_0, /* GPIO1 */
    ADC_CHANNEL_1, /* GPIO2 */
    ADC_CHANNEL_3, /* GPIO4 */
    ADC_CHANNEL_4, /* GPIO5 */
    ADC_CHANNEL_5, /* GPIO6 */
};

typedef struct {
    uint32_t acc;
    uint32_t n;
    uint16_t filt;     /* EMA filtered (Q0, 0..4095) */
    uint16_t stable;   /* hysteresis output */
    uint16_t last_emit;
    drv_pot_cal_t cal;
} pot_state_t;

static struct {
    adc_continuous_handle_t  hnd;
    TaskHandle_t             task;
    pot_state_t              pot[DRV_POT_COUNT];
    bool                     ready;
    bool                     running;
    volatile uint32_t        tick;
} S;

static inline uint8_t chan_to_idx(adc_channel_t c)
{
    for (uint8_t i = 0; i < DRV_POT_COUNT; ++i)
        if (s_chans[i] == c) return i;
    return 0xFF;
}

static void apply_ema(pot_state_t *p, uint16_t sample)
{
    /* EMA: y = y + alpha * (x - y), alpha in Q16 */
    int32_t y = p->filt;
    int32_t d = (int32_t)sample - y;
    y += (d * (int32_t)KIA_ADC_EMA_ALPHA_Q16) >> 16;
    if (y < 0) y = 0;
    if (y > 4095) y = 4095;
    p->filt = (uint16_t)y;
}

static void apply_hysteresis(pot_state_t *p)
{
    /* Hystérésis adaptative : deadband = base + gain * |delta| */
    int32_t  delta = (int32_t)p->filt - (int32_t)p->stable;
    uint32_t adapt = KIA_ADC_HYST_BASE_LSB +
                     ((uint32_t)(delta < 0 ? -delta : delta) * KIA_ADC_HYST_GAIN_Q16 >> 16);
    if ((delta > 0 && (uint32_t)delta > adapt) || (delta < 0 && (uint32_t)-delta > adapt)) {
        p->stable = p->filt;
    }
}

static void pot_task(void *arg)
{
    KIA_UNUSED(arg);
    const size_t   buf_bytes = 1024;
    uint8_t       *buf       = pvPortMalloc(buf_bytes);
    configASSERT(buf);

    while (S.running) {
        uint32_t got = 0;
        esp_err_t e = adc_continuous_read(S.hnd, buf, buf_bytes, &got, 100);
        if (e == ESP_ERR_TIMEOUT) continue;
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "adc read err 0x%x", e);
            continue;
        }
        for (size_t i = 0; i < got; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *d = (adc_digi_output_data_t *)(buf + i);
            uint8_t  k = chan_to_idx(d->type2.channel);
            if (k >= DRV_POT_COUNT) continue;
            S.pot[k].acc += d->type2.data;
            if (++S.pot[k].n >= KIA_ADC_OVERSAMPLE) {
                uint16_t avg = (uint16_t)(S.pot[k].acc / S.pot[k].n);
                S.pot[k].acc = 0;
                S.pot[k].n   = 0;
                apply_ema(&S.pot[k], avg);
                apply_hysteresis(&S.pot[k]);
            }
        }
        ++S.tick;
    }

    vPortFree(buf);
    vTaskDelete(NULL);
}

esp_err_t drv_pot_init(void)
{
    if (S.ready) return ESP_OK;
    memset(&S, 0, sizeof(S));

    for (int i = 0; i < DRV_POT_COUNT; ++i) {
        S.pot[i].cal.raw_min = 80;
        S.pot[i].cal.raw_max = 4015;
    }

    adc_continuous_handle_cfg_t hcfg = {
        .max_store_buf_size = 4096,
        .conv_frame_size    = 256,
    };
    KIA_RET(adc_continuous_new_handle(&hcfg, &S.hnd));

    adc_digi_pattern_config_t pat[DRV_POT_COUNT];
    for (int i = 0; i < DRV_POT_COUNT; ++i) {
        pat[i] = (adc_digi_pattern_config_t){
            .atten     = KIA_ADC_ATTEN,
            .channel   = s_chans[i],
            .unit      = KIA_ADC_UNIT,
            .bit_width = KIA_ADC_BITWIDTH,
        };
    }

    adc_continuous_config_t ccfg = {
        .pattern_num    = DRV_POT_COUNT,
        .adc_pattern    = pat,
        .sample_freq_hz = KIA_ADC_SAMPLE_RATE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    KIA_RET(adc_continuous_config(S.hnd, &ccfg));

    S.ready = true;
    return ESP_OK;
}

esp_err_t drv_pot_start(void)
{
    KIA_RET_IF(!S.ready, ESP_ERR_INVALID_STATE);
    if (S.running) return ESP_OK;
    KIA_RET(adc_continuous_start(S.hnd));
    S.running = true;
    BaseType_t ok = xTaskCreatePinnedToCore(pot_task, "pot", 4096, NULL, 12, &S.task, KIA_CORE_RT);
    KIA_RET_IF(ok != pdPASS, ESP_ERR_NO_MEM);
    return ESP_OK;
}

esp_err_t drv_pot_stop(void)
{
    if (!S.running) return ESP_OK;
    S.running = false;
    vTaskDelay(pdMS_TO_TICKS(20));
    KIA_RET(adc_continuous_stop(S.hnd));
    return ESP_OK;
}

float drv_pot_get_normalised(uint8_t idx)
{
    if (idx >= DRV_POT_COUNT) return 0;
    pot_state_t *p   = &S.pot[idx];
    int32_t      lo  = p->cal.raw_min, hi = p->cal.raw_max;
    if (hi <= lo) return 0;
    int32_t v = (int32_t)p->stable - lo;
    if (v < 0) v = 0;
    if (v > (hi - lo)) v = hi - lo;
    return (float)v / (float)(hi - lo);
}

uint16_t drv_pot_get_raw(uint8_t idx)
{
    return idx < DRV_POT_COUNT ? S.pot[idx].stable : 0;
}

esp_err_t drv_pot_set_calibration(uint8_t idx, drv_pot_cal_t cal)
{
    KIA_RET_IF(idx >= DRV_POT_COUNT, ESP_ERR_INVALID_ARG);
    S.pot[idx].cal = cal;
    return ESP_OK;
}

esp_err_t drv_pot_get_calibration(uint8_t idx, drv_pot_cal_t *out)
{
    KIA_RET_IF(idx >= DRV_POT_COUNT || !out, ESP_ERR_INVALID_ARG);
    *out = S.pot[idx].cal;
    return ESP_OK;
}

bool drv_pot_motion_detected(uint16_t threshold_lsb)
{
    bool moved = false;
    for (int i = 0; i < DRV_POT_COUNT; ++i) {
        int32_t d = (int32_t)S.pot[i].stable - (int32_t)S.pot[i].last_emit;
        if (d < 0) d = -d;
        if ((uint32_t)d > threshold_lsb) {
            S.pot[i].last_emit = S.pot[i].stable;
            moved              = true;
        }
    }
    return moved;
}
