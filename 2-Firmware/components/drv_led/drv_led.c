#include "drv_led.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "app_config.h"
#include "kia_err.h"

#define WS2812_T0H_NS 350
#define WS2812_T0L_NS 900
#define WS2812_T1H_NS 900
#define WS2812_T1L_NS 350

typedef struct {
    rmt_encoder_t        base;
    rmt_encoder_t       *bytes_encoder;
    rmt_encoder_t       *copy_encoder;
    int                  state;
    rmt_symbol_word_t    reset_code;
} ws2812_encoder_t;

static struct {
    rmt_channel_handle_t chan;
    rmt_encoder_handle_t enc;
    bool                 ready;
    uint8_t              cur_r, cur_g, cur_b;
    uint8_t              hb;
} S;

static size_t ws2812_encode(rmt_encoder_t *e, rmt_channel_handle_t chan, const void *data, size_t n,
                            rmt_encode_state_t *st)
{
    ws2812_encoder_t *self = __containerof(e, ws2812_encoder_t, base);
    rmt_encode_state_t s   = 0;
    size_t out             = 0;
    switch (self->state) {
        case 0:
            out += self->bytes_encoder->encode(self->bytes_encoder, chan, data, n, &s);
            if (s & RMT_ENCODING_COMPLETE) self->state = 1;
            if (s & RMT_ENCODING_MEM_FULL) { *st = s; return out; }
            /* fallthrough */
        case 1:
            out += self->copy_encoder->encode(self->copy_encoder, chan, &self->reset_code,
                                              sizeof(self->reset_code), &s);
            if (s & RMT_ENCODING_COMPLETE) { self->state = 0; *st = RMT_ENCODING_COMPLETE; }
            if (s & RMT_ENCODING_MEM_FULL) { *st = s; }
            return out;
    }
    return out;
}

static esp_err_t ws2812_del(rmt_encoder_t *e)
{
    ws2812_encoder_t *self = __containerof(e, ws2812_encoder_t, base);
    rmt_del_encoder(self->bytes_encoder);
    rmt_del_encoder(self->copy_encoder);
    free(self);
    return ESP_OK;
}

static esp_err_t ws2812_reset(rmt_encoder_t *e)
{
    ws2812_encoder_t *self = __containerof(e, ws2812_encoder_t, base);
    rmt_encoder_reset(self->bytes_encoder);
    rmt_encoder_reset(self->copy_encoder);
    self->state = 0;
    return ESP_OK;
}

static esp_err_t new_ws2812_encoder(uint32_t resolution_hz, rmt_encoder_handle_t *out)
{
    ws2812_encoder_t *self = calloc(1, sizeof(*self));
    KIA_RET_IF(!self, ESP_ERR_NO_MEM);
    self->base.encode   = ws2812_encode;
    self->base.del      = ws2812_del;
    self->base.reset    = ws2812_reset;

    rmt_bytes_encoder_config_t bcfg = {
        .bit0 =
            {
                .level0    = 1,
                .duration0 = (uint32_t)(WS2812_T0H_NS * (resolution_hz / 1000000)) / 1000,
                .level1    = 0,
                .duration1 = (uint32_t)(WS2812_T0L_NS * (resolution_hz / 1000000)) / 1000,
            },
        .bit1 =
            {
                .level0    = 1,
                .duration0 = (uint32_t)(WS2812_T1H_NS * (resolution_hz / 1000000)) / 1000,
                .level1    = 0,
                .duration1 = (uint32_t)(WS2812_T1L_NS * (resolution_hz / 1000000)) / 1000,
            },
        .flags.msb_first = 1,
    };
    KIA_RET(rmt_new_bytes_encoder(&bcfg, &self->bytes_encoder));

    rmt_copy_encoder_config_t ccfg = {};
    KIA_RET(rmt_new_copy_encoder(&ccfg, &self->copy_encoder));

    uint32_t reset_ticks = resolution_hz / 1000000 * 50;
    self->reset_code     = (rmt_symbol_word_t){
            .level0    = 0,
            .duration0 = reset_ticks,
            .level1    = 0,
            .duration1 = reset_ticks,
    };
    *out = &self->base;
    return ESP_OK;
}

esp_err_t drv_led_init(void)
{
    if (S.ready) return ESP_OK;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << KIA_GPIO_LED_STATUS) | (1ULL << KIA_GPIO_LED_FAULT),
        .mode         = GPIO_MODE_OUTPUT,
    };
    KIA_RET(gpio_config(&io));
    gpio_set_level(KIA_GPIO_LED_STATUS, 0);
    gpio_set_level(KIA_GPIO_LED_FAULT, 0);

    rmt_tx_channel_config_t cc = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = KIA_GPIO_WS2812,
        .mem_block_symbols = 64,
        .resolution_hz     = 10000000,
        .trans_queue_depth = 4,
    };
    KIA_RET(rmt_new_tx_channel(&cc, &S.chan));
    KIA_RET(new_ws2812_encoder(10000000, &S.enc));
    KIA_RET(rmt_enable(S.chan));
    S.ready = true;
    return drv_led_set_rgb(0, 0, 0);
}

esp_err_t drv_led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    KIA_RET_IF(!S.ready, ESP_ERR_INVALID_STATE);
    uint8_t grb[3] = {g, r, b};
    rmt_transmit_config_t tc = {.loop_count = 0};
    S.cur_r = r; S.cur_g = g; S.cur_b = b;
    return rmt_transmit(S.chan, S.enc, grb, sizeof grb, &tc);
}

esp_err_t drv_led_set_mode_color(drv_led_mode_color_t c)
{
    switch (c) {
        case DRV_LED_COLOR_OFF:    return drv_led_set_rgb(0, 0, 0);
        case DRV_LED_COLOR_BOOT:   return drv_led_set_rgb(8, 8, 8);
        case DRV_LED_COLOR_READY:  return drv_led_set_rgb(0, 32, 0);
        case DRV_LED_COLOR_MANUAL: return drv_led_set_rgb(0, 24, 32);
        case DRV_LED_COLOR_APP:    return drv_led_set_rgb(0, 0, 64);
        case DRV_LED_COLOR_CAL:    return drv_led_set_rgb(48, 32, 0);
        case DRV_LED_COLOR_FAULT:  return drv_led_set_rgb(64, 0, 0);
        case DRV_LED_COLOR_ESTOP:  return drv_led_set_rgb(128, 0, 0);
        case DRV_LED_COLOR_OTA:    return drv_led_set_rgb(64, 0, 64);
    }
    return ESP_ERR_INVALID_ARG;
}

esp_err_t drv_led_set_status(bool on)
{
    gpio_set_level(KIA_GPIO_LED_STATUS, on);
    return ESP_OK;
}

esp_err_t drv_led_set_fault(bool on)
{
    gpio_set_level(KIA_GPIO_LED_FAULT, on);
    return ESP_OK;
}

esp_err_t drv_led_heartbeat(void)
{
    S.hb ^= 1;
    return drv_led_set_status(S.hb);
}
