#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    DRV_LED_COLOR_OFF = 0,
    DRV_LED_COLOR_BOOT,      /* dim white */
    DRV_LED_COLOR_READY,     /* green */
    DRV_LED_COLOR_MANUAL,    /* cyan */
    DRV_LED_COLOR_APP,       /* blue */
    DRV_LED_COLOR_CAL,       /* yellow */
    DRV_LED_COLOR_FAULT,     /* red */
    DRV_LED_COLOR_ESTOP,     /* red blinking */
    DRV_LED_COLOR_OTA,       /* magenta */
} drv_led_mode_color_t;

esp_err_t drv_led_init(void);
esp_err_t drv_led_set_mode_color(drv_led_mode_color_t c);
esp_err_t drv_led_set_rgb(uint8_t r, uint8_t g, uint8_t b);
esp_err_t drv_led_set_status(bool on);
esp_err_t drv_led_set_fault(bool on);
esp_err_t drv_led_heartbeat(void); /* called periodically by telemetry task */
