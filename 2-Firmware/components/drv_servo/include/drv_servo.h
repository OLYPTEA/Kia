#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define DRV_SERVO_COUNT 5

typedef struct {
    int16_t pulse_min_us;
    int16_t pulse_max_us;
    float   angle_min_deg;
    float   angle_max_deg;
    float   offset_deg;
    bool    inverted;
} drv_servo_cal_t;

esp_err_t drv_servo_init(void);
esp_err_t drv_servo_enable(bool on);
esp_err_t drv_servo_set_calibration(uint8_t idx, const drv_servo_cal_t *cal);
esp_err_t drv_servo_get_calibration(uint8_t idx, drv_servo_cal_t *out);

/* Set target angle in deg. Saturates to per-servo limits. Thread-safe. */
esp_err_t drv_servo_set_angle(uint8_t idx, float deg);
float     drv_servo_get_angle(uint8_t idx);

/* Direct pulse override (debug / calibration only). */
esp_err_t drv_servo_set_pulse_us(uint8_t idx, uint32_t us);

/* All to neutral (1500 us nominal, after calibration mid-point). */
esp_err_t drv_servo_home_all(void);

/* Returns the global enable state mirrored from SERVO_EN GPIO. */
bool drv_servo_is_enabled(void);
