#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef enum {
    APP_CAL_SERVO_ZERO = 0,
    APP_CAL_SERVO_MIN,
    APP_CAL_SERVO_MAX,
} app_cal_servo_op_t;

typedef enum {
    APP_CAL_POT_MIN = 0,
    APP_CAL_POT_MAX,
} app_cal_pot_op_t;

esp_err_t app_calibration_servo(uint8_t idx, app_cal_servo_op_t op);
esp_err_t app_calibration_pot(uint8_t idx, app_cal_pot_op_t op);
esp_err_t app_calibration_set_geometry(float base_h_mm, float upper_mm, float fore_mm, float tool_mm);
esp_err_t app_calibration_persist(void);

/* Legacy entry points kept for compatibility. */
esp_err_t app_calibration_servo_capture_zero(uint8_t idx);
esp_err_t app_calibration_pot_capture_endpoints(uint8_t idx);
