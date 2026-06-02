#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define DRV_POT_COUNT 5

typedef struct {
    uint16_t raw_min;
    uint16_t raw_max;
} drv_pot_cal_t;

esp_err_t drv_pot_init(void);
esp_err_t drv_pot_start(void);
esp_err_t drv_pot_stop(void);

/* Returns the filtered, hysteresis-applied value normalised to [0..1]. */
float drv_pot_get_normalised(uint8_t idx);

/* Returns raw filtered value (0..4095). */
uint16_t drv_pot_get_raw(uint8_t idx);

esp_err_t drv_pot_set_calibration(uint8_t idx, drv_pot_cal_t cal);
esp_err_t drv_pot_get_calibration(uint8_t idx, drv_pot_cal_t *out);

/* Returns true if any pot moved more than `threshold_lsb` since last call. */
bool drv_pot_motion_detected(uint16_t threshold_lsb);
