#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef struct {
    int16_t  bus_mv;
    int32_t  shunt_uv;
    int32_t  current_ma;
    int32_t  power_mw;
} drv_ina219_sample_t;

esp_err_t drv_ina219_init(void);
esp_err_t drv_ina219_read(drv_ina219_sample_t *out);

/* Set shunt resistance in milli-ohms (typically 10). */
esp_err_t drv_ina219_set_shunt_mohm(uint16_t mohm);
