#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t drv_buzzer_init(void);
esp_err_t drv_buzzer_tone(uint32_t freq_hz, uint32_t duration_ms);
esp_err_t drv_buzzer_beep_ok(void);
esp_err_t drv_buzzer_beep_err(void);
esp_err_t drv_buzzer_beep_fault(void);
