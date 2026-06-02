#pragma once

#include "esp_err.h"

esp_err_t svc_telemetry_init(void);
esp_err_t svc_telemetry_set_rate_hz(uint16_t hz);
