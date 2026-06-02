#pragma once

#include "esp_err.h"

esp_err_t svc_storage_init(void);
esp_err_t svc_storage_save_calibration(void);
esp_err_t svc_storage_load_calibration(void);
esp_err_t svc_storage_factory_reset(void);
