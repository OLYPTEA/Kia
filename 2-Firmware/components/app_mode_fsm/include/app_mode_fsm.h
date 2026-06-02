#pragma once

#include "esp_err.h"

typedef enum {
    APP_MODE_BOOT = 0,
    APP_MODE_IDLE,
    APP_MODE_MANUAL,
    APP_MODE_APP,
    APP_MODE_CALIB,
    APP_MODE_FAULT,
    APP_MODE_ESTOP,
} app_mode_t;

esp_err_t  app_mode_fsm_init(void);
app_mode_t app_mode_fsm_current(void);
esp_err_t  app_mode_fsm_request(app_mode_t target);
