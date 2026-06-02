#pragma once

#include <stdbool.h>

#include "esp_err.h"

typedef enum {
    DRV_BTN_ESTOP = 0,
    DRV_BTN_MODE,
    DRV_BTN__COUNT,
} drv_btn_id_t;

typedef void (*drv_btn_cb_t)(drv_btn_id_t id, bool pressed, void *user);

esp_err_t drv_buttons_init(void);
esp_err_t drv_buttons_register(drv_btn_id_t id, drv_btn_cb_t cb, void *user);
bool      drv_buttons_get_state(drv_btn_id_t id);
