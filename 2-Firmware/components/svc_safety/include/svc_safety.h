#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    SAFETY_OK = 0,
    SAFETY_ESTOP,
    SAFETY_OVERCURRENT,
    SAFETY_UNDERVOLT,
    SAFETY_HW_FAIL,
    SAFETY_TIMEOUT,
} safety_fault_t;

esp_err_t      svc_safety_init(void);
bool           svc_safety_is_ok(void);
safety_fault_t svc_safety_last_fault(void);
esp_err_t      svc_safety_arm(void);     /* enables servo rail if no active fault */
esp_err_t      svc_safety_trip(safety_fault_t f);
esp_err_t      svc_safety_clear(void);
int32_t        svc_safety_last_current_ma(void);
