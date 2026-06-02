#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define DOM_JOINT_COUNT 5

typedef struct {
    float min_deg;
    float max_deg;
    float home_deg;
    float max_speed_dps;
    float max_accel_dps2;
} dom_joint_spec_t;

typedef struct {
    float pos_deg;
    float vel_dps;
    float target_deg;
    float vel_limit_dps;
} dom_joint_state_t;

esp_err_t dom_joint_init(void);

const dom_joint_spec_t *dom_joint_spec(uint8_t idx);
dom_joint_state_t      *dom_joint_state(uint8_t idx);

bool dom_joint_in_limits(uint8_t idx, float deg);
float dom_joint_clamp(uint8_t idx, float deg);
