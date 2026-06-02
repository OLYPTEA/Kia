#include "dom_joint.h"

#include <string.h>

#include "app_config.h"
#include "kia_err.h"

static const dom_joint_spec_t s_spec[DOM_JOINT_COUNT] = {
    [KIA_J_BASE]     = {KIA_LIM_BASE_MIN_DEG,     KIA_LIM_BASE_MAX_DEG,     0,    KIA_SLEW_MAX_DEGPS, 1500},
    [KIA_J_SHOULDER] = {KIA_LIM_SHOULDER_MIN_DEG, KIA_LIM_SHOULDER_MAX_DEG, 90,   KIA_SLEW_MAX_DEGPS, 1500},
    [KIA_J_ELBOW]    = {KIA_LIM_ELBOW_MIN_DEG,    KIA_LIM_ELBOW_MAX_DEG,    0,    KIA_SLEW_MAX_DEGPS, 1500},
    [KIA_J_WRIST]    = {KIA_LIM_WRIST_MIN_DEG,    KIA_LIM_WRIST_MAX_DEG,    0,    KIA_SLEW_MAX_DEGPS, 1500},
    [KIA_J_GRIP]     = {KIA_LIM_GRIP_MIN_DEG,     KIA_LIM_GRIP_MAX_DEG,     45,   KIA_SLEW_MAX_DEGPS, 1500},
};

static dom_joint_state_t s_state[DOM_JOINT_COUNT];

esp_err_t dom_joint_init(void)
{
    for (int i = 0; i < DOM_JOINT_COUNT; ++i) {
        s_state[i].pos_deg       = s_spec[i].home_deg;
        s_state[i].target_deg    = s_spec[i].home_deg;
        s_state[i].vel_dps       = 0;
        s_state[i].vel_limit_dps = s_spec[i].max_speed_dps;
    }
    return ESP_OK;
}

const dom_joint_spec_t *dom_joint_spec(uint8_t i)
{
    return i < DOM_JOINT_COUNT ? &s_spec[i] : NULL;
}

dom_joint_state_t *dom_joint_state(uint8_t i)
{
    return i < DOM_JOINT_COUNT ? &s_state[i] : NULL;
}

bool dom_joint_in_limits(uint8_t i, float deg)
{
    if (i >= DOM_JOINT_COUNT) return false;
    return deg >= s_spec[i].min_deg && deg <= s_spec[i].max_deg;
}

float dom_joint_clamp(uint8_t i, float deg)
{
    if (i >= DOM_JOINT_COUNT) return deg;
    return kia_clampf(deg, s_spec[i].min_deg, s_spec[i].max_deg);
}
