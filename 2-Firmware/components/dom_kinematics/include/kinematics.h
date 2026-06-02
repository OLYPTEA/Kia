#pragma once

#include <stdbool.h>

#include "esp_err.h"

typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
    float pitch_deg; /* tool pitch from horizontal */
} kin_pose_t;

typedef struct {
    float base_deg;
    float shoulder_deg;
    float elbow_deg;
    float wrist_deg;
} kin_joints_t;

typedef enum {
    KIN_ELBOW_UP = 0,
    KIN_ELBOW_DOWN,
} kin_config_t;

void      kin_set_geometry(float base_h, float upper, float fore, float tool);
esp_err_t kin_fk(const kin_joints_t *j, kin_pose_t *p);
esp_err_t kin_ik(const kin_pose_t *p, kin_config_t cfg, kin_joints_t *out);
bool      kin_pose_reachable(const kin_pose_t *p);
