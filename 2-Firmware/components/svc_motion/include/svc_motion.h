#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "kinematics.h"

typedef enum {
    MOTION_SRC_NONE = 0,
    MOTION_SRC_MANUAL_POT,
    MOTION_SRC_APP_JOINT,
    MOTION_SRC_APP_CART,
    MOTION_SRC_HOLD,
} motion_src_t;

esp_err_t svc_motion_init(void);
esp_err_t svc_motion_set_source(motion_src_t s);
motion_src_t svc_motion_get_source(void);

esp_err_t svc_motion_set_joint_target(uint8_t idx, float deg, float duration_s);
esp_err_t svc_motion_set_joint_targets(const kin_joints_t *j, float gripper_deg, float duration_s);
esp_err_t svc_motion_set_cartesian_target(const kin_pose_t *p, float duration_s);
esp_err_t svc_motion_line_to(const kin_pose_t *p, float duration_s, float step_mm);
esp_err_t svc_motion_home(void);
esp_err_t svc_motion_hold(void);
esp_err_t svc_motion_use_scurve(bool enable);
void      svc_motion_clear_queue(void);
size_t    svc_motion_queue_size(void);

void svc_motion_get_joints(kin_joints_t *out, float *gripper);
void svc_motion_get_pose(kin_pose_t *out);
bool svc_motion_is_idle(void);
