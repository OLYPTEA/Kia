#pragma once

#include "esp_err.h"

#include "kinematics.h"

/* Straight-line in TCP space between current pose and `target`.
 * Subdivides into N segments of ~step_mm and pushes joint moves into the queue.
 * Tool pitch is linearly interpolated. */
esp_err_t cart_path_plan_line(const kin_pose_t *target, float total_duration_s, float step_mm);
