#include "cart_path.h"

#include <math.h>

#include "kia_err.h"
#include "move_queue.h"
#include "svc_motion.h"

esp_err_t cart_path_plan_line(const kin_pose_t *target, float total_dur, float step_mm)
{
    if (!target || total_dur <= 0 || step_mm <= 0) return ESP_ERR_INVALID_ARG;

    kin_pose_t start;
    svc_motion_get_pose(&start);

    float dx = target->x_mm - start.x_mm;
    float dy = target->y_mm - start.y_mm;
    float dz = target->z_mm - start.z_mm;
    float d  = sqrtf(dx * dx + dy * dy + dz * dz);
    if (d < 0.5f) return ESP_OK;

    int n = (int)ceilf(d / step_mm);
    if (n < 2) n = 2;
    float seg_dur = total_dur / n;

    for (int i = 1; i <= n; ++i) {
        float u = (float)i / n;
        kin_pose_t p = {
            .x_mm      = start.x_mm + dx * u,
            .y_mm      = start.y_mm + dy * u,
            .z_mm      = start.z_mm + dz * u,
            .pitch_deg = start.pitch_deg + (target->pitch_deg - start.pitch_deg) * u,
        };
        kin_joints_t j;
        esp_err_t    e = kin_ik(&p, KIN_ELBOW_UP, &j);
        if (e != ESP_OK) e = kin_ik(&p, KIN_ELBOW_DOWN, &j);
        if (e != ESP_OK) return e;
        move_t m = {
            .type       = MOVE_JOINT,
            .duration_s = seg_dur,
            .u.joint    = {.j = j, .grip = 0.0f / 0.0f}, /* NaN -> keep current grip */
        };
        KIA_RET(move_queue_push(&m));
    }
    return ESP_OK;
}
