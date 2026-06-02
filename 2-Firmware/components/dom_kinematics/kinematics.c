#include "kinematics.h"

#include <math.h>

#include "app_config.h"
#include "dom_joint.h"
#include "kia_err.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static struct {
    float h_base;
    float L1;
    float L2;
    float L_tool;
} G = {KIA_LINK_BASE_HEIGHT_MM, KIA_LINK_UPPER_ARM_MM, KIA_LINK_FOREARM_MM, KIA_LINK_TOOL_MM};

static inline float deg2rad(float d) { return d * (float)M_PI / 180.0f; }
static inline float rad2deg(float r) { return r * 180.0f / (float)M_PI; }

void kin_set_geometry(float base_h, float upper, float fore, float tool)
{
    G.h_base = base_h;
    G.L1     = upper;
    G.L2     = fore;
    G.L_tool = tool;
}

esp_err_t kin_fk(const kin_joints_t *j, kin_pose_t *p)
{
    KIA_RET_IF(!j || !p, ESP_ERR_INVALID_ARG);
    float q0 = deg2rad(j->base_deg);
    float q1 = deg2rad(j->shoulder_deg);
    float q2 = deg2rad(j->elbow_deg);
    float q3 = deg2rad(j->wrist_deg);

    /* Planar arm in the (r,z) plane, rotated by q0 around Z. */
    float r = G.L1 * sinf(q1) + G.L2 * sinf(q1 + q2) + G.L_tool * sinf(q1 + q2 + q3);
    float z = G.h_base + G.L1 * cosf(q1) + G.L2 * cosf(q1 + q2) + G.L_tool * cosf(q1 + q2 + q3);

    p->x_mm     = r * cosf(q0);
    p->y_mm     = r * sinf(q0);
    p->z_mm     = z;
    p->pitch_deg = rad2deg(q1 + q2 + q3 - (float)M_PI / 2);
    return ESP_OK;
}

esp_err_t kin_ik(const kin_pose_t *p, kin_config_t cfg, kin_joints_t *o)
{
    KIA_RET_IF(!p || !o, ESP_ERR_INVALID_ARG);

    /* J0 — base */
    float r_tcp = sqrtf(p->x_mm * p->x_mm + p->y_mm * p->y_mm);
    float q0    = atan2f(p->y_mm, p->x_mm);

    /* Tool pitch from horizontal -> wrist link direction. */
    float pitch_rad = deg2rad(p->pitch_deg);
    /* Convention: q1+q2+q3 = pi/2 - pitch  (q from vertical, pitch from horizontal) */
    float phi_total = (float)M_PI / 2 - pitch_rad;

    /* Wrist target = TCP - tool * (cos(pitch), sin(pitch)) projected in (r,z) */
    float r_w = r_tcp - G.L_tool * cosf(pitch_rad);
    float z_w = p->z_mm - G.h_base - G.L_tool * sinf(pitch_rad);

    float d2 = r_w * r_w + z_w * z_w;
    float d  = sqrtf(d2);
    if (d > (G.L1 + G.L2) || d < fabsf(G.L1 - G.L2)) return KIA_ERR_UNREACHABLE;

    float c2 = (d2 - G.L1 * G.L1 - G.L2 * G.L2) / (2 * G.L1 * G.L2);
    if (c2 < -1.0f) c2 = -1.0f;
    if (c2 > 1.0f) c2 = 1.0f;
    float s2 = (cfg == KIN_ELBOW_UP ? -1.0f : 1.0f) * sqrtf(1 - c2 * c2);
    float q2 = atan2f(s2, c2);

    float k1 = G.L1 + G.L2 * c2;
    float k2 = G.L2 * s2;
    float q1 = atan2f(r_w, z_w) - atan2f(k2, k1);

    float q3 = phi_total - q1 - q2;

    o->base_deg     = rad2deg(q0);
    o->shoulder_deg = rad2deg(q1);
    o->elbow_deg    = rad2deg(q2);
    o->wrist_deg    = rad2deg(q3);

    if (!dom_joint_in_limits(KIA_J_BASE, o->base_deg) ||
        !dom_joint_in_limits(KIA_J_SHOULDER, o->shoulder_deg) ||
        !dom_joint_in_limits(KIA_J_ELBOW, o->elbow_deg) ||
        !dom_joint_in_limits(KIA_J_WRIST, o->wrist_deg)) {
        return KIA_ERR_LIMIT;
    }
    return ESP_OK;
}

bool kin_pose_reachable(const kin_pose_t *p)
{
    kin_joints_t j;
    return kin_ik(p, KIN_ELBOW_UP, &j) == ESP_OK || kin_ik(p, KIN_ELBOW_DOWN, &j) == ESP_OK;
}
