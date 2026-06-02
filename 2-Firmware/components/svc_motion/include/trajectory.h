#pragma once

#include <stdbool.h>
#include <stdint.h>

/* === Quintic polynomial 1D ============================================== */
typedef struct {
    float a0, a1, a2, a3, a4, a5;
    float t0_us;
    float duration_us;
    bool  active;
} traj_quintic_t;

void  traj_init_quintic(traj_quintic_t *t, float pos0, float vel0, float acc0, float pos1, float vel1,
                        float acc1, float duration_s);
float traj_sample(traj_quintic_t *t, float now_s, float *vel_out);
bool  traj_done(const traj_quintic_t *t, float now_s);

/* === S-curve (7-phase jerk-limited) 1D ================================== */
typedef struct {
    float p0;
    float v_max;     /* signed */
    float a_max;     /* signed */
    float j_max;     /* magnitude */
    float dir;       /* +1 or -1 */
    float Tj1, Ta, Tv, Tj2, Td; /* phase durations (>=0) */
    float t0_us;
    float duration_us;
    bool  active;
} traj_scurve_t;

bool  traj_init_scurve(traj_scurve_t *t, float pos0, float pos1, float v_max, float a_max, float j_max);
float traj_scurve_sample(traj_scurve_t *t, float now_s, float *vel_out);
bool  traj_scurve_done(const traj_scurve_t *t, float now_s);
