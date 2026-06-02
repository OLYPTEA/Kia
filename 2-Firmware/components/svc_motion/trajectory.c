#include "trajectory.h"

#include <math.h>

/* ============ Quintic ============ */

void traj_init_quintic(traj_quintic_t *t, float p0, float v0, float a0, float p1, float v1, float a1,
                       float duration_s)
{
    if (duration_s < 0.01f) duration_s = 0.01f;
    float T  = duration_s;
    float T2 = T * T;
    float T3 = T2 * T;
    float T4 = T3 * T;
    float T5 = T4 * T;

    t->a0 = p0;
    t->a1 = v0;
    t->a2 = 0.5f * a0;

    float dp = p1 - (p0 + v0 * T + 0.5f * a0 * T2);
    float dv = v1 - (v0 + a0 * T);
    float da = a1 - a0;

    t->a3 = (10.0f * dp - 4.0f * dv * T - 0.5f * da * T2) / T3;
    t->a4 = (-15.0f * dp + 7.0f * dv * T + da * T2) / T4;
    t->a5 = (6.0f * dp - 3.0f * dv * T - 0.5f * da * T2) / T5;

    t->duration_us = duration_s * 1e6f;
    t->t0_us       = 0;
    t->active      = true;
}

float traj_sample(traj_quintic_t *t, float now_s, float *vel_out)
{
    if (!t->active) {
        if (vel_out) *vel_out = 0;
        return t->a0;
    }
    if (t->t0_us == 0) t->t0_us = now_s * 1e6f;
    float te = now_s * 1e6f - t->t0_us;
    if (te >= t->duration_us) {
        t->active = false;
        te        = t->duration_us;
    }
    float s  = te * 1e-6f;
    float s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    float p  = t->a0 + t->a1 * s + t->a2 * s2 + t->a3 * s3 + t->a4 * s4 + t->a5 * s5;
    if (vel_out)
        *vel_out = t->a1 + 2 * t->a2 * s + 3 * t->a3 * s2 + 4 * t->a4 * s3 + 5 * t->a5 * s4;
    return p;
}

bool traj_done(const traj_quintic_t *t, float now_s)
{
    if (!t->active) return true;
    return (now_s * 1e6f - t->t0_us) >= t->duration_us;
}

/* ============ S-curve 7-phase jerk-limited =============
 * Simplified profile for rest-to-rest motion (v0 = v1 = 0).
 * Reference: Biagiotti & Melchiorri, Trajectory Planning for
 * Automatic Machines and Robots, Ch.3.
 */

bool traj_init_scurve(traj_scurve_t *t, float p0, float p1, float v_max, float a_max, float j_max)
{
    if (v_max <= 0 || a_max <= 0 || j_max <= 0) return false;
    float h    = p1 - p0;
    float dir  = (h >= 0) ? 1.0f : -1.0f;
    float ha   = fabsf(h);
    if (ha < 1e-6f) { t->active = false; return false; }

    /* Check if a_max is reachable: need ha >= 2 a_max^3 / j_max^2 → otherwise reduce a_max. */
    float Tj   = a_max / j_max;
    float Tj2_lim = sqrtf(v_max / j_max);
    if (Tj > Tj2_lim) {
        Tj    = Tj2_lim;
        a_max = j_max * Tj;
    }

    float Ta;
    if (v_max * j_max >= a_max * a_max) {
        Ta = v_max / a_max + Tj;
    } else {
        Tj = sqrtf(v_max / j_max);
        Ta = 2.0f * Tj;
    }
    /* Check if v_max reached. */
    if (Ta * v_max < ha) {
        /* Cruise phase. */
        t->Tj1 = Tj;
        t->Ta  = Ta;
        t->Tv  = (ha - Ta * v_max) / v_max;
        t->Tj2 = Tj;
        t->Td  = Ta;
    } else {
        /* No cruise. Recompute acceleration phase. */
        float disc = Tj * Tj * Tj * Tj * 0.25f + ha / j_max;
        Ta = Tj * 0.5f + sqrtf(disc);
        t->Tj1 = Tj; t->Ta = Ta; t->Tv = 0; t->Tj2 = Tj; t->Td = Ta;
    }

    t->p0          = p0;
    t->v_max       = v_max;
    t->a_max       = a_max;
    t->j_max       = j_max;
    t->dir         = dir;
    t->duration_us = (t->Ta + t->Tv + t->Td) * 1e6f;
    t->t0_us       = 0;
    t->active      = true;
    return true;
}

float traj_scurve_sample(traj_scurve_t *t, float now_s, float *vel_out)
{
    if (!t->active) {
        if (vel_out) *vel_out = 0;
        return t->p0;
    }
    if (t->t0_us == 0) t->t0_us = now_s * 1e6f;
    float te = now_s * 1e6f - t->t0_us;
    if (te >= t->duration_us) {
        t->active = false;
        if (vel_out) *vel_out = 0;
        return t->p0 + t->dir * (0.5f * (t->Ta + t->Td) * t->v_max + t->Tv * t->v_max);
    }
    float sec  = te * 1e-6f;
    float Tj1  = t->Tj1, Ta = t->Ta, Tv = t->Tv, Tj2 = t->Tj2, Td = t->Td;
    float v_lim = t->a_max * (Ta - Tj1);

    float p = 0, v = 0;
    if (sec < Tj1) {
        /* Phase 1: jerk +j_max */
        p = (1.0f / 6.0f) * t->j_max * sec * sec * sec;
        v = 0.5f * t->j_max * sec * sec;
    } else if (sec < Ta - Tj1) {
        float dt = sec - Tj1;
        p = (1.0f / 6.0f) * t->j_max * Tj1 * Tj1 * Tj1 + 0.5f * t->a_max * Tj1 * dt + 0.5f * t->a_max * dt * dt;
        v = t->a_max * (Tj1 * 0.5f + dt);
    } else if (sec < Ta) {
        float dt = sec - (Ta - Tj1);
        float p_ph1 = (1.0f / 6.0f) * t->j_max * Tj1 * Tj1 * Tj1;
        float v_ph1 = 0.5f * t->j_max * Tj1 * Tj1;
        float dt_mid = Ta - 2 * Tj1;
        float p_mid = v_ph1 * dt_mid + 0.5f * t->a_max * dt_mid * dt_mid;
        float v_mid = v_ph1 + t->a_max * dt_mid;
        p = p_ph1 + p_mid + v_mid * dt - (1.0f / 6.0f) * t->j_max * dt * dt * dt;
        v = v_mid - 0.5f * t->j_max * dt * dt;
    } else if (sec < Ta + Tv) {
        float dt = sec - Ta;
        p = 0.5f * v_lim * Ta + v_lim * dt;
        v = v_lim;
    } else {
        /* Deceleration mirror */
        float dt_total = Ta + Tv + Td;
        float remain   = dt_total - sec;
        if (remain < 0) remain = 0;
        traj_scurve_t mirror = *t;
        mirror.t0_us = 0;
        mirror.active = true;
        float p_m, v_m;
        /* Sample the acceleration phase at "remain" and reflect. */
        if (remain < Tj2) {
            p_m = (1.0f / 6.0f) * t->j_max * remain * remain * remain;
            v_m = 0.5f * t->j_max * remain * remain;
        } else if (remain < Td - Tj2) {
            float dt = remain - Tj2;
            p_m = (1.0f / 6.0f) * t->j_max * Tj2 * Tj2 * Tj2 + 0.5f * t->a_max * Tj2 * dt + 0.5f * t->a_max * dt * dt;
            v_m = t->a_max * (Tj2 * 0.5f + dt);
        } else {
            float dt = remain - (Td - Tj2);
            float p_ph1 = (1.0f / 6.0f) * t->j_max * Tj2 * Tj2 * Tj2;
            float v_ph1 = 0.5f * t->j_max * Tj2 * Tj2;
            float dt_mid = Td - 2 * Tj2;
            float p_mid = v_ph1 * dt_mid + 0.5f * t->a_max * dt_mid * dt_mid;
            float v_mid = v_ph1 + t->a_max * dt_mid;
            p_m = p_ph1 + p_mid + v_mid * dt - (1.0f / 6.0f) * t->j_max * dt * dt * dt;
            v_m = v_mid - 0.5f * t->j_max * dt * dt;
        }
        float p_total = 0.5f * v_lim * Ta + v_lim * Tv + 0.5f * v_lim * Td;
        p = p_total - p_m;
        v = v_m;
    }
    if (vel_out) *vel_out = t->dir * v;
    return t->p0 + t->dir * p;
}

bool traj_scurve_done(const traj_scurve_t *t, float now_s)
{
    if (!t->active) return true;
    return (now_s * 1e6f - t->t0_us) >= t->duration_us;
}
