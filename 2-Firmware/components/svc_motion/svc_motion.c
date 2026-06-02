#include "svc_motion.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "app_config.h"
#include "cart_path.h"
#include "dom_joint.h"
#include "drv_pot.h"
#include "drv_servo.h"
#include "kia_err.h"
#include "kinematics.h"
#include "move_queue.h"
#include "svc_safety.h"
#include "trajectory.h"

static const char *TAG = "svc_motion";

typedef struct {
    motion_src_t      src;
    traj_quintic_t    tr[DOM_JOINT_COUNT];
    traj_scurve_t     sc[DOM_JOINT_COUNT];
    float             current[DOM_JOINT_COUNT];
    SemaphoreHandle_t mtx;
    TaskHandle_t      task;
    bool              ready;
    bool              use_scurve;
} motion_t;

static motion_t M;

static void start_move(const move_t *m);

static float now_s(void)
{
    return (float)esp_timer_get_time() * 1e-6f;
}

static inline float map_pot_to_joint(uint8_t j, float n)
{
    const dom_joint_spec_t *s = dom_joint_spec(j);
    return s->min_deg + n * (s->max_deg - s->min_deg);
}

static void task(void *arg)
{
    KIA_UNUSED(arg);
    const TickType_t period = pdMS_TO_TICKS(1000 / KIA_CTRL_RATE_HZ);
    TickType_t       wake   = xTaskGetTickCount();

    while (true) {
        if (!svc_safety_is_ok()) {
            vTaskDelayUntil(&wake, period);
            continue;
        }

        xSemaphoreTake(M.mtx, portMAX_DELAY);
        float t = now_s();
        switch (M.src) {
            case MOTION_SRC_MANUAL_POT:
                for (uint8_t i = 0; i < DOM_JOINT_COUNT; ++i) {
                    float n   = drv_pot_get_normalised(i);
                    float tgt = map_pot_to_joint(i, n);
                    M.current[i] += kia_clampf(tgt - M.current[i], -KIA_SLEW_MAX_DEGPS / KIA_CTRL_RATE_HZ,
                                               KIA_SLEW_MAX_DEGPS / KIA_CTRL_RATE_HZ);
                }
                break;
            case MOTION_SRC_APP_JOINT:
            case MOTION_SRC_APP_CART: {
                bool any_active = false;
                for (uint8_t i = 0; i < DOM_JOINT_COUNT; ++i) {
                    if (M.use_scurve && M.sc[i].active) {
                        M.current[i] = traj_scurve_sample(&M.sc[i], t, NULL);
                        any_active   = true;
                    } else if (M.tr[i].active) {
                        M.current[i] = traj_sample(&M.tr[i], t, NULL);
                        any_active   = true;
                    }
                }
                if (!any_active) {
                    move_t next;
                    if (move_queue_pop(&next)) start_move(&next);
                }
            } break;
            case MOTION_SRC_HOLD:
            default: break;
        }
        for (uint8_t i = 0; i < DOM_JOINT_COUNT; ++i) {
            float c = dom_joint_clamp(i, M.current[i]);
            M.current[i] = c;
            drv_servo_set_angle(i, c);
            dom_joint_state(i)->pos_deg = c;
        }
        xSemaphoreGive(M.mtx);

        vTaskDelayUntil(&wake, period);
    }
}

esp_err_t svc_motion_init(void)
{
    if (M.ready) return ESP_OK;
    memset(&M, 0, sizeof(M));
    dom_joint_init();
    for (uint8_t i = 0; i < DOM_JOINT_COUNT; ++i) M.current[i] = dom_joint_spec(i)->home_deg;
    M.mtx = xSemaphoreCreateMutex();
    KIA_RET_IF(!M.mtx, ESP_ERR_NO_MEM);
    M.src = MOTION_SRC_HOLD;
    KIA_RET(move_queue_init(KIA_TRAJECTORY_QUEUE_LEN));

    drv_pot_start();

    BaseType_t ok = xTaskCreatePinnedToCore(task, "motion", KIA_STACK_MOTION, NULL, KIA_PRIO_MOTION,
                                            &M.task, KIA_CORE_RT);
    KIA_RET_IF(ok != pdPASS, ESP_ERR_NO_MEM);

    M.ready = true;
    ESP_LOGI(TAG, "motion ctrl @ %d Hz", KIA_CTRL_RATE_HZ);
    return ESP_OK;
}

esp_err_t svc_motion_set_source(motion_src_t s)
{
    xSemaphoreTake(M.mtx, portMAX_DELAY);
    M.src = s;
    xSemaphoreGive(M.mtx);
    return ESP_OK;
}

motion_src_t svc_motion_get_source(void) { return M.src; }

esp_err_t svc_motion_set_joint_target(uint8_t i, float deg, float dur)
{
    KIA_RET_IF(i >= DOM_JOINT_COUNT, ESP_ERR_INVALID_ARG);
    if (!dom_joint_in_limits(i, deg)) return KIA_ERR_LIMIT;
    xSemaphoreTake(M.mtx, portMAX_DELAY);
    traj_init_quintic(&M.tr[i], M.current[i], 0, 0, deg, 0, 0, dur);
    M.src = MOTION_SRC_APP_JOINT;
    xSemaphoreGive(M.mtx);
    return ESP_OK;
}

esp_err_t svc_motion_set_joint_targets(const kin_joints_t *j, float grip, float dur)
{
    KIA_RET_IF(!j, ESP_ERR_INVALID_ARG);
    if (!dom_joint_in_limits(KIA_J_BASE, j->base_deg) ||
        !dom_joint_in_limits(KIA_J_SHOULDER, j->shoulder_deg) ||
        !dom_joint_in_limits(KIA_J_ELBOW, j->elbow_deg) ||
        !dom_joint_in_limits(KIA_J_WRIST, j->wrist_deg) ||
        !dom_joint_in_limits(KIA_J_GRIP, grip)) {
        return KIA_ERR_LIMIT;
    }
    xSemaphoreTake(M.mtx, portMAX_DELAY);
    traj_init_quintic(&M.tr[KIA_J_BASE],     M.current[KIA_J_BASE],     0, 0, j->base_deg,     0, 0, dur);
    traj_init_quintic(&M.tr[KIA_J_SHOULDER], M.current[KIA_J_SHOULDER], 0, 0, j->shoulder_deg, 0, 0, dur);
    traj_init_quintic(&M.tr[KIA_J_ELBOW],    M.current[KIA_J_ELBOW],    0, 0, j->elbow_deg,    0, 0, dur);
    traj_init_quintic(&M.tr[KIA_J_WRIST],    M.current[KIA_J_WRIST],    0, 0, j->wrist_deg,    0, 0, dur);
    traj_init_quintic(&M.tr[KIA_J_GRIP],     M.current[KIA_J_GRIP],     0, 0, grip,            0, 0, dur);
    M.src = MOTION_SRC_APP_JOINT;
    xSemaphoreGive(M.mtx);
    return ESP_OK;
}

esp_err_t svc_motion_set_cartesian_target(const kin_pose_t *p, float dur)
{
    KIA_RET_IF(!p, ESP_ERR_INVALID_ARG);
    kin_joints_t j;
    esp_err_t e = kin_ik(p, KIN_ELBOW_UP, &j);
    if (e == KIA_ERR_LIMIT || e == KIA_ERR_UNREACHABLE) {
        e = kin_ik(p, KIN_ELBOW_DOWN, &j);
    }
    KIA_RET(e);
    return svc_motion_set_joint_targets(&j, M.current[KIA_J_GRIP], dur);
}

esp_err_t svc_motion_home(void)
{
    kin_joints_t j = {
        dom_joint_spec(KIA_J_BASE)->home_deg,
        dom_joint_spec(KIA_J_SHOULDER)->home_deg,
        dom_joint_spec(KIA_J_ELBOW)->home_deg,
        dom_joint_spec(KIA_J_WRIST)->home_deg,
    };
    return svc_motion_set_joint_targets(&j, dom_joint_spec(KIA_J_GRIP)->home_deg, 2.5f);
}

esp_err_t svc_motion_hold(void)
{
    xSemaphoreTake(M.mtx, portMAX_DELAY);
    for (uint8_t i = 0; i < DOM_JOINT_COUNT; ++i) M.tr[i].active = false;
    M.src = MOTION_SRC_HOLD;
    xSemaphoreGive(M.mtx);
    return ESP_OK;
}

void svc_motion_get_joints(kin_joints_t *o, float *g)
{
    if (o) {
        o->base_deg     = M.current[KIA_J_BASE];
        o->shoulder_deg = M.current[KIA_J_SHOULDER];
        o->elbow_deg    = M.current[KIA_J_ELBOW];
        o->wrist_deg    = M.current[KIA_J_WRIST];
    }
    if (g) *g = M.current[KIA_J_GRIP];
}

void svc_motion_get_pose(kin_pose_t *p)
{
    kin_joints_t j;
    svc_motion_get_joints(&j, NULL);
    kin_fk(&j, p);
}

bool svc_motion_is_idle(void)
{
    for (uint8_t i = 0; i < DOM_JOINT_COUNT; ++i) {
        if (M.tr[i].active) return false;
        if (M.use_scurve && M.sc[i].active) return false;
    }
    return move_queue_size() == 0;
}

esp_err_t svc_motion_use_scurve(bool en)
{
    M.use_scurve = en;
    return ESP_OK;
}

void   svc_motion_clear_queue(void) { move_queue_clear(); }
size_t svc_motion_queue_size(void)  { return move_queue_size(); }

esp_err_t svc_motion_line_to(const kin_pose_t *p, float dur, float step_mm)
{
    if (step_mm <= 0) step_mm = 5.0f;
    KIA_RET(cart_path_plan_line(p, dur, step_mm));
    M.src = MOTION_SRC_APP_CART;
    return ESP_OK;
}

static void start_move(const move_t *m)
{
    float now = now_s();
    switch (m->type) {
        case MOVE_JOINT: {
            const kin_joints_t *j = &m->u.joint.j;
            float vmax = KIA_SLEW_MAX_DEGPS;
            if (M.use_scurve) {
                traj_init_scurve(&M.sc[KIA_J_BASE],     M.current[KIA_J_BASE],     j->base_deg,     vmax, vmax * 4, vmax * 16);
                traj_init_scurve(&M.sc[KIA_J_SHOULDER], M.current[KIA_J_SHOULDER], j->shoulder_deg, vmax, vmax * 4, vmax * 16);
                traj_init_scurve(&M.sc[KIA_J_ELBOW],    M.current[KIA_J_ELBOW],    j->elbow_deg,    vmax, vmax * 4, vmax * 16);
                traj_init_scurve(&M.sc[KIA_J_WRIST],    M.current[KIA_J_WRIST],    j->wrist_deg,    vmax, vmax * 4, vmax * 16);
                if (!isnanf(m->u.joint.grip))
                    traj_init_scurve(&M.sc[KIA_J_GRIP], M.current[KIA_J_GRIP], m->u.joint.grip, vmax, vmax*4, vmax*16);
            } else {
                float dur = m->duration_s > 0 ? m->duration_s : 1.0f;
                traj_init_quintic(&M.tr[KIA_J_BASE],     M.current[KIA_J_BASE],     0, 0, j->base_deg,     0, 0, dur);
                traj_init_quintic(&M.tr[KIA_J_SHOULDER], M.current[KIA_J_SHOULDER], 0, 0, j->shoulder_deg, 0, 0, dur);
                traj_init_quintic(&M.tr[KIA_J_ELBOW],    M.current[KIA_J_ELBOW],    0, 0, j->elbow_deg,    0, 0, dur);
                traj_init_quintic(&M.tr[KIA_J_WRIST],    M.current[KIA_J_WRIST],    0, 0, j->wrist_deg,    0, 0, dur);
                if (!isnanf(m->u.joint.grip))
                    traj_init_quintic(&M.tr[KIA_J_GRIP], M.current[KIA_J_GRIP], 0, 0, m->u.joint.grip, 0, 0, dur);
            }
        } break;
        default: break;
    }
    (void)now;
}
