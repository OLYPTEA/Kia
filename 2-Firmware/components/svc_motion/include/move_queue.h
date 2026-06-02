#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "kinematics.h"

typedef enum {
    MOVE_NONE = 0,
    MOVE_JOINT,
    MOVE_CART_PTP,
    MOVE_CART_LINE,
    MOVE_DWELL,
} move_type_t;

typedef struct {
    move_type_t type;
    union {
        struct { kin_joints_t j; float grip; } joint;
        struct { kin_pose_t p;                } cart;
        struct { float ms;                    } dwell;
    } u;
    float duration_s;
    uint16_t seq;
} move_t;

esp_err_t move_queue_init(uint16_t depth);
esp_err_t move_queue_push(const move_t *m);
bool      move_queue_pop(move_t *out);
size_t    move_queue_size(void);
void      move_queue_clear(void);
