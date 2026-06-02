#pragma once

#include "esp_err.h"
#include "esp_log.h"

typedef enum {
    KIA_OK = 0,
    KIA_ERR_INVALID_ARG = 0x1001,
    KIA_ERR_NOT_INIT,
    KIA_ERR_BUSY,
    KIA_ERR_TIMEOUT,
    KIA_ERR_OUT_OF_RANGE,
    KIA_ERR_NOT_FOUND,
    KIA_ERR_NO_MEM,
    KIA_ERR_PROTO,
    KIA_ERR_CRC,
    KIA_ERR_CALIB_MISSING,
    KIA_ERR_LIMIT,
    KIA_ERR_UNREACHABLE,
    KIA_ERR_FAULT_ACTIVE,
    KIA_ERR_HW_FAIL,
} kia_err_t;

#define KIA_CHK(x)                                                                                       \
    do {                                                                                                 \
        esp_err_t _e = (x);                                                                              \
        if (_e != ESP_OK) {                                                                              \
            ESP_LOGE("KIA_CHK", "%s failed at %s:%d -> 0x%x", #x, __FILE__, __LINE__, _e);               \
            abort();                                                                                     \
        }                                                                                                \
    } while (0)

#define KIA_RET(x)                                                                                       \
    do {                                                                                                 \
        esp_err_t _e = (x);                                                                              \
        if (_e != ESP_OK) {                                                                              \
            return _e;                                                                                   \
        }                                                                                                \
    } while (0)

#define KIA_RET_IF(cond, err)                                                                            \
    do {                                                                                                 \
        if (cond) {                                                                                      \
            return (err);                                                                                \
        }                                                                                                \
    } while (0)

#define KIA_ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))
#define KIA_UNUSED(x)    ((void)(x))

static inline float kia_clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline int32_t kia_clampi(int32_t v, int32_t lo, int32_t hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float kia_lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

static inline float kia_mapf(float v, float in_lo, float in_hi, float out_lo, float out_hi)
{
    return out_lo + (v - in_lo) * (out_hi - out_lo) / (in_hi - in_lo);
}
