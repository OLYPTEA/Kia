#include "app_calibration.h"

#include "drv_pot.h"
#include "drv_servo.h"
#include "kia_err.h"
#include "kinematics.h"
#include "svc_storage.h"

esp_err_t app_calibration_servo(uint8_t idx, app_cal_servo_op_t op)
{
    drv_servo_cal_t c;
    KIA_RET(drv_servo_get_calibration(idx, &c));
    float cur = drv_servo_get_angle(idx);
    switch (op) {
        case APP_CAL_SERVO_ZERO: c.offset_deg   = -cur; break;
        case APP_CAL_SERVO_MIN:  c.angle_min_deg = cur; break;
        case APP_CAL_SERVO_MAX:  c.angle_max_deg = cur; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    if (c.angle_max_deg <= c.angle_min_deg) return ESP_ERR_INVALID_ARG;
    return drv_servo_set_calibration(idx, &c);
}

esp_err_t app_calibration_pot(uint8_t idx, app_cal_pot_op_t op)
{
    drv_pot_cal_t c;
    KIA_RET(drv_pot_get_calibration(idx, &c));
    uint16_t v = drv_pot_get_raw(idx);
    if (op == APP_CAL_POT_MIN) c.raw_min = v;
    else if (op == APP_CAL_POT_MAX) c.raw_max = v;
    else return ESP_ERR_INVALID_ARG;
    if (c.raw_max <= c.raw_min + 100) return ESP_ERR_INVALID_ARG;
    return drv_pot_set_calibration(idx, c);
}

esp_err_t app_calibration_set_geometry(float bh, float up, float fo, float to)
{
    if (bh <= 0 || up <= 0 || fo <= 0 || to <= 0) return ESP_ERR_INVALID_ARG;
    kin_set_geometry(bh, up, fo, to);
    return ESP_OK;
}

esp_err_t app_calibration_persist(void)
{
    return svc_storage_save_calibration();
}

esp_err_t app_calibration_servo_capture_zero(uint8_t idx)
{
    return app_calibration_servo(idx, APP_CAL_SERVO_ZERO);
}

esp_err_t app_calibration_pot_capture_endpoints(uint8_t idx)
{
    drv_pot_cal_t c;
    if (drv_pot_get_calibration(idx, &c) != ESP_OK) return ESP_ERR_INVALID_ARG;
    uint16_t v = drv_pot_get_raw(idx);
    if (v < (c.raw_min + c.raw_max) / 2) c.raw_min = v;
    else c.raw_max = v;
    return drv_pot_set_calibration(idx, c);
}
