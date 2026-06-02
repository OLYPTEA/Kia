#include "svc_telemetry.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "drv_led.h"
#include "kia_err.h"
#include "kinematics.h"
#include "proto_binary.h"
#include "svc_comm.h"
#include "svc_motion.h"
#include "svc_safety.h"

static struct {
    TaskHandle_t  task;
    uint16_t      rate_hz;
    bool          ready;
} S = {.rate_hz = 20};

static void task(void *arg)
{
    KIA_UNUSED(arg);
    char buf[160];
    TickType_t wake = xTaskGetTickCount();
    uint8_t    led_div = 0;
    while (true) {
        kin_joints_t j; float g; kin_pose_t p;
        svc_motion_get_joints(&j, &g);
        svc_motion_get_pose(&p);

        int n = snprintf(buf, sizeof buf,
                         "TLM J=[%.1f,%.1f,%.1f,%.1f,%.1f] XYZ=[%.1f,%.1f,%.1f] P=%.1f I=%ld F=%d\n",
                         j.base_deg, j.shoulder_deg, j.elbow_deg, j.wrist_deg, g, p.x_mm, p.y_mm,
                         p.z_mm, p.pitch_deg, (long)svc_safety_last_current_ma(),
                         svc_safety_last_fault());
        if (n > 0) svc_comm_broadcast((uint8_t *)buf, n);
        proto_binary_telemetry_broadcast(0);

        if (++led_div >= (S.rate_hz / 2)) {
            drv_led_heartbeat();
            led_div = 0;
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / S.rate_hz));
    }
}

esp_err_t svc_telemetry_init(void)
{
    if (S.ready) return ESP_OK;
    BaseType_t ok = xTaskCreatePinnedToCore(task, "tlm", KIA_STACK_TELEMETRY, NULL, KIA_PRIO_TELEMETRY,
                                            &S.task, KIA_CORE_IO);
    KIA_RET_IF(ok != pdPASS, ESP_ERR_NO_MEM);
    S.ready = true;
    return ESP_OK;
}

esp_err_t svc_telemetry_set_rate_hz(uint16_t hz)
{
    if (hz < 1 || hz > 200) return ESP_ERR_INVALID_ARG;
    S.rate_hz = hz;
    return ESP_OK;
}
