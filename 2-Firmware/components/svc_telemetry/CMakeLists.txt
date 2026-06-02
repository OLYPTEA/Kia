idf_component_register(
    SRCS "svc_telemetry.c"
    INCLUDE_DIRS "include"
    REQUIRES common drv_led drv_ina219 svc_motion svc_safety svc_comm dom_kinematics esp_timer
)
