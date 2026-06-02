idf_component_register(
    SRCS "drv_servo.c"
    INCLUDE_DIRS "include"
    REQUIRES common driver esp_driver_mcpwm
    PRIV_REQUIRES esp_timer
)
