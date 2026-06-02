idf_component_register(
    SRCS "drv_buzzer.c"
    INCLUDE_DIRS "include"
    REQUIRES common driver esp_driver_ledc esp_timer
)
