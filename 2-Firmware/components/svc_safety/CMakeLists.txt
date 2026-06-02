idf_component_register(
    SRCS "svc_safety.c"
    INCLUDE_DIRS "include"
    REQUIRES common drv_servo drv_ina219 drv_led drv_buzzer esp_event esp_timer esp_task_wdt
)
