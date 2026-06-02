idf_component_register(
    SRCS "app_mode_fsm.c"
    INCLUDE_DIRS "include"
    REQUIRES common drv_buttons drv_led drv_pot svc_motion svc_safety esp_event esp_timer
)
