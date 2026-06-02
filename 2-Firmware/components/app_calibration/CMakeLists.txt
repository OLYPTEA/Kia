idf_component_register(
    SRCS "app_calibration.c"
    INCLUDE_DIRS "include"
    REQUIRES common drv_servo drv_pot svc_storage
)
