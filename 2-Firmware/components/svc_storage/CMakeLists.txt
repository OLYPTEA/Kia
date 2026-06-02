idf_component_register(
    SRCS "svc_storage.c"
    INCLUDE_DIRS "include"
    REQUIRES common nvs_flash drv_servo drv_pot
)
