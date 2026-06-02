idf_component_register(
    SRCS "drv_ina219.c"
    INCLUDE_DIRS "include"
    REQUIRES common driver esp_driver_i2c
)
