idf_component_register(
    SRCS "svc_ota.c"
    INCLUDE_DIRS "include"
    REQUIRES common app_update esp_app_format esp_event svc_comm
)
