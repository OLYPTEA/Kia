#include "kia_log.h"

void kia_log_init(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
}

void kia_log_set_level(const char *tag, esp_log_level_t lvl)
{
    esp_log_level_set(tag, lvl);
}
