#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"

void app_main(void)
{
    printf("\n\n=== Kia Arm Unit Tests ===\n");
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();
    vTaskDelay(pdMS_TO_TICKS(100));
}
