#pragma once

#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(KIA_EVENT_BASE);

typedef enum {
    KIA_EVT_BOOT_DONE = 0,
    KIA_EVT_ESTOP_TRIGGERED,
    KIA_EVT_ESTOP_CLEARED,
    KIA_EVT_FAULT_RAISED,
    KIA_EVT_FAULT_CLEARED,
    KIA_EVT_MODE_CHANGED,
    KIA_EVT_CALIBRATION_DONE,
    KIA_EVT_TARGET_REACHED,
    KIA_EVT_OVERCURRENT,
    KIA_EVT_LINK_UP,
    KIA_EVT_LINK_DOWN,
    KIA_EVT_OTA_BEGIN,
    KIA_EVT_OTA_DONE,
    KIA_EVT__COUNT
} kia_event_id_t;

typedef struct {
    uint32_t code;
    uint32_t aux;
} kia_evt_fault_t;

typedef struct {
    uint8_t prev;
    uint8_t next;
} kia_evt_mode_t;
