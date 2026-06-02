#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "svc_comm.h"

typedef enum {
    OTA_IDLE = 0,
    OTA_RECEIVING,
    OTA_FINALISING,
    OTA_FAILED,
} svc_ota_state_t;

typedef struct {
    uint32_t total_bytes;
    uint32_t received_bytes;
    uint16_t chunk_size;
    uint32_t crc32_expected;
} svc_ota_session_t;

esp_err_t svc_ota_init(void);

/* Called by proto_binary opcode handlers. */
esp_err_t svc_ota_begin(comm_transport_id_t t, uint32_t total_bytes, uint16_t chunk_size,
                        uint32_t crc32_expected);
esp_err_t svc_ota_chunk(comm_transport_id_t t, uint32_t offset, const uint8_t *data, size_t n);
esp_err_t svc_ota_end(comm_transport_id_t t);
esp_err_t svc_ota_abort(void);

svc_ota_state_t svc_ota_get_state(void);
uint32_t        svc_ota_received_bytes(void);
uint32_t        svc_ota_total_bytes(void);
