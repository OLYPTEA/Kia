#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    COMM_TRANSPORT_USB = 0,
    COMM_TRANSPORT_BLE,
    COMM_TRANSPORT__COUNT,
} comm_transport_id_t;

typedef int (*comm_send_fn_t)(comm_transport_id_t t, const uint8_t *buf, size_t n);

esp_err_t svc_comm_init(void);
esp_err_t svc_comm_send(comm_transport_id_t t, const uint8_t *buf, size_t n);
esp_err_t svc_comm_broadcast(const uint8_t *buf, size_t n);

/* Called by transports when bytes arrive */
void svc_comm_on_rx(comm_transport_id_t t, const uint8_t *buf, size_t n);

void svc_comm_register_sender(comm_transport_id_t t, comm_send_fn_t fn);
