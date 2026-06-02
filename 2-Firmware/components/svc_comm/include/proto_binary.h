#pragma once

#include <stddef.h>
#include <stdint.h>

#include "proto_ops.h"
#include "svc_comm.h"

/* Inbound: feed bytes received from transport into binary parser. */
void proto_binary_feed(comm_transport_id_t src, const uint8_t *data, size_t n);

/* Outbound: encode and send a binary frame on a transport (or all transports). */
int proto_binary_send(comm_transport_id_t t, uint8_t op, uint16_t seq, const void *body, size_t body_n);
int proto_binary_broadcast(uint8_t op, uint16_t seq, const void *body, size_t body_n);

/* Convenience: ACK / NACK to the last received frame. */
int proto_binary_ack(comm_transport_id_t t, uint16_t seq);
int proto_binary_nack(comm_transport_id_t t, uint16_t seq, kia_nack_t reason);

/* Telemetry helper called by svc_telemetry. */
int proto_binary_telemetry_broadcast(uint16_t seq);
