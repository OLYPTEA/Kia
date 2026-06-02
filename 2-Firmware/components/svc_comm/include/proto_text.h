#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "svc_comm.h"

/* Feed bytes received from one transport into the text parser.
 * Replies are dispatched back through the same transport. */
void proto_text_feed(comm_transport_id_t src, const uint8_t *data, size_t n);
