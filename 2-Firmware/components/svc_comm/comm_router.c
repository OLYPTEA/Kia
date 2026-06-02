#include "svc_comm.h"

#include <string.h>

#include "esp_log.h"

#include "kia_err.h"

static const char *TAG = "comm_router";

static comm_send_fn_t s_senders[COMM_TRANSPORT__COUNT];

void svc_comm_register_sender(comm_transport_id_t t, comm_send_fn_t fn)
{
    if (t >= COMM_TRANSPORT__COUNT) return;
    s_senders[t] = fn;
}

esp_err_t svc_comm_send(comm_transport_id_t t, const uint8_t *buf, size_t n)
{
    if (t >= COMM_TRANSPORT__COUNT || !s_senders[t]) return ESP_ERR_INVALID_STATE;
    return s_senders[t](t, buf, n) >= 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t svc_comm_broadcast(const uint8_t *buf, size_t n)
{
    for (int t = 0; t < COMM_TRANSPORT__COUNT; ++t)
        if (s_senders[t]) s_senders[t]((comm_transport_id_t)t, buf, n);
    return ESP_OK;
}

#include "proto_binary.h"
#include "proto_text.h"

void svc_comm_on_rx(comm_transport_id_t t, const uint8_t *buf, size_t n)
{
    /* Detect framing: a leading non-printable byte (< 0x20 except CR/LF/TAB) → binary,
     * otherwise text. Pragmatic heuristic — robust enough for ASCII commands. */
    if (n == 0) return;
    uint8_t c = buf[0];
    if (c >= 0x20 || c == '\r' || c == '\n' || c == '\t') {
        proto_text_feed(t, buf, n);
    } else {
        proto_binary_feed(t, buf, n);
    }
}
