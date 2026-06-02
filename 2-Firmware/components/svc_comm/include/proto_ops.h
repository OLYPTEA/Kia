#pragma once

#include <stdint.h>

/* Binary protocol opcodes. Top bit = direction:
 *   0x00..0x7F : host -> device (request)
 *   0x80..0xFF : device -> host (response / async)
 */

typedef enum {
    /* System */
    OP_PING        = 0x01,
    OP_GET_STATUS  = 0x02,
    OP_GET_INFO    = 0x03,
    /* Motion */
    OP_SET_JOINT   = 0x10,
    OP_SET_JOINTS  = 0x11,
    OP_SET_XYZ     = 0x12,
    OP_HOME        = 0x13,
    OP_HOLD        = 0x14,
    OP_QUEUE_PUSH  = 0x15,
    OP_QUEUE_CLEAR = 0x16,
    /* Mode / safety */
    OP_MODE_SET    = 0x20,
    OP_ARM         = 0x21,
    OP_CLEAR_FAULT = 0x22,
    /* Storage */
    OP_SAVE_CAL    = 0x30,
    OP_LOAD_CAL    = 0x31,
    OP_FACTORY_RST = 0x3F,
    /* Calibration */
    OP_CAL_SERVO_ZERO  = 0x40,
    OP_CAL_POT_MIN     = 0x41,
    OP_CAL_POT_MAX     = 0x42,
    OP_CAL_SET_GEOM    = 0x43,
    /* OTA */
    OP_OTA_BEGIN  = 0x50,
    OP_OTA_CHUNK  = 0x51,
    OP_OTA_END    = 0x52,
    OP_OTA_ABORT  = 0x53,
    /* Telemetry stream control */
    OP_TLM_RATE   = 0x60,

    /* Async / replies */
    OP_PONG       = 0x81,
    OP_STATUS     = 0x82,
    OP_INFO       = 0x83,
    OP_TELEMETRY  = 0xF0,
    OP_EVENT      = 0xF1,
    OP_ACK        = 0xFE,
    OP_NACK       = 0xFF,
} kia_op_t;

/* NACK error codes (1 byte body). */
typedef enum {
    NACK_BAD_OP      = 0x01,
    NACK_BAD_LEN     = 0x02,
    NACK_BAD_ARG     = 0x03,
    NACK_LIMIT       = 0x04,
    NACK_UNREACHABLE = 0x05,
    NACK_FAULT       = 0x06,
    NACK_BUSY        = 0x07,
    NACK_NOMEM       = 0x08,
    NACK_BAD_STATE   = 0x09,
    NACK_NOT_IMPL    = 0xFE,
    NACK_INTERNAL    = 0xFF,
} kia_nack_t;
