"""Opcodes, response codes and enums — mirror of firmware proto_ops.h.

Keep BYTE-IDENTICAL with components/svc_comm/proto_ops.h.
"""
from __future__ import annotations

from enum import IntEnum


class OP(IntEnum):
    """Host -> device request opcodes."""
    PING       = 0x01
    GET_STATUS = 0x02

    SET_JOINT  = 0x10
    SET_JOINTS = 0x11
    SET_XYZ    = 0x12
    HOME       = 0x13
    HOLD       = 0x14

    MODE_SET   = 0x20
    ARM        = 0x21
    CLEAR      = 0x22

    SAVE       = 0x30
    LOAD       = 0x31
    FACTORY    = 0x3F

    CAL_J      = 0x40
    CAL_P      = 0x41
    CAL_GEOM   = 0x42
    CAL_QUERY  = 0x43

    OTA_BEGIN  = 0x50
    OTA_CHUNK  = 0x51
    OTA_END    = 0x52
    OTA_ABORT  = 0x53

    TLM_RATE   = 0x60


class RSP(IntEnum):
    """Device -> host response / async opcodes."""
    PONG      = 0x81
    STATUS    = 0x82
    TELEMETRY = 0xF0
    ACK       = 0xFE
    NACK      = 0xFF


NACK_REASON: dict[int, str] = {
    0x01: "BAD_OP", 0x02: "BAD_LEN", 0x03: "BAD_ARG", 0x04: "LIMIT",
    0x05: "UNREACHABLE", 0x06: "FAULT", 0x07: "BUSY", 0x08: "NOMEM",
    0x09: "BAD_STATE", 0xFE: "NOT_IMPL", 0xFF: "INTERNAL",
}


class Mode(IntEnum):
    """Mode FSM values (MODE_SET payload byte)."""
    MANUAL = 0
    APP    = 1
    HOLD   = 2

    @property
    def label(self) -> str:
        return self.name.capitalize()


class Fault(IntEnum):
    """Fault codes surfaced in telemetry/status (fault byte)."""
    NONE        = 0
    OVERCURRENT = 1
    OVERCURRENT_AVG = 2
    BROWNOUT    = 3
    ESTOP       = 4
    WDT         = 5
    INTERNAL    = 6

    @property
    def label(self) -> str:
        return self.name.replace("_", " ").title()


class Src(IntEnum):
    """Command source / active controller owner (src byte)."""
    NONE   = 0
    MANUAL = 1
    APP    = 2
    SAFETY = 3
    CALIB  = 4

    @property
    def label(self) -> str:
        return self.name.capitalize()
