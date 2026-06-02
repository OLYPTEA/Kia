"""Kia wire protocol — transport-agnostic core (no UI, no hardware coupling).

Single source of truth on the host side, shared by the CLI and Kia Studio.
Mirrors firmware: components/svc_comm/proto_ops.h + proto_binary.c.
"""

from .opcodes import OP, RSP, NACK_REASON, Fault, Mode, Src
from .framing import cobs_encode, cobs_decode, crc32
from .codec import Status, build_frame, parse_frame, decode_status, FrameStreamer
from .transport import Transport, SerialTransport, LoopbackTransport
from .client import KiaClient

__all__ = [
    "OP", "RSP", "NACK_REASON", "Fault", "Mode", "Src",
    "cobs_encode", "cobs_decode", "crc32",
    "Status", "build_frame", "parse_frame", "decode_status", "FrameStreamer",
    "Transport", "SerialTransport", "LoopbackTransport",
    "KiaClient",
]
