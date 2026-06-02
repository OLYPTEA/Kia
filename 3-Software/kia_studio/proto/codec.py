"""Frame encode/decode + Status payload codec + incremental RX streamer.

Frame body (after COBS decode), little-endian:
    u8 op | u8 flags | u16 seq | u32 crc32 | u8 payload[]
    crc32 = CRC32-ISO over (op|flags|seq|payload)

Telemetry/STATUS payload (44 bytes):
    4xf32 joints | f32 grip | 3xf32 xyz | f32 pitch |
    u32 cur_ma | u8 fault | u8 src | u8 idle | u8 _pad
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Optional

from .framing import cobs_decode, cobs_encode, crc32
from .opcodes import Fault, Mode, RSP, Src

_HDR = struct.Struct("<BBH")        # op, flags, seq
_CRC = struct.Struct("<I")
_HEADER_LEN = _HDR.size + _CRC.size  # 8
_STATUS_LEN = 44


@dataclass(slots=True)
class Status:
    """Decoded telemetry / status snapshot."""
    joints: tuple[float, float, float, float]
    grip: float
    xyz: tuple[float, float, float]
    pitch: float
    current_ma: int
    fault: int
    src: int
    idle: bool
    raw: bytes = field(default=b"", repr=False)

    @property
    def fault_enum(self) -> Fault:
        try:
            return Fault(self.fault)
        except ValueError:
            return Fault.INTERNAL

    @property
    def src_enum(self) -> Src:
        try:
            return Src(self.src)
        except ValueError:
            return Src.NONE

    @property
    def in_fault(self) -> bool:
        return self.fault != Fault.NONE


def build_frame(op: int, seq: int, body: bytes = b"") -> bytes:
    """Build a COBS-framed binary command (header + crc + body), 0x00-terminated."""
    header = _HDR.pack(int(op), 0, seq & 0xFFFF)
    crc = crc32(header + body)
    return cobs_encode(header + _CRC.pack(crc) + body)


def parse_frame(raw: bytes) -> Optional[tuple[int, int, bytes]]:
    """Validate a decoded frame; return (op, seq, body) or None on CRC/length error."""
    if len(raw) < _HEADER_LEN:
        return None
    op, flags, seq = _HDR.unpack_from(raw, 0)
    crc_recv = _CRC.unpack_from(raw, 4)[0]
    body = raw[_HEADER_LEN:]
    if crc32(_HDR.pack(op, flags, seq) + body) != crc_recv:
        return None
    return op, seq, body


def decode_status(body: bytes) -> Optional[Status]:
    """Decode a STATUS/TELEMETRY payload into a Status, or None if too short."""
    if len(body) < _STATUS_LEN:
        return None
    joints = struct.unpack_from("<4f", body, 0)
    grip = struct.unpack_from("<f", body, 16)[0]
    xyz = struct.unpack_from("<3f", body, 20)
    pitch = struct.unpack_from("<f", body, 32)[0]
    cur = struct.unpack_from("<I", body, 36)[0]
    fault, src, idle, _pad = body[40], body[41], body[42], body[43]
    return Status(joints, grip, xyz, pitch, int(cur), fault, src, bool(idle), bytes(body[:_STATUS_LEN]))


class FrameStreamer:
    """Incremental RX parser splitting a mixed byte stream into binary frames + text lines.

    Binary frames are COBS, 0x00-terminated. Text lines (CLI/debug) are \\n-terminated.
    Demux rule: a segment is treated as text only while it contains *no* non-printable
    byte. A binary frame always carries non-printable bytes (CRC/floats/opcode) before
    any embedded 0x0A, so a real frame is never mis-flushed as text. Call `feed(chunk)`
    -> (frames, lines).
    """

    def __init__(self) -> None:
        self._seg = bytearray()      # bytes accumulated since last 0x00 delimiter
        self._line = bytearray()     # printable run for the current text line
        self._has_binary = False     # segment contains a non-printable byte

    def feed(self, data: bytes) -> tuple[list[tuple[int, int, bytes]], list[str]]:
        frames: list[tuple[int, int, bytes]] = []
        lines: list[str] = []
        for b in data:
            if b == 0x00:
                if self._seg:
                    decoded = cobs_decode(bytes(self._seg) + b"\x00")
                    if decoded:
                        f = parse_frame(decoded)
                        if f:
                            frames.append(f)
                self._reset_seg()
                continue
            self._seg.append(b)
            if b == 0x0A:                       # \n — possible text line end
                if not self._has_binary:
                    if self._line:
                        lines.append(self._line.decode("utf-8", "replace"))
                    self._line.clear()
                    self._seg.clear()
            elif b == 0x0D:                      # \r — ignored for line content
                pass
            elif 0x20 <= b < 0x7F or b == 0x09:  # printable / tab
                self._line.append(b)
            else:                                # non-printable -> this is a binary frame
                self._has_binary = True
                self._line.clear()
        return frames, lines

    def _reset_seg(self) -> None:
        self._seg.clear()
        self._line.clear()
        self._has_binary = False

    def reset(self) -> None:
        self._reset_seg()
