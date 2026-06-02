"""KiaClient — high-level command + telemetry API over any Transport.

UI-agnostic and thread-safe to construct. RX callbacks fire on the transport's
read thread; a GUI layer should marshal them onto its own loop (Qt signals, etc.).
"""
from __future__ import annotations

import struct
import threading
import time
from typing import Callable, Optional

from .codec import FrameStreamer, Status, build_frame, decode_status
from .opcodes import OP, RSP, Mode

StatusCb = Callable[[Status], None]
AckCb = Callable[[int], None]            # (seq)
NackCb = Callable[[int, int], None]      # (seq, reason)
PongCb = Callable[[tuple[int, int, int]], None]
TextCb = Callable[[str], None]


class KiaClient:
    """Encodes commands, decodes responses/telemetry, dispatches via callbacks."""

    def __init__(self, transport):
        self.t = transport
        self._streamer = FrameStreamer()
        self._seq = 0
        self._seq_lock = threading.Lock()
        self.last_telemetry: Optional[Status] = None

        # hooks (set by the application layer)
        self.on_telemetry: Optional[StatusCb] = None
        self.on_status: Optional[StatusCb] = None
        self.on_ack: Optional[AckCb] = None
        self.on_nack: Optional[NackCb] = None
        self.on_pong: Optional[PongCb] = None
        self.on_text: Optional[TextCb] = None

        self.t.set_on_rx(self._on_rx)

    # ---- sequence ----
    def _next_seq(self) -> int:
        with self._seq_lock:
            self._seq = (self._seq + 1) & 0xFFFF or 1
            return self._seq

    # ---- RX path ----
    def _on_rx(self, data: bytes) -> None:
        frames, lines = self._streamer.feed(data)
        for op, seq, body in frames:
            self._dispatch(op, seq, body)
        if lines and self.on_text:
            for ln in lines:
                self.on_text(ln)

    def _dispatch(self, op: int, seq: int, body: bytes) -> None:
        if op in (RSP.TELEMETRY, RSP.STATUS):
            st = decode_status(body)
            if st:
                self.last_telemetry = st
                if op == RSP.TELEMETRY and self.on_telemetry:
                    self.on_telemetry(st)
                if op == RSP.STATUS and self.on_status:
                    self.on_status(st)
        elif op == RSP.PONG and len(body) >= 3:
            if self.on_pong:
                self.on_pong((body[0], body[1], body[2]))
        elif op == RSP.ACK:
            if self.on_ack:
                self.on_ack(seq)
        elif op == RSP.NACK:
            if self.on_nack:
                self.on_nack(seq, body[0] if body else 0xFF)

    # ---- TX: raw ----
    def _send(self, op: int, body: bytes = b"") -> int:
        seq = self._next_seq()
        self.t.send(build_frame(op, seq, body))
        return seq

    def text(self, line: str) -> None:
        if not line.endswith("\n"):
            line += "\n"
        self.t.send(line.encode())

    # ---- TX: system ----
    def ping(self) -> int:           return self._send(OP.PING)
    def get_status(self) -> int:     return self._send(OP.GET_STATUS)

    # ---- TX: motion ----
    def set_joint(self, idx: int, deg: float, dur: float = 1.0) -> int:
        return self._send(OP.SET_JOINT, struct.pack("<Bff", idx, deg, dur))

    def set_joints(self, j0: float, j1: float, j2: float, j3: float,
                   grip: float, dur: float = 2.0) -> int:
        return self._send(OP.SET_JOINTS, struct.pack("<6f", j0, j1, j2, j3, grip, dur))

    def set_xyz(self, x: float, y: float, z: float, pitch: float, dur: float = 2.0) -> int:
        return self._send(OP.SET_XYZ, struct.pack("<5f", x, y, z, pitch, dur))

    def home(self) -> int:           return self._send(OP.HOME)
    def hold(self) -> int:           return self._send(OP.HOLD)

    # ---- TX: mode / safety ----
    def mode_set(self, mode: Mode | int) -> int:
        return self._send(OP.MODE_SET, struct.pack("<B", int(mode)))

    def arm(self) -> int:            return self._send(OP.ARM)
    def clear(self) -> int:          return self._send(OP.CLEAR)

    # ---- TX: storage ----
    def save(self) -> int:           return self._send(OP.SAVE)
    def load(self) -> int:           return self._send(OP.LOAD)
    def factory(self) -> int:        return self._send(OP.FACTORY)

    # ---- TX: calibration ----
    def cal_joint(self, idx: int, which: int) -> int:
        """which: 0=ZERO 1=MIN 2=MAX."""
        return self._send(OP.CAL_J, struct.pack("<BB", idx, which))

    def cal_pot(self, idx: int, which: int) -> int:
        """which: 0=MIN 1=MAX."""
        return self._send(OP.CAL_P, struct.pack("<BB", idx, which))

    def cal_geom(self, base_h: float, upper: float, fore: float, tool: float) -> int:
        return self._send(OP.CAL_GEOM, struct.pack("<4f", base_h, upper, fore, tool))

    # ---- TX: telemetry rate ----
    def set_telemetry_rate(self, hz: int) -> int:
        return self._send(OP.TLM_RATE, struct.pack("<H", hz))

    # ---- TX: OTA ----
    def ota_push(self, fw_path: str, chunk_size: int = 1024,
                 progress: Optional[Callable[[int, int], None]] = None,
                 inter_chunk_s: float = 0.005) -> None:
        from .framing import crc32
        with open(fw_path, "rb") as f:
            data = f.read()
        total = len(data)
        self._send(OP.OTA_BEGIN, struct.pack("<IHI", total, chunk_size, crc32(data)))
        time.sleep(0.2)
        off = 0
        while off < total:
            n = min(chunk_size, total - off)
            self._send(OP.OTA_CHUNK, struct.pack("<I", off) + data[off:off + n])
            off += n
            if progress:
                progress(off, total)
            time.sleep(inter_chunk_s)
        self._send(OP.OTA_END)
