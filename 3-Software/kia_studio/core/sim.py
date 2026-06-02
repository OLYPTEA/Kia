"""SimDevice / SimTransport — an in-process emulator of the firmware.

Lets the whole GUI run without the physical board (PCB not fabricated yet).
Plugs in wherever a Transport is expected. Decodes host commands, integrates a
simple slew-limited motion model, and pushes TELEMETRY at a fixed rate — using the
real protocol codec and the real FK, so nothing is faked at the wire level.
"""
from __future__ import annotations

import struct
import threading
import time
from typing import Callable, Optional

from ..proto.codec import build_frame, decode_status, parse_frame
from ..proto.framing import cobs_decode
from ..proto.opcodes import OP, RSP, Fault, Mode, Src
from .joints import GRIP, JOINTS
from .kinematics import Geometry, fk_pose, solve_ik, load_geometry

_FW = (1, 0, 0)


class SimTransport:
    """Transport-compatible firmware emulator (USB-CDC stand-in)."""

    def __init__(self, *, telemetry_hz: int = 20, slew_dps: float = 240.0):
        self._on_rx: Optional[Callable[[bytes], None]] = None
        self._geo = load_geometry()   # calibrated link geometry if exported, else nominal
        self._slew = slew_dps
        self._dt = 1.0 / telemetry_hz
        # state: [base, shoulder, elbow, wrist, grip] target/current (deg)
        home = [j.home_deg for j in JOINTS] + [GRIP.home_deg]
        self._cur = list(home)
        self._tgt = list(home)
        self._mode = Mode.APP
        self._fault = Fault.NONE
        self._src = Src.APP
        self._lock = threading.Lock()
        self._open = True
        self._thr = threading.Thread(target=self._tick_loop, name="kia-sim", daemon=True)
        self._thr.start()

    # ---- Transport API ----
    def set_on_rx(self, cb) -> None:
        self._on_rx = cb

    def send(self, data: bytes) -> None:
        # host -> device : may carry text (ignored) or COBS binary frames
        self._consume(data)

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    # ---- command handling ----
    def _consume(self, data: bytes) -> None:
        seg = bytearray()
        for b in data:
            if b == 0x00:
                if seg:
                    decoded = cobs_decode(bytes(seg) + b"\x00")
                    f = parse_frame(decoded) if decoded else None
                    if f:
                        self._handle(*f)
                seg.clear()
            else:
                seg.append(b)

    def _emit(self, op: int, seq: int, body: bytes = b"") -> None:
        if self._on_rx:
            self._on_rx(build_frame(op, seq, body))

    def _ack(self, seq: int) -> None:
        self._emit(RSP.ACK, seq)

    def _handle(self, op: int, seq: int, body: bytes) -> None:
        with self._lock:
            if op == OP.PING:
                self._emit(RSP.PONG, seq, bytes(_FW))
            elif op == OP.GET_STATUS:
                self._emit(RSP.STATUS, seq, self._status_payload())
            elif op == OP.SET_JOINT and len(body) >= 9:
                if not self._motion_allowed(seq):
                    return
                idx, deg, _dur = struct.unpack_from("<Bff", body, 0)
                if 0 <= idx < len(self._tgt):
                    self._tgt[idx] = self._clamp(idx, deg)
                self._ack(seq)
            elif op == OP.SET_JOINTS and len(body) >= 24:
                if not self._motion_allowed(seq):
                    return
                vals = struct.unpack_from("<6f", body, 0)
                for i in range(5):
                    self._tgt[i] = self._clamp(i, vals[i])
                self._ack(seq)
            elif op == OP.SET_XYZ and len(body) >= 20:
                if not self._motion_allowed(seq):
                    return
                x, y, z, pitch, _dur = struct.unpack_from("<5f", body, 0)
                res = solve_ik(x, y, z, pitch, self._geo)
                if not res.ok:
                    self._emit(RSP.NACK, seq, bytes([0x05]))   # UNREACHABLE
                    return
                self._tgt[0:4] = list(res.joints())
                self._ack(seq)
            elif op == OP.HOME:
                if not self._motion_allowed(seq):
                    return
                self._tgt = [j.home_deg for j in JOINTS] + [GRIP.home_deg]
                self._ack(seq)
            elif op == OP.HOLD:
                self._tgt = list(self._cur)
                self._ack(seq)
            elif op == OP.MODE_SET and body:
                try:
                    self._mode = Mode(body[0])
                    self._src = Src.APP if self._mode == Mode.APP else (
                        Src.MANUAL if self._mode == Mode.MANUAL else Src.SAFETY)
                except ValueError:
                    pass
                self._ack(seq)
            elif op == OP.ARM:
                # re-arm after a HOLD/STOP -> resume APP control
                self._mode = Mode.APP
                self._src = Src.APP
                self._ack(seq)
            elif op == OP.CLEAR:
                self._fault = Fault.NONE
                self._ack(seq)
            elif op in (OP.SAVE, OP.LOAD, OP.FACTORY, OP.TLM_RATE):
                self._ack(seq)
            else:
                self._emit(RSP.NACK, seq, bytes([0xFE]))   # NOT_IMPL

    def _motion_allowed(self, seq: int) -> bool:
        """Reject motion commands while held (STOP -> MODE HOLD). NACK BAD_STATE."""
        if self._mode == Mode.HOLD:
            self._emit(RSP.NACK, seq, bytes([0x09]))       # BAD_STATE
            return False
        return True

    def _clamp(self, idx: int, deg: float) -> float:
        spec = JOINTS[idx] if idx < len(JOINTS) else GRIP
        return spec.clamp(deg)

    # ---- motion + telemetry ----
    def _tick_loop(self) -> None:
        step = self._slew * self._dt
        while self._open:
            with self._lock:
                for i in range(len(self._cur)):
                    d = self._tgt[i] - self._cur[i]
                    if abs(d) <= step:
                        self._cur[i] = self._tgt[i]
                    else:
                        self._cur[i] += step if d > 0 else -step
                payload = self._status_payload()
            self._emit(RSP.TELEMETRY, 0, payload)
            time.sleep(self._dt)

    def _status_payload(self) -> bytes:
        j = self._cur
        p = fk_pose(j[0], j[1], j[2], j[3], self._geo)
        idle = all(abs(self._tgt[i] - self._cur[i]) < 1e-3 for i in range(len(self._cur)))
        return (struct.pack("<4f", j[0], j[1], j[2], j[3])
                + struct.pack("<f", j[4])
                + struct.pack("<3f", p.x, p.y, p.z)
                + struct.pack("<f", p.pitch)
                + struct.pack("<I", 1200)
                + bytes([int(self._fault), int(self._src), int(idle), 0]))
