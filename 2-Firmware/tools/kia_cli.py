#!/usr/bin/env python3
"""Kia Arm host CLI — text + binary protocol over USB CDC or BLE."""
from __future__ import annotations

import argparse
import asyncio
import hashlib
import struct
import sys
import threading
import time
import zlib
from dataclasses import dataclass
from typing import Optional

try:
    import serial  # type: ignore
except ImportError:
    serial = None  # type: ignore

try:
    from bleak import BleakClient, BleakScanner  # type: ignore
except ImportError:
    BleakClient = None  # type: ignore


# === Protocol ============================================================
SVC_UUID = "00000000-0000-0000-0010-00214d52415f4b49"  # placeholder GUID format
RX_UUID  = "00000001-0000-0000-0010-00214d52415f4b49"
TX_UUID  = "00000002-0000-0000-0010-00214d52415f4b49"

OP = {
    "PING":         0x01,
    "GET_STATUS":   0x02,
    "SET_JOINT":    0x10,
    "SET_JOINTS":   0x11,
    "SET_XYZ":      0x12,
    "HOME":         0x13,
    "HOLD":         0x14,
    "MODE_SET":     0x20,
    "ARM":          0x21,
    "CLEAR":        0x22,
    "SAVE":         0x30,
    "LOAD":         0x31,
    "FACTORY":      0x3F,
    "OTA_BEGIN":    0x50,
    "OTA_CHUNK":    0x51,
    "OTA_END":      0x52,
    "OTA_ABORT":    0x53,
}

NACK_REASON = {
    0x01: "BAD_OP", 0x02: "BAD_LEN", 0x03: "BAD_ARG", 0x04: "LIMIT",
    0x05: "UNREACHABLE", 0x06: "FAULT", 0x07: "BUSY", 0x08: "NOMEM",
    0x09: "BAD_STATE", 0xFE: "NOT_IMPL", 0xFF: "INTERNAL",
}


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    out.append(0x00)
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    if len(data) < 2 or data[-1] != 0x00:
        return b""
    out = bytearray()
    i = 0
    end = len(data) - 1
    while i < end:
        code = data[i]
        if code == 0:
            return b""
        i += 1
        for _ in range(code - 1):
            if i >= end:
                return bytes(out)
            out.append(data[i])
            i += 1
        if code != 0xFF and i < end:
            out.append(0)
    return bytes(out)


def build_frame(op: int, seq: int, body: bytes = b"") -> bytes:
    header = struct.pack("<BBH", op, 0, seq)
    crc    = zlib.crc32(header + body) & 0xFFFFFFFF
    raw    = header + struct.pack("<I", crc) + body
    return cobs_encode(raw)


def parse_frame(raw: bytes) -> Optional[tuple[int, int, bytes]]:
    if len(raw) < 8:
        return None
    op, flags, seq = struct.unpack_from("<BBH", raw, 0)
    crc_recv = struct.unpack_from("<I", raw, 4)[0]
    body     = raw[8:]
    crc_calc = zlib.crc32(struct.pack("<BBH", op, flags, seq) + body) & 0xFFFFFFFF
    if crc_recv != crc_calc:
        return None
    return op, seq, body


# === Transport abstraction ===============================================
class Transport:
    def send(self, data: bytes) -> None: raise NotImplementedError
    def close(self) -> None: ...
    def set_on_rx(self, cb) -> None: ...


class SerialTransport(Transport):
    def __init__(self, port: str, baud: int = 115200):
        if serial is None:
            raise RuntimeError("pyserial not installed (pip install pyserial)")
        self.s   = serial.Serial(port, baud, timeout=0.05)
        self.on_rx = None
        self.alive = True
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def send(self, data: bytes) -> None:
        self.s.write(data)

    def set_on_rx(self, cb):
        self.on_rx = cb

    def _rx_loop(self):
        buf = bytearray()
        while self.alive:
            chunk = self.s.read(256)
            if chunk and self.on_rx:
                self.on_rx(bytes(chunk))

    def close(self):
        self.alive = False
        self.s.close()


class BleTransport(Transport):
    def __init__(self, name_filter: str = "KiaArm"):
        if BleakClient is None:
            raise RuntimeError("bleak not installed (pip install bleak)")
        self.client: Optional[BleakClient] = None
        self.name_filter = name_filter
        self.on_rx = None
        self.loop  = asyncio.new_event_loop()
        self.thr   = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thr.start()
        fut = asyncio.run_coroutine_threadsafe(self._connect(), self.loop)
        fut.result(timeout=15)

    async def _connect(self):
        print("scanning for BLE...")
        devs = await BleakScanner.discover(timeout=5)
        tgt  = None
        for d in devs:
            if d.name and self.name_filter in d.name:
                tgt = d; break
        if not tgt:
            raise RuntimeError(f"BLE device matching '{self.name_filter}' not found")
        self.client = BleakClient(tgt)
        await self.client.connect()
        await self.client.start_notify(TX_UUID, self._notify)
        print(f"connected to {tgt.name} [{tgt.address}]")

    def _notify(self, sender, data: bytes):
        if self.on_rx:
            self.on_rx(bytes(data))

    def send(self, data: bytes) -> None:
        asyncio.run_coroutine_threadsafe(
            self.client.write_gatt_char(RX_UUID, data, response=False), self.loop
        )

    def set_on_rx(self, cb):
        self.on_rx = cb

    def close(self):
        if self.client and self.client.is_connected:
            asyncio.run_coroutine_threadsafe(self.client.disconnect(), self.loop)


# === Client API ==========================================================
@dataclass
class Status:
    joints: tuple[float, float, float, float]
    grip: float
    xyz: tuple[float, float, float]
    pitch: float
    current_ma: int
    fault: int
    src: int
    idle: bool


class KiaClient:
    def __init__(self, transport: Transport):
        self.t   = transport
        self.t.set_on_rx(self._on_rx)
        self.rx_buf = bytearray()
        self.text_buf = bytearray()
        self.seq = 0
        self.pending: dict[int, asyncio.Future] = {}
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        self.last_telemetry: Optional[Status] = None

    def _next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFFFF or 1
        return self.seq

    def _on_rx(self, data: bytes):
        for b in data:
            if b == 0x00:
                if self.rx_buf:
                    self.rx_buf.append(0x00)
                    decoded = cobs_decode(bytes(self.rx_buf))
                    self.rx_buf.clear()
                    if decoded:
                        self._dispatch_binary(decoded)
            else:
                if b in (ord('\r'), ord('\n')):
                    if self.text_buf:
                        line = self.text_buf.decode(errors="replace")
                        print(f"< {line}")
                        self.text_buf.clear()
                else:
                    self.text_buf.append(b)
                self.rx_buf.append(b)

    def _dispatch_binary(self, raw: bytes):
        f = parse_frame(raw)
        if not f:
            return
        op, seq, body = f
        if op == 0xF0 and len(body) >= 44:
            j = struct.unpack_from("<4f", body, 0)
            g = struct.unpack_from("<f",  body, 16)[0]
            x = struct.unpack_from("<3f", body, 20)
            pit = struct.unpack_from("<f", body, 32)[0]
            cur = struct.unpack_from("<I", body, 36)[0]
            fault, src, idle, _ = body[40], body[41], body[42], body[43]
            self.last_telemetry = Status(j, g, x, pit, int(cur), fault, src, bool(idle))
        if op == 0xFF and body:
            print(f"< NACK seq={seq} reason={NACK_REASON.get(body[0], hex(body[0]))}")

    # ---- text commands ----
    def text(self, line: str):
        if not line.endswith("\n"):
            line += "\n"
        self.t.send(line.encode())

    # ---- binary commands ----
    def ping(self):
        self.t.send(build_frame(OP["PING"], self._next_seq()))

    def home(self):
        self.t.send(build_frame(OP["HOME"], self._next_seq()))

    def set_joint(self, idx: int, deg: float, dur: float = 1.0):
        body = struct.pack("<Bff", idx, deg, dur)
        self.t.send(build_frame(OP["SET_JOINT"], self._next_seq(), body))

    def set_joints(self, j0, j1, j2, j3, g, dur: float = 2.0):
        body = struct.pack("<6f", j0, j1, j2, j3, g, dur)
        self.t.send(build_frame(OP["SET_JOINTS"], self._next_seq(), body))

    def set_xyz(self, x, y, z, pitch, dur: float = 2.0):
        body = struct.pack("<5f", x, y, z, pitch, dur)
        self.t.send(build_frame(OP["SET_XYZ"], self._next_seq(), body))

    def save(self):
        self.t.send(build_frame(OP["SAVE"], self._next_seq()))

    def arm(self):
        self.t.send(build_frame(OP["ARM"], self._next_seq()))

    def clear(self):
        self.t.send(build_frame(OP["CLEAR"], self._next_seq()))

    # ---- OTA ----
    def ota_push(self, fw_path: str, chunk_size: int = 1024):
        data = open(fw_path, "rb").read()
        total = len(data)
        crc   = zlib.crc32(data) & 0xFFFFFFFF
        print(f"OTA {total} bytes CRC32=0x{crc:08X}")
        self.t.send(build_frame(OP["OTA_BEGIN"], self._next_seq(),
                                struct.pack("<IHI", total, chunk_size, crc)))
        time.sleep(0.2)
        off = 0
        while off < total:
            n = min(chunk_size, total - off)
            self.t.send(build_frame(OP["OTA_CHUNK"], self._next_seq(),
                                    struct.pack("<I", off) + data[off:off + n]))
            off += n
            if off % (chunk_size * 16) == 0:
                print(f"  {off}/{total}  ({100 * off // total}%)")
            time.sleep(0.005)
        self.t.send(build_frame(OP["OTA_END"], self._next_seq()))
        print("OTA push complete — wait for ACK then power-cycle")


# === Interactive loop ====================================================
def repl(c: KiaClient):
    print("Kia Arm CLI. Type 'help' for commands, 'q' to quit.")
    while True:
        try:
            line = input("kia> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        if line in ("q", "quit", "exit"):
            break
        if line == "help":
            print("text mode commands: PING, STATUS, J <i> <deg> [dur], XYZ <x y z p [dur]>,")
            print("                    MODE MANUAL|APP|HOLD, HOME, HOLD, ARM, CLEAR, SAVE,")
            print("                    CAL J<i> {ZERO|MIN|MAX}, CAL P<i> {MIN|MAX}, CAL GEOM bh up fo to")
            print("binary helpers:  bping, bhome, bjoint <i> <deg>, bxyz <x y z p>, bsave, ota <path>")
            continue
        if line.startswith("bping"):  c.ping(); continue
        if line.startswith("bhome"):  c.home(); continue
        if line.startswith("bsave"):  c.save(); continue
        if line.startswith("barm"):   c.arm();  continue
        if line.startswith("bclear"): c.clear(); continue
        if line.startswith("bjoint"):
            _, i, d = line.split(); c.set_joint(int(i), float(d)); continue
        if line.startswith("bxyz"):
            _, x, y, z, p = line.split(); c.set_xyz(float(x), float(y), float(z), float(p)); continue
        if line.startswith("ota "):
            _, path = line.split(maxsplit=1); c.ota_push(path); continue
        if line == "tlm":
            print(c.last_telemetry); continue
        c.text(line)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    s   = sub.add_parser("serial");  s.add_argument("port"); s.add_argument("-b", "--baud", default=115200, type=int)
    b   = sub.add_parser("ble");     b.add_argument("--name", default="KiaArm")
    args = ap.parse_args()

    transport: Transport
    if args.cmd == "serial":
        transport = SerialTransport(args.port, args.baud)
    else:
        transport = BleTransport(args.name)

    client = KiaClient(transport)
    try:
        repl(client)
    finally:
        transport.close()


if __name__ == "__main__":
    main()
