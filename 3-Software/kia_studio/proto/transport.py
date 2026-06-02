"""Byte transports. UI-agnostic; deliver raw RX bytes via a callback.

SerialTransport  — USB CDC (pyserial), background read thread.
LoopbackTransport — in-memory, for offline tests (no hardware, no Qt).
"""
from __future__ import annotations

import threading
from typing import Callable, Optional

try:
    import serial            # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError:           # pragma: no cover - optional at import time
    serial = None             # type: ignore
    list_ports = None         # type: ignore

RxCallback = Callable[[bytes], None]


class Transport:
    """Abstract byte pipe."""

    def send(self, data: bytes) -> None:
        raise NotImplementedError

    def set_on_rx(self, cb: Optional[RxCallback]) -> None:
        self._on_rx = cb

    def close(self) -> None:
        ...

    @property
    def is_open(self) -> bool:
        return False


class SerialTransport(Transport):
    """USB CDC transport over pyserial with a daemon read loop."""

    def __init__(self, port: str, baud: int = 115200, *, read_timeout: float = 0.05):
        if serial is None:
            raise RuntimeError("pyserial not installed (pip install pyserial)")
        self._on_rx: Optional[RxCallback] = None
        self._ser = serial.Serial(port, baud, timeout=read_timeout)
        self._alive = True
        self._thr = threading.Thread(target=self._rx_loop, name="kia-serial-rx", daemon=True)
        self._thr.start()

    @staticmethod
    def list_available() -> list[tuple[str, str]]:
        """Return [(device, description)] for connected serial ports."""
        if list_ports is None:
            return []
        return [(p.device, p.description or p.device) for p in list_ports.comports()]

    def send(self, data: bytes) -> None:
        if self._alive:
            self._ser.write(data)

    def _rx_loop(self) -> None:
        while self._alive:
            try:
                chunk = self._ser.read(256)
            except Exception:
                break
            if chunk and self._on_rx:
                self._on_rx(bytes(chunk))

    def close(self) -> None:
        self._alive = False
        try:
            self._ser.close()
        except Exception:
            pass

    @property
    def is_open(self) -> bool:
        return self._alive and bool(getattr(self._ser, "is_open", False))


class LoopbackTransport(Transport):
    """In-memory transport for tests. Captures host->device bytes; inject simulates device->host."""

    def __init__(self) -> None:
        self._on_rx: Optional[RxCallback] = None
        self.tx_log = bytearray()
        self._open = True

    def send(self, data: bytes) -> None:
        self.tx_log.extend(data)

    def inject(self, data: bytes) -> None:
        """Simulate bytes arriving from the device."""
        if self._on_rx:
            self._on_rx(bytes(data))

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open
