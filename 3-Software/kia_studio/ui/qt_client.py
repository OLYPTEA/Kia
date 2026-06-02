"""QtKiaClient — Qt adapter over proto.KiaClient.

KiaClient callbacks fire on the transport's RX thread (serial / sim). This adapter
re-emits them as Qt signals, which Qt delivers to the GUI thread via a queued
connection — so widgets are only ever touched from the main thread.
"""
from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QObject, Signal

from ..proto import KiaClient, SerialTransport
from ..proto.opcodes import Mode


class QtKiaClient(QObject):
    telemetry    = Signal(object)   # Status
    status       = Signal(object)   # Status
    ack          = Signal(int)      # seq
    nack         = Signal(int, int)  # seq, reason
    pong         = Signal(object)   # (maj, min, patch)
    text         = Signal(str)
    connected    = Signal(str)      # human label
    disconnected = Signal(str)
    error        = Signal(str)

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)
        self._client: Optional[KiaClient] = None
        self._transport = None
        self._label = ""

    # ---- lifecycle ----
    @property
    def is_connected(self) -> bool:
        return self._client is not None

    @property
    def client(self) -> Optional[KiaClient]:
        return self._client

    def connect_serial(self, port: str, baud: int = 115200) -> None:
        self._open(lambda: SerialTransport(port, baud), f"USB {port} @ {baud}")

    def connect_sim(self) -> None:
        from ..core.sim import SimTransport
        self._open(SimTransport, "Simulator")

    def _open(self, make_transport, label: str) -> None:
        if self._client is not None:
            self.disconnect()
        try:
            self._transport = make_transport()
            self._client = KiaClient(self._transport)
            self._wire(self._client)
            self._label = label
            self.connected.emit(label)
            # kick off a handshake + status request
            self._client.ping()
            self._client.get_status()
        except Exception as exc:  # noqa: BLE001
            self._client = None
            self._transport = None
            self.error.emit(str(exc))

    def disconnect(self) -> None:
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:
                pass
        self._client = None
        self._transport = None
        if self._label:
            self.disconnected.emit(self._label)
        self._label = ""

    def _wire(self, c: KiaClient) -> None:
        c.on_telemetry = self.telemetry.emit
        c.on_status    = self.status.emit
        c.on_ack       = self.ack.emit
        c.on_nack      = lambda seq, reason: self.nack.emit(seq, reason)
        c.on_pong      = self.pong.emit
        c.on_text      = self.text.emit

    # ---- command forwarders (no-op if disconnected) ----
    def ping(self):                       self._do(lambda c: c.ping())
    def get_status(self):                 self._do(lambda c: c.get_status())
    def home(self):                       self._do(lambda c: c.home())
    def hold(self):                       self._do(lambda c: c.hold())
    def arm(self):                        self._do(lambda c: c.arm())
    def clear(self):                      self._do(lambda c: c.clear())
    def save(self):                       self._do(lambda c: c.save())
    def mode_set(self, m: Mode | int):    self._do(lambda c: c.mode_set(m))
    def set_joint(self, i, deg, dur=1.0): self._do(lambda c: c.set_joint(i, deg, dur))
    def set_joints(self, *a, **k):        self._do(lambda c: c.set_joints(*a, **k))
    def set_xyz(self, *a, **k):           self._do(lambda c: c.set_xyz(*a, **k))

    def _do(self, fn):
        if self._client is not None:
            try:
                fn(self._client)
            except Exception as exc:  # noqa: BLE001
                self.error.emit(str(exc))
