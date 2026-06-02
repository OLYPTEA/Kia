"""ConnectionBar — port selection + baud + connect/disconnect, plus a Simulator option."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox, QHBoxLayout, QLabel, QPushButton, QWidget,
)

from ...proto import SerialTransport
from ..qt_client import QtKiaClient

_BAUDS = ["115200", "230400", "460800", "921600"]


class ConnectionBar(QWidget):
    def __init__(self, client: QtKiaClient, parent=None):
        super().__init__(parent)
        self._client = client

        self.port = QComboBox()
        self.port.setMinimumWidth(220)
        self.refresh_btn = QPushButton("↻")          # refresh glyph
        self.refresh_btn.setFixedWidth(32)
        self.refresh_btn.setToolTip("Refresh serial ports")

        self.baud = QComboBox()
        self.baud.addItems(_BAUDS)
        self.baud.setCurrentText("115200")

        self.sim_btn = QPushButton("Simulator")
        self.sim_btn.setCheckable(False)
        self.sim_btn.setToolTip("Connect to the in-process device emulator (no hardware)")

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("connectBtn")

        self.state = QLabel("Disconnected")
        self.state.setObjectName("connState")

        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 6, 8, 6)
        lay.addWidget(QLabel("Port:"))
        lay.addWidget(self.port)
        lay.addWidget(self.refresh_btn)
        lay.addWidget(QLabel("Baud:"))
        lay.addWidget(self.baud)
        lay.addStretch(1)
        lay.addWidget(self.state)
        lay.addWidget(self.sim_btn)
        lay.addWidget(self.connect_btn)

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        self.sim_btn.clicked.connect(self._on_sim_clicked)

        client.connected.connect(self._on_connected)
        client.disconnected.connect(self._on_disconnected)
        client.error.connect(self._on_error)

        self.refresh_ports()

    # ---- ports ----
    def refresh_ports(self) -> None:
        self.port.clear()
        ports = SerialTransport.list_available()
        if not ports:
            self.port.addItem("(no serial ports)", userData=None)
            self.port.setEnabled(False)
        else:
            self.port.setEnabled(True)
            for dev, desc in ports:
                self.port.addItem(f"{dev} — {desc}", userData=dev)

    # ---- actions ----
    def _on_connect_clicked(self) -> None:
        if self._client.is_connected:
            self._client.disconnect()
            return
        dev = self.port.currentData()
        if not dev:
            self.state.setText("No port selected")
            return
        self._client.connect_serial(dev, int(self.baud.currentText()))

    def _on_sim_clicked(self) -> None:
        if self._client.is_connected:
            self._client.disconnect()
        self._client.connect_sim()

    # ---- client feedback ----
    def _on_connected(self, label: str) -> None:
        self.state.setText(label)
        self.state.setProperty("ok", True)
        self.connect_btn.setText("Disconnect")
        self._set_controls_enabled(False)
        self._restyle(self.state)

    def _on_disconnected(self, _label: str) -> None:
        self.state.setText("Disconnected")
        self.state.setProperty("ok", False)
        self.connect_btn.setText("Connect")
        self._set_controls_enabled(True)
        self._restyle(self.state)

    def _on_error(self, msg: str) -> None:
        self.state.setText(f"Error: {msg}")
        self.state.setProperty("ok", False)
        self._restyle(self.state)

    def _set_controls_enabled(self, enabled: bool) -> None:
        self.port.setEnabled(enabled)
        self.baud.setEnabled(enabled)
        self.refresh_btn.setEnabled(enabled)
        self.sim_btn.setEnabled(enabled)

    @staticmethod
    def _restyle(w: QWidget) -> None:
        w.style().unpolish(w)
        w.style().polish(w)
