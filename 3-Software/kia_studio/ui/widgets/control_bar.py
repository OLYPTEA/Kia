"""ControlBar — mode selector, HOME/ARM/CLEAR/SAVE, and an always-visible SOFT STOP.

SOFT STOP sends HOLD + MODE HOLD: motion freezes and further motion commands are
rejected until re-armed (ARM). The real E-STOP remains the physical button that
gates +VSERVO — this app button cannot cut the rail.
"""
from __future__ import annotations

from PySide6.QtWidgets import (
    QComboBox, QHBoxLayout, QLabel, QPushButton, QWidget,
)

from ...proto.opcodes import Mode
from ..qt_client import QtKiaClient


class ControlBar(QWidget):
    def __init__(self, client: QtKiaClient, parent=None):
        super().__init__(parent)
        self._client = client
        self._guard = False

        self.mode = QComboBox()
        for m in (Mode.MANUAL, Mode.APP, Mode.HOLD):
            self.mode.addItem(m.label, userData=int(m))
        self.mode.setCurrentIndex(1)  # APP

        self.home_btn = QPushButton("Home")
        self.arm_btn = QPushButton("Arm")
        self.clear_btn = QPushButton("Clear")
        self.save_btn = QPushButton("Save")

        self.stop_btn = QPushButton("◼  SOFT STOP")
        self.stop_btn.setObjectName("stopBtn")
        self.stop_btn.setToolTip("HOLD + MODE HOLD. Real E-STOP is the physical button.")
        self.stop_btn.setMinimumHeight(34)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.addWidget(QLabel("Mode:"))
        lay.addWidget(self.mode)
        lay.addWidget(self.home_btn)
        lay.addWidget(self.arm_btn)
        lay.addWidget(self.clear_btn)
        lay.addWidget(self.save_btn)
        lay.addStretch(1)
        lay.addWidget(self.stop_btn)

        self.mode.currentIndexChanged.connect(self._on_mode)
        self.home_btn.clicked.connect(client.home)
        self.arm_btn.clicked.connect(self._on_arm)
        self.clear_btn.clicked.connect(client.clear)
        self.save_btn.clicked.connect(client.save)
        self.stop_btn.clicked.connect(self._on_stop)

    def _on_mode(self, _i: int) -> None:
        if self._guard:
            return
        self._client.mode_set(self.mode.currentData())

    def _on_arm(self) -> None:
        self._client.arm()
        self._set_mode_silent(Mode.APP)

    def _on_stop(self) -> None:
        self._client.hold()
        self._client.mode_set(Mode.HOLD)
        self._set_mode_silent(Mode.HOLD)

    def _set_mode_silent(self, m: Mode) -> None:
        self._guard = True
        idx = self.mode.findData(int(m))
        if idx >= 0:
            self.mode.setCurrentIndex(idx)
        self._guard = False
