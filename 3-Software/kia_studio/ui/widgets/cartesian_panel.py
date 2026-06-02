"""CartesianPanel — X/Y/Z/pitch target with live host-IK reachability feedback."""
from __future__ import annotations

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QPushButton, QVBoxLayout, QWidget,
)

from ...core.kinematics import IkStatus, ik, load_geometry

_LIVE_MS = 60


class CartesianPanel(QWidget):
    # emitted with [x, y, z, pitch] and a trajectory duration (s)
    command = Signal(list, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._geo = load_geometry()
        reach = self._geo.L1 + self._geo.L2 + self._geo.L_tool

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        box = QGroupBox("Cartesian target (TCP)")
        grid = QGridLayout(box)

        def mk(rng, val, step, suffix):
            sb = QDoubleSpinBox()
            sb.setRange(-rng, rng)
            sb.setDecimals(1)
            sb.setSingleStep(step)
            sb.setSuffix(suffix)
            sb.setValue(val)
            return sb

        self.sb_x = mk(reach, 150.0, 5.0, " mm")
        self.sb_y = mk(reach, 0.0, 5.0, " mm")
        self.sb_z = mk(self._geo.h_base + reach, 150.0, 5.0, " mm")
        self.sb_z.setRange(0.0, self._geo.h_base + reach)
        self.sb_pitch = mk(180.0, 0.0, 1.0, "°")

        for r, (lbl, sb) in enumerate(
            [("X", self.sb_x), ("Y", self.sb_y), ("Z", self.sb_z), ("Pitch", self.sb_pitch)]
        ):
            grid.addWidget(QLabel(lbl), r, 0)
            grid.addWidget(sb, r, 1)
            sb.valueChanged.connect(self._on_change)
        root.addWidget(box)

        # config + reachability
        cfg = QGroupBox("Solver")
        cl = QGridLayout(cfg)
        self.elbow = QComboBox()
        self.elbow.addItems(["Elbow up", "Elbow down"])
        cl.addWidget(QLabel("Config"), 0, 0)
        cl.addWidget(self.elbow, 0, 1)
        self.reach_lbl = QLabel("—")
        self.reach_lbl.setObjectName("reachState")
        cl.addWidget(QLabel("Reachable"), 1, 0)
        cl.addWidget(self.reach_lbl, 1, 1)
        self.preview = QLabel("—")
        self.preview.setObjectName("tlmValue")
        self.preview.setWordWrap(True)
        cl.addWidget(QLabel("IK joints"), 2, 0)
        cl.addWidget(self.preview, 2, 1)
        self.elbow.currentIndexChanged.connect(self._on_change)
        root.addWidget(cfg)

        # send controls
        send = QHBoxLayout()
        self.live = QCheckBox("Live")
        send.addWidget(self.live)
        send.addStretch(1)
        send.addWidget(QLabel("Duration:"))
        self.dur = QDoubleSpinBox()
        self.dur.setRange(0.0, 10.0)
        self.dur.setDecimals(2)
        self.dur.setSingleStep(0.25)
        self.dur.setValue(2.0)
        self.dur.setSuffix(" s")
        send.addWidget(self.dur)
        self.apply_btn = QPushButton("Apply")
        self.apply_btn.clicked.connect(self._send_now)
        send.addWidget(self.apply_btn)
        root.addLayout(send)
        root.addStretch(1)

        self._live_timer = QTimer(self)
        self._live_timer.setSingleShot(True)
        self._live_timer.setInterval(_LIVE_MS)
        self._live_timer.timeout.connect(self._send_now)

        self._refresh_reach()

    # ---- reachability ----
    def _pose(self) -> list:
        return [self.sb_x.value(), self.sb_y.value(), self.sb_z.value(), self.sb_pitch.value()]

    def _refresh_reach(self) -> None:
        x, y, z, pitch = self._pose()
        res = ik(x, y, z, pitch, elbow_up=self.elbow.currentIndex() == 0, geo=self._geo)
        ok = res.status is IkStatus.OK
        text = {IkStatus.OK: "yes", IkStatus.UNREACHABLE: "out of range",
                IkStatus.LIMIT: "joint limit"}[res.status]
        self.reach_lbl.setText(text)
        self.reach_lbl.setProperty("ok", ok)
        self.reach_lbl.setProperty("bad", not ok)
        self.reach_lbl.style().unpolish(self.reach_lbl)
        self.reach_lbl.style().polish(self.reach_lbl)
        self.apply_btn.setEnabled(ok)
        if res.status is not IkStatus.UNREACHABLE:
            b, s, e, w = res.joints()
            self.preview.setText(f"{b:+.1f}  {s:+.1f}  {e:+.1f}  {w:+.1f}")
        else:
            self.preview.setText("—")

    # ---- edits ----
    def _on_change(self, *_a) -> None:
        self._refresh_reach()
        if self.live.isChecked() and self.apply_btn.isEnabled():
            if not self._live_timer.isActive():
                self._live_timer.start()

    def _send_now(self) -> None:
        if self.apply_btn.isEnabled():
            self.command.emit(self._pose(), self.dur.value())
