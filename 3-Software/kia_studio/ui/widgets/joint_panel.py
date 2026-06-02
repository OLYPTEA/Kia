"""JointPanel — per-joint sliders + spinboxes, with live-throttled or on-release sending."""
from __future__ import annotations

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QButtonGroup, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QRadioButton, QSlider, QVBoxLayout, QWidget,
)

from ...core.joints import GRIP, JOINTS

_SCALE = 10  # slider int units per degree (0.1 deg resolution)
_LIVE_MS = 40  # ~25 Hz coalesced live send


class JointRow(QWidget):
    valueChanged = Signal(float)   # live, during edit
    committed = Signal(float)      # on slider release / spinbox commit

    def __init__(self, spec, parent=None):
        super().__init__(parent)
        self.spec = spec
        self._guard = False

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(spec.min_deg * _SCALE), int(spec.max_deg * _SCALE))
        self.slider.setValue(int(spec.home_deg * _SCALE))

        self.spin = QDoubleSpinBox()
        self.spin.setRange(spec.min_deg, spec.max_deg)
        self.spin.setDecimals(1)
        self.spin.setSingleStep(0.5)
        self.spin.setSuffix("°")
        self.spin.setValue(spec.home_deg)
        self.spin.setFixedWidth(84)

        self.slider.valueChanged.connect(self._on_slider)
        self.slider.sliderReleased.connect(lambda: self.committed.emit(self.value()))
        self.spin.valueChanged.connect(self._on_spin)
        self.spin.editingFinished.connect(lambda: self.committed.emit(self.value()))

    def _on_slider(self, v: int) -> None:
        if self._guard:
            return
        deg = v / _SCALE
        self._guard = True
        self.spin.setValue(deg)
        self._guard = False
        self.valueChanged.emit(deg)

    def _on_spin(self, deg: float) -> None:
        if self._guard:
            return
        self._guard = True
        self.slider.setValue(int(deg * _SCALE))
        self._guard = False
        self.valueChanged.emit(deg)

    def value(self) -> float:
        return self.spin.value()

    def set_value_silent(self, deg: float) -> None:
        self._guard = True
        self.spin.setValue(deg)
        self.slider.setValue(int(deg * _SCALE))
        self._guard = False

    @property
    def held(self) -> bool:
        return self.slider.isSliderDown()


class JointPanel(QWidget):
    # emitted with [base, shoulder, elbow, wrist, grip] (deg) and a trajectory duration (s)
    command = Signal(list, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.rows: list[JointRow] = []
        self._editing = False

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        box = QGroupBox("Joint control")
        grid = QGridLayout(box)
        specs = list(JOINTS) + [GRIP]
        for r, spec in enumerate(specs):
            row = JointRow(spec)
            self.rows.append(row)
            grid.addWidget(QLabel(spec.name), r, 0)
            grid.addWidget(row.slider, r, 1)
            grid.addWidget(row.spin, r, 2)
            row.valueChanged.connect(self._on_live)
            row.committed.connect(self._on_committed)
        grid.setColumnStretch(1, 1)
        root.addWidget(box)

        # send-mode + duration controls
        ctl = QGroupBox("Send mode")
        cl = QHBoxLayout(ctl)
        self.rb_live = QRadioButton("Live")
        self.rb_release = QRadioButton("On release")
        self.rb_live.setChecked(True)
        grp = QButtonGroup(self)
        grp.addButton(self.rb_live)
        grp.addButton(self.rb_release)
        cl.addWidget(self.rb_live)
        cl.addWidget(self.rb_release)
        cl.addStretch(1)
        cl.addWidget(QLabel("Duration:"))
        self.dur = QDoubleSpinBox()
        self.dur.setRange(0.0, 10.0)
        self.dur.setDecimals(2)
        self.dur.setSingleStep(0.25)
        self.dur.setValue(1.5)
        self.dur.setSuffix(" s")
        cl.addWidget(self.dur)
        root.addWidget(ctl)
        root.addStretch(1)

        self._live_timer = QTimer(self)
        self._live_timer.setSingleShot(True)
        self._live_timer.setInterval(_LIVE_MS)
        self._live_timer.timeout.connect(self._send_live)

    # ---- user edits ----
    def _on_live(self, _deg: float) -> None:
        self._editing = True
        if self.rb_live.isChecked():
            if not self._live_timer.isActive():
                self._live_timer.start()

    def _on_committed(self, _deg: float) -> None:
        self._editing = False
        if self.rb_release.isChecked():
            self.command.emit(self._values(), self.dur.value())

    def _send_live(self) -> None:
        self.command.emit(self._values(), 0.08)
        self._editing = self._any_held()

    def _values(self) -> list:
        return [row.value() for row in self.rows]

    def current_pose(self) -> list:
        """[base, shoulder, elbow, wrist, grip] — the dialled-in pose (for keyframes)."""
        return self._values()

    def apply_pose(self, pose) -> None:
        """Set the sliders silently from a pose (e.g. timeline preview), no command sent."""
        for row, v in zip(self.rows, pose):
            row.set_value_silent(float(v))

    def _any_held(self) -> bool:
        return any(row.held for row in self.rows)

    # ---- telemetry sync ----
    def update_from_status(self, st) -> None:
        if self._editing or self._any_held():
            return
        vals = list(st.joints) + [st.grip]
        for row, v in zip(self.rows, vals):
            row.set_value_silent(v)
