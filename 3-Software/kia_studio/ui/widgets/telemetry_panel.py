"""TelemetryPanel — live readout of the decoded Status (joints, pose, current, fault)."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget,
)

from ...core.joints import GRIP, JOINTS
from ...proto.codec import Status
from ...proto.opcodes import Fault


class _Field(QLabel):
    def __init__(self, text="—"):
        super().__init__(text)
        self.setObjectName("tlmValue")
        self.setAlignment(Qt.AlignRight | Qt.AlignVCenter)


class TelemetryPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._joint_vals: list[_Field] = []
        self._stale = True

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # joints
        jbox = QGroupBox("Joints (deg)")
        jgrid = QGridLayout(jbox)
        names = [j.name for j in JOINTS] + [GRIP.name]
        for row, name in enumerate(names):
            jgrid.addWidget(QLabel(name), row, 0)
            v = _Field()
            self._joint_vals.append(v)
            jgrid.addWidget(v, row, 1)
        root.addWidget(jbox)

        # pose
        pbox = QGroupBox("TCP pose")
        pgrid = QGridLayout(pbox)
        self.f_x, self.f_y, self.f_z, self.f_pitch = _Field(), _Field(), _Field(), _Field()
        for col, (lbl, fld) in enumerate(
            [("X (mm)", self.f_x), ("Y (mm)", self.f_y),
             ("Z (mm)", self.f_z), ("Pitch (°)", self.f_pitch)]
        ):
            pgrid.addWidget(QLabel(lbl), col, 0)
            pgrid.addWidget(fld, col, 1)
        root.addWidget(pbox)

        # system
        sbox = QGroupBox("System")
        sgrid = QGridLayout(sbox)
        self.f_current = _Field()
        self.f_fault = _Field()
        self.f_src = _Field()
        self.f_idle = _Field()
        for row, (lbl, fld) in enumerate(
            [("Current (mA)", self.f_current), ("Fault", self.f_fault),
             ("Source", self.f_src), ("Idle", self.f_idle)]
        ):
            sgrid.addWidget(QLabel(lbl), row, 0)
            sgrid.addWidget(fld, row, 1)
        root.addWidget(sbox)
        root.addStretch(1)

    def update_status(self, st: Status) -> None:
        vals = list(st.joints) + [st.grip]
        for fld, v in zip(self._joint_vals, vals):
            fld.setText(f"{v:+7.2f}")
        self.f_x.setText(f"{st.xyz[0]:+7.1f}")
        self.f_y.setText(f"{st.xyz[1]:+7.1f}")
        self.f_z.setText(f"{st.xyz[2]:+7.1f}")
        self.f_pitch.setText(f"{st.pitch:+7.1f}")
        self.f_current.setText(f"{st.current_ma}")
        self.f_idle.setText("yes" if st.idle else "no")
        self.f_src.setText(st.src_enum.label)

        fault = st.fault_enum
        self.f_fault.setText(fault.label)
        self.f_fault.setProperty("fault", fault is not Fault.NONE)
        self.f_fault.style().unpolish(self.f_fault)
        self.f_fault.style().polish(self.f_fault)
