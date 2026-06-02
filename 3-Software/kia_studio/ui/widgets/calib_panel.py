"""Live mesh-calibration panel — align each STL link against the FK skeleton.

Per-link controls (recenter / scale / rotation XYZ / translation XYZ) drive the viewport
in real time; a test-pose selector and a grip slider check alignment and jaw articulation
across configurations; Save persists to resources/mesh_calib.json. Gripper jaws expose an
extra hinge group (pivot + axis + gain) so they swing about the right point/axis.
"""
from __future__ import annotations

import math
import os
import time

from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout,
                               QGroupBox, QHBoxLayout, QLabel, QPushButton, QSlider,
                               QVBoxLayout, QWidget)
from PySide6.QtCore import Qt, QTimer

from ..mesh_config import GRIP_LINKS, LINK_ORDER, save_calibration
from ...core.kinematics import (geometry_from_pivots, save_geometry,
                                firmware_geometry_header)

_LABELS = {"bati": "Bâti (fixe)", "base": "Base (lacet)", "shoulder": "Épaule",
           "elbow": "Coude", "wrist": "Poignet", "grip1": "Mors 1", "grip2": "Mors 2"}

# which joint index a link's rotation is driven by (for the oscillation preview)
_JOINT_OF = {"base": 0, "shoulder": 1, "elbow": 2, "wrist": 3}
_HOME_POSE = (0.0, 90.0, 0.0, 0.0)   # base, shoulder, elbow, wrist — pivot invariance point
_JOG_AMP = 40.0                      # deg, ± around the joint home value

_TEST_POSES = {
    "Home": (0.0, 90.0, 0.0, 0.0),
    "Reach": (30.0, 60.0, -30.0, 10.0),
    "Folded": (-45.0, 120.0, -60.0, 30.0),
    "Vertical": (0.0, 0.0, 0.0, 0.0),
    "Up": (0.0, 30.0, 20.0, 0.0),
}


class CalibrationPanel(QWidget):
    def __init__(self, viewport, parent=None):
        super().__init__(parent)
        self.viewport = viewport
        self._loading = False
        self._jog_t0 = 0.0
        self._jog_timer = QTimer(self)
        self._jog_timer.setInterval(30)
        self._jog_timer.timeout.connect(self._jog_tick)
        self._build()
        self.viewport.pivotMoved.connect(self._on_pivot_dragged)
        self._select_link(0)

    def _on_pivot_dragged(self, name):
        """Sync the pivot spinboxes while the user drags the 3D gizmo."""
        if name != self._current():
            return
        cfg = self.viewport.calib[name]
        self._loading = True
        self.sb_px.setValue(cfg["px"]); self.sb_py.setValue(cfg["py"]); self.sb_pz.setValue(cfg["pz"])
        self._loading = False

    def _spin(self, lo, hi, step, dec, suffix=""):
        sb = QDoubleSpinBox()
        sb.setRange(lo, hi)
        sb.setSingleStep(step)
        sb.setDecimals(dec)
        if suffix:
            sb.setSuffix(suffix)
        sb.valueChanged.connect(self._apply)
        return sb

    def _build(self):
        root = QVBoxLayout(self)

        top = QHBoxLayout()
        top.addWidget(QLabel("Lien"))
        self.link_combo = QComboBox()
        for n in LINK_ORDER:
            self.link_combo.addItem(_LABELS[n], n)
        self.link_combo.currentIndexChanged.connect(self._select_link)
        top.addWidget(self.link_combo, 1)
        self.solo = QCheckBox("Isoler")
        self.solo.toggled.connect(self._on_solo)
        top.addWidget(self.solo)
        root.addLayout(top)

        box = QGroupBox("Placement du lien")
        form = QFormLayout(box)
        self.cb_recenter = QCheckBox("Recentrer sur le centroïde")
        self.cb_recenter.toggled.connect(self._apply)
        form.addRow(self.cb_recenter)
        self.sb_scale = self._spin(0.0005, 5.0, 0.005, 4, "")
        form.addRow("Échelle", self.sb_scale)
        self.sb_rx = self._spin(-180, 180, 5, 1, " °")
        self.sb_ry = self._spin(-180, 180, 5, 1, " °")
        self.sb_rz = self._spin(-180, 180, 5, 1, " °")
        form.addRow("Rot X", self.sb_rx)
        form.addRow("Rot Y", self.sb_ry)
        form.addRow("Rot Z", self.sb_rz)
        self.sb_tx = self._spin(-500, 500, 1, 2, " mm")
        self.sb_ty = self._spin(-500, 500, 1, 2, " mm")
        self.sb_tz = self._spin(-500, 500, 1, 2, " mm")
        form.addRow("Trans X", self.sb_tx)
        form.addRow("Trans Y", self.sb_ty)
        form.addRow("Trans Z", self.sb_tz)
        root.addWidget(box)

        self.pbox = QGroupBox("Point de pivot — glisser les axes R/V/B en 3D, ou régler ici")
        pbox = self.pbox
        pform = QFormLayout(pbox)
        self.sb_px = self._spin(-300, 300, 1, 2, " mm")
        self.sb_py = self._spin(-300, 300, 1, 2, " mm")
        self.sb_pz = self._spin(-300, 300, 1, 2, " mm")
        pform.addRow("Pivot X", self.sb_px)
        pform.addRow("Pivot Y", self.sb_py)
        pform.addRow("Pivot Z", self.sb_pz)
        root.addWidget(pbox)

        # gripper-jaw hinge (only relevant for grip links)
        self.hinge_box = QGroupBox("Charnière pince (mors)")
        hform = QFormLayout(self.hinge_box)
        self.sb_hx = self._spin(-300, 300, 1, 2, " mm")
        self.sb_hy = self._spin(-300, 300, 1, 2, " mm")
        self.sb_hz = self._spin(-300, 300, 1, 2, " mm")
        hform.addRow("Pivot X", self.sb_hx)
        hform.addRow("Pivot Y", self.sb_hy)
        hform.addRow("Pivot Z", self.sb_hz)
        self.cmb_haxis = QComboBox()
        self.cmb_haxis.addItems(["x", "y", "z"])
        self.cmb_haxis.currentTextChanged.connect(self._apply)
        hform.addRow("Axe", self.cmb_haxis)
        self.sb_hgain = self._spin(-2.0, 2.0, 0.05, 3, " °/u")
        hform.addRow("Gain", self.sb_hgain)
        root.addWidget(self.hinge_box)

        pose_row = QHBoxLayout()
        pose_row.addWidget(QLabel("Pose test"))
        self.pose_combo = QComboBox()
        for name in _TEST_POSES:
            self.pose_combo.addItem(name)
        self.pose_combo.currentTextChanged.connect(self._on_pose)
        pose_row.addWidget(self.pose_combo, 1)
        self.jog_cb = QCheckBox("Oscillation auto")
        self.jog_cb.toggled.connect(self._on_jog)
        pose_row.addWidget(self.jog_cb)
        root.addLayout(pose_row)

        grip_row = QHBoxLayout()
        grip_row.addWidget(QLabel("Grip test"))
        self.grip_slider = QSlider(Qt.Horizontal)
        self.grip_slider.setRange(0, 90)
        self.grip_slider.valueChanged.connect(self._on_grip)
        self.grip_lbl = QLabel("0°")
        grip_row.addWidget(self.grip_slider, 1)
        grip_row.addWidget(self.grip_lbl)
        root.addLayout(grip_row)

        btns = QHBoxLayout()
        self.reset_btn = QPushButton("Réinitialiser ce lien")
        self.reset_btn.clicked.connect(self._reset_link)
        self.save_btn = QPushButton("Sauvegarder")
        self.save_btn.clicked.connect(self._save)
        self.geom_btn = QPushButton("Exporter géométrie")
        self.geom_btn.setToolTip("Déduire h_base/L1/L2/L_tool des pivots → host + snippet firmware")
        self.geom_btn.clicked.connect(self._export_geometry)
        btns.addWidget(self.reset_btn)
        btns.addWidget(self.save_btn)
        btns.addWidget(self.geom_btn)
        root.addLayout(btns)

        self.status = QLabel("")
        self.status.setWordWrap(True)
        root.addWidget(self.status)
        root.addStretch(1)

    # ---- field <-> calib --------------------------------------------
    def _current(self) -> str:
        return self.link_combo.currentData()

    def _select_link(self, _idx):
        name = self._current()
        cfg = self.viewport.calib[name]
        self._loading = True
        self.cb_recenter.setChecked(bool(cfg["recenter"]))
        self.sb_scale.setValue(cfg["scale"])
        self.sb_rx.setValue(cfg["rx"]); self.sb_ry.setValue(cfg["ry"]); self.sb_rz.setValue(cfg["rz"])
        self.sb_tx.setValue(cfg["tx"]); self.sb_ty.setValue(cfg["ty"]); self.sb_tz.setValue(cfg["tz"])
        self.sb_px.setValue(cfg.get("px", 0.0)); self.sb_py.setValue(cfg.get("py", 0.0))
        self.sb_pz.setValue(cfg.get("pz", 0.0))
        self.sb_hx.setValue(cfg.get("hx", 0.0)); self.sb_hy.setValue(cfg.get("hy", 0.0))
        self.sb_hz.setValue(cfg.get("hz", 0.0))
        self.cmb_haxis.setCurrentText(cfg.get("haxis", "x"))
        self.sb_hgain.setValue(cfg.get("hgain", 0.0))
        self._loading = False
        self.hinge_box.setVisible(name in GRIP_LINKS)
        self.pbox.setVisible(name in _JOINT_OF)   # only chain joints have a pivot
        self.viewport.show_pivot_marker(name)
        if self.solo.isChecked():
            self.viewport.highlight_link(name)

    def _apply(self, *_):
        if self._loading:
            return
        self.viewport.set_link_calib(self._current(), dict(
            recenter=self.cb_recenter.isChecked(),
            scale=self.sb_scale.value(),
            rx=self.sb_rx.value(), ry=self.sb_ry.value(), rz=self.sb_rz.value(),
            tx=self.sb_tx.value(), ty=self.sb_ty.value(), tz=self.sb_tz.value(),
            px=self.sb_px.value(), py=self.sb_py.value(), pz=self.sb_pz.value(),
            hx=self.sb_hx.value(), hy=self.sb_hy.value(), hz=self.sb_hz.value(),
            haxis=self.cmb_haxis.currentText(), hgain=self.sb_hgain.value(),
        ))

    def _on_solo(self, on):
        self.viewport.highlight_link(self._current() if on else None)

    def _on_pose(self, name):
        if not self._jog_cb_on():
            self.viewport.set_pose(*_TEST_POSES[name], self.grip_slider.value())

    def _jog_cb_on(self):
        return self.jog_cb.isChecked()

    def _on_jog(self, on):
        if on:
            self._jog_t0 = time.monotonic()
            self._jog_timer.start()
        else:
            self._jog_timer.stop()
            self._on_pose(self.pose_combo.currentText())   # settle back to test pose

    def _jog_tick(self):
        # oscillate the selected joint around its HOME value (the pivot-invariance point),
        # so the part swings about its pivot from the calibrated home placement.
        name = self._current()
        base = list(_HOME_POSE)
        grip = float(self.grip_slider.value())
        phase = math.sin(2 * math.pi * 0.35 * (time.monotonic() - self._jog_t0))
        if name in _JOINT_OF:
            i = _JOINT_OF[name]
            base[i] = base[i] + _JOG_AMP * phase
        elif name in GRIP_LINKS:
            grip = 45.0 + 45.0 * phase     # sweep the jaw 0..90
        self.viewport.set_pose(*base, grip)

    def _on_grip(self, v):
        self.grip_lbl.setText(f"{v}°")
        self.viewport.set_pose(*self.viewport._pose, float(v))

    def _reset_link(self):
        from ..mesh_config import default_calib
        self.viewport.calib[self._current()] = default_calib()[self._current()]
        self._select_link(0)
        self.viewport.set_pose(*self.viewport._pose, self.viewport._grip)

    def _save(self):
        path = save_calibration(self.viewport.calib)
        self.status.setText(f"Sauvegardé → {path}")

    def _export_geometry(self):
        """Derive link lengths from the calibrated pivots; persist host + firmware snippet."""
        pivots = self.viewport._pivots()
        tcp = self.viewport._tcp_home()
        geo = geometry_from_pivots(pivots, tcp)
        save_geometry(geo)
        res_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__)))), "resources")
        fw_path = os.path.join(res_dir, "geometry_firmware.h")
        with open(fw_path, "w", encoding="utf-8") as f:
            f.write(firmware_geometry_header(geo))
        self.status.setText(
            f"Géométrie : h_base={geo.h_base:.1f}  L1={geo.L1:.1f}  L2={geo.L2:.1f}  "
            f"L_tool={geo.L_tool:.1f} mm\nHost → geometry.json · Firmware → geometry_firmware.h "
            "(à coller dans app_config.h)")

    # ---- freeze the view on the home reference while this tab is shown ----
    def showEvent(self, e):
        super().showEvent(e)
        self.viewport.set_calib_frozen(True)
        self.viewport.show_pivot_marker(self._current())

    def hideEvent(self, e):
        super().hideEvent(e)
        self._jog_timer.stop()
        self.jog_cb.setChecked(False)
        self.viewport.set_calib_frozen(False)
        self.viewport.show_pivot_marker(None)
