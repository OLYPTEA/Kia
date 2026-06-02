"""Timeline / keyframe editor — compose, preview and save predefined arm motions.

Capture the current pose as timed keyframes, scrub/play the interpolated motion (linear
or eased) to preview it in the 3D view, save/load sequences as JSON, and stream a
sequence to the device/simulator.
"""
from __future__ import annotations

import os

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QDoubleSpinBox, QFileDialog,
                               QHBoxLayout, QHeaderView, QLabel, QPushButton, QSlider,
                               QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from ...core.animation import Sequence
from ...core.trajectory import build_trajectory

_SEQ_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
                        "resources", "sequences")
_COLS = ("t (s)", "Délai (s)", "Base", "Épaule", "Coude", "Poignet", "Grip")
_TICK_MS = 33
_CAPTURE_GAP = 1.0   # seconds added after the last keyframe on capture


class TimelinePanel(QWidget):
    poseChanged = Signal(list)        # [base, shoulder, elbow, wrist, grip] while scrub/play
    playbackActive = Signal(bool)     # True while playing (freeze telemetry / preview)

    def __init__(self, pose_provider=None, parent=None):
        super().__init__(parent)
        self.seq = Sequence()
        self.traj = build_trajectory(self.seq)   # smooth spline trajectory (rebuilt on edit)
        self._pose_provider = pose_provider      # callable -> current pose [5]
        self._t = 0.0
        self._playing = False
        self._build()
        self._refresh_table()

    def set_pose_provider(self, fn):
        self._pose_provider = fn

    # ---- UI ----
    def _build(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 6, 8, 6)
        root.setSpacing(6)

        transport = QHBoxLayout()
        self.btn_start = QPushButton("⏮")
        self.btn_play = QPushButton("▶")
        self.btn_stop = QPushButton("⏹")
        for b in (self.btn_start, self.btn_play, self.btn_stop):
            b.setFixedWidth(40)
        self.btn_start.clicked.connect(lambda: self._seek(0.0))
        self.btn_play.clicked.connect(self._toggle_play)
        self.btn_stop.clicked.connect(self._stop)
        self.chk_loop = QCheckBox("Boucle")
        self.chk_smooth = QCheckBox("Trajectoire lisse")
        self.chk_smooth.setChecked(True)
        self.chk_smooth.setToolTip("Spline C² + limites vitesse/accélération (sinon linéaire)")
        self.chk_smooth.toggled.connect(self._on_smooth_toggled)
        self.chk_robot = QCheckBox("Piloter le robot")
        self.chk_robot.setToolTip("Pendant la lecture, streamer la trajectoire au robot/simulateur connecté")
        transport.addWidget(self.btn_start)
        transport.addWidget(self.btn_play)
        transport.addWidget(self.btn_stop)
        transport.addWidget(self.chk_loop)
        transport.addWidget(self.chk_smooth)
        transport.addWidget(self.chk_robot)

        self.scrub = QSlider(Qt.Horizontal)
        self.scrub.setRange(0, 1000)
        self.scrub.valueChanged.connect(self._on_scrub)
        transport.addWidget(self.scrub, 1)

        self.time_lbl = QLabel("0.00 / 0.00 s")
        self.time_lbl.setMinimumWidth(110)
        transport.addWidget(self.time_lbl)
        transport.addWidget(QLabel("Vitesse"))
        self.speed = QDoubleSpinBox()
        self.speed.setRange(0.1, 4.0)
        self.speed.setSingleStep(0.1)
        self.speed.setValue(1.0)
        self.speed.setSuffix("×")
        transport.addWidget(self.speed)
        root.addLayout(transport)

        edit = QHBoxLayout()
        self.btn_capture = QPushButton("＋ Keyframe")
        self.btn_update = QPushButton("⟳ Maj. pose")
        self.btn_delete = QPushButton("🗑 Suppr.")
        self.btn_capture.clicked.connect(self._capture)
        self.btn_update.clicked.connect(self._update_selected)
        self.btn_delete.clicked.connect(self._delete_selected)
        edit.addWidget(self.btn_capture)
        edit.addWidget(self.btn_update)
        edit.addWidget(self.btn_delete)
        edit.addStretch(1)
        self.btn_new = QPushButton("Nouveau")
        self.btn_load = QPushButton("Charger…")
        self.btn_save = QPushButton("Sauvegarder…")
        self.btn_new.clicked.connect(self._new)
        self.btn_load.clicked.connect(self._load)
        self.btn_save.clicked.connect(self._save)
        for b in (self.btn_new, self.btn_load, self.btn_save):
            edit.addWidget(b)
        root.addLayout(edit)

        self.table = QTableWidget(0, len(_COLS))
        self.table.setHorizontalHeaderLabels(_COLS)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.SelectedClicked)
        self.table.setMaximumHeight(150)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.itemChanged.connect(self._on_item_changed)
        self.table.itemSelectionChanged.connect(self._on_row_selected)
        root.addWidget(self.table)

        self._timer = QTimer(self)
        self._timer.setInterval(_TICK_MS)
        self._timer.timeout.connect(self._tick)

    # ---- trajectory / sampling ----
    def _rebuild(self):
        self.traj = build_trajectory(self.seq, smooth=self.chk_smooth.isChecked())

    def _motion_duration(self) -> float:
        return self.traj.duration

    def _sample(self, t: float):
        return self.traj.sample(t)

    def is_playing(self) -> bool:
        return self._playing

    def robot_enabled(self) -> bool:
        return self.chk_robot.isChecked()

    def _on_smooth_toggled(self, _on):
        self._rebuild()
        self._refresh_duration()
        self._seek(min(self._t, self._motion_duration()))

    # ---- table <-> sequence ----
    def _refresh_table(self):
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.seq.keyframes))
        for r, kf in enumerate(self.seq.keyframes):
            vals = [f"{kf.t:.2f}", f"{kf.dwell:.2f}", *[f"{p:.1f}" for p in kf.pose]]
            for c, v in enumerate(vals):
                it = QTableWidgetItem(v)
                if c > 1:
                    it.setFlags(it.flags() & ~Qt.ItemIsEditable)   # only time + dwell editable
                self.table.setItem(r, c, it)
        self.table.blockSignals(False)
        self._rebuild()
        self._refresh_duration()

    def _refresh_duration(self):
        self.scrub.setEnabled(bool(self._motion_duration() > 0))
        self._update_time_label()

    def _on_item_changed(self, item):
        col = item.column()
        if col > 1:
            return
        try:
            val = float(item.text().replace(",", "."))
        except ValueError:
            self._refresh_table()
            return
        if col == 0:
            self.seq.set_time(item.row(), val)
        else:
            self.seq.set_dwell(item.row(), val)
        self._refresh_table()

    def _on_row_selected(self):
        rows = self.table.selectionModel().selectedRows()
        if rows:
            self._seek(self.seq.keyframes[rows[0].row()].t)

    # ---- editing ----
    def _current_pose(self):
        if self._pose_provider is None:
            return [0.0, 90.0, 0.0, 0.0, 0.0]
        return list(self._pose_provider())

    def _capture(self):
        t = (self.seq.duration + _CAPTURE_GAP) if self.seq.keyframes else 0.0
        self.seq.add(t, self._current_pose())
        self._refresh_table()

    def _update_selected(self):
        rows = self.table.selectionModel().selectedRows()
        if not rows:
            return
        self.seq.keyframes[rows[0].row()].pose = tuple(self._current_pose())
        self._refresh_table()

    def _delete_selected(self):
        rows = self.table.selectionModel().selectedRows()
        if rows:
            self.seq.remove(rows[0].row())
            self._refresh_table()

    # ---- playback ----
    def _toggle_play(self):
        if self._playing:
            self._set_playing(False)
        elif self._motion_duration() > 0:
            if self._t >= self._motion_duration():
                self._t = 0.0
            self._set_playing(True)

    def _set_playing(self, on: bool):
        self._playing = on
        self.btn_play.setText("⏸" if on else "▶")
        self.playbackActive.emit(on)
        if on:
            self._timer.start()
        else:
            self._timer.stop()

    def _stop(self):
        self._set_playing(False)
        self._seek(0.0)

    def _tick(self):
        dur = self._motion_duration()
        self._t += (_TICK_MS / 1000.0) * self.speed.value()
        if self._t >= dur:
            if self.chk_loop.isChecked():
                self._t = 0.0
            else:
                self._t = dur
                self._set_playing(False)
        self._apply_time(self._t)

    def _seek(self, t: float):
        self._t = max(0.0, min(t, self._motion_duration()))
        self._apply_time(self._t)

    def _apply_time(self, t: float):
        pose = self._sample(t)
        dur = self._motion_duration()
        self.scrub.blockSignals(True)
        self.scrub.setValue(int(1000 * (t / dur)) if dur > 0 else 0)
        self.scrub.blockSignals(False)
        self._update_time_label()
        if pose is not None:
            self.poseChanged.emit(pose)

    def _on_scrub(self, value):
        dur = self._motion_duration()
        if dur > 0:
            self._seek(dur * value / 1000.0)

    def _update_time_label(self):
        self.time_lbl.setText(f"{self._t:.2f} / {self._motion_duration():.2f} s")

    # ---- persistence ----
    def _new(self):
        self._stop()
        self.seq = Sequence()
        self._refresh_table()

    def _save(self):
        os.makedirs(_SEQ_DIR, exist_ok=True)
        path, _ = QFileDialog.getSaveFileName(self, "Sauvegarder la séquence",
                                              os.path.join(_SEQ_DIR, "mouvement.json"),
                                              "Séquences (*.json)")
        if path:
            self.seq.name = os.path.splitext(os.path.basename(path))[0]
            self.seq.save(path)

    def _load(self):
        os.makedirs(_SEQ_DIR, exist_ok=True)
        path, _ = QFileDialog.getOpenFileName(self, "Charger une séquence", _SEQ_DIR,
                                              "Séquences (*.json)")
        if path:
            self._stop()
            self.seq = Sequence.load(path)
            self._refresh_table()
