"""Per-link mesh placement calibration — editable live, persisted to JSON.

The transform maps mesh-local CAD vertices into the link frame, applied BEFORE the FK
frame:

    v_link = T(t) @ Rz(rz) @ Ry(ry) @ Rx(rx) @ Scale(scale) @ T(-center)

`center` is the mesh centroid when ``recenter`` is set (default), else the origin.
Chain order: base, shoulder, elbow, wrist, grip (grip rides the wrist frame).

Defaults are a neutral first pass (recenter + uniform scale); the live calibration
panel writes refined values to ``resources/mesh_calib.json``, which overrides defaults.
"""
from __future__ import annotations

import json
import math
import os

import numpy as np

from ..core.kinematics import DEFAULT_PIVOTS

# STL units are ~1000-1800 while the arm is ~290 mm reach: ~0.12 is a sane start.
_DEFAULT_SCALE = 0.12

# calibration schema version. v2 stores joint pivots as WORLD-home points (pivot-driven
# chain). Older files stored link-local pivot offsets, which we ignore on load.
_SCHEMA_VERSION = 2

# Chain order. 'bati' is the fixed housing (world-anchored, never moves); 'base' is the
# yaw yoke that rotates about +Z with q0 and carries the shoulder; 'grip1'/'grip2' are the
# two gripper jaws (ride the wrist frame and hinge open/closed with the grip angle).
LINK_ORDER = ("bati", "base", "shoulder", "elbow", "wrist", "grip1", "grip2")

# Links that hinge with the grip angle (their hinge gain != 0 articulates the jaw).
GRIP_LINKS = ("grip1", "grip2")

# Immutable per-link metadata (file + render color), never persisted.
LINK_META: dict[str, dict] = {
    "bati":     dict(file="Bati.stl",    color=(0.38, 0.40, 0.46, 1.0)),
    "base":     dict(file="Base1.stl",   color=(0.48, 0.52, 0.62, 1.0)),
    "shoulder": dict(file="Epaule.stl",  color=(0.55, 0.62, 0.78, 1.0)),
    "elbow":    dict(file="Coude.stl",   color=(0.62, 0.68, 0.85, 1.0)),
    "wrist":    dict(file="Poignet.stl", color=(0.70, 0.74, 0.90, 1.0)),
    "grip1":    dict(file="Grip1.stl",   color=(0.80, 0.82, 0.95, 1.0)),
    "grip2":    dict(file="Grip2.stl",   color=(0.78, 0.86, 0.92, 1.0)),
}

# Editable calibration fields with defaults.
#  - p* : joint PIVOT point (mm, WORLD-home) — the rotation centre of the joint; the part
#         and everything downstream swing about it. Placed via the 3D gizmo.
#  - h* : gripper-jaw hinge — about axis `haxis` through (hx,hy,hz), by hgain*grip deg.
_CALIB_FIELDS = ("recenter", "scale", "rx", "ry", "rz", "tx", "ty", "tz",
                 "px", "py", "pz", "hx", "hy", "hz", "haxis", "hgain")
_PIVOT_FIELDS = ("px", "py", "pz")

_AXES = ("x", "y", "z")


def default_calib() -> dict[str, dict]:
    out = {}
    for n in LINK_ORDER:
        px, py, pz = DEFAULT_PIVOTS.get(n, (0.0, 0.0, 0.0))
        out[n] = dict(recenter=True, scale=_DEFAULT_SCALE,
                      rx=0.0, ry=0.0, rz=0.0, tx=0.0, ty=0.0, tz=0.0,
                      px=px, py=py, pz=pz,
                      hx=0.0, hy=0.0, hz=0.0, haxis="x", hgain=0.0)
    return out


_CALIB_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                           "resources", "mesh_calib.json")


def load_calibration() -> dict[str, dict]:
    """Return the calibration table, overlaying the JSON file over defaults."""
    calib = default_calib()
    if os.path.exists(_CALIB_PATH):
        try:
            with open(_CALIB_PATH, "r", encoding="utf-8") as f:
                saved = json.load(f)
            # pre-v2 files store pivots in the obsolete link-local sense -> keep defaults
            fields = _CALIB_FIELDS
            if saved.get("_version", 1) < _SCHEMA_VERSION:
                fields = tuple(f for f in _CALIB_FIELDS if f not in _PIVOT_FIELDS)
            for n in LINK_ORDER:
                if n in saved:
                    calib[n].update({k: saved[n][k] for k in fields if k in saved[n]})
            if "grip" in saved:  # migrate a legacy single 'grip' calibration onto both jaws
                for jaw in GRIP_LINKS:
                    if jaw not in saved:
                        calib[jaw].update({k: saved["grip"][k]
                                           for k in fields if k in saved["grip"]})
        except (OSError, ValueError, KeyError):
            pass
    return calib


def save_calibration(calib: dict[str, dict]) -> str:
    os.makedirs(os.path.dirname(_CALIB_PATH), exist_ok=True)
    data = {"_version": _SCHEMA_VERSION}
    data.update({n: {k: calib[n][k] for k in _CALIB_FIELDS} for n in LINK_ORDER})
    with open(_CALIB_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    return _CALIB_PATH


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]], float)


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]], float)


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)


def _trans(p):
    m = np.eye(4)
    m[:3, 3] = p
    return m


def _scale(s):
    m = np.eye(4)
    m[0, 0] = m[1, 1] = m[2, 2] = s
    return m


def calib_matrix(center: np.ndarray, cfg: dict) -> np.ndarray:
    """Build the 4x4 mesh-local -> link-frame STATIC placement for one link.

        v_link = T(t) @ R @ Scale(s) @ T(-center)

    This positions/orients the mesh at the home pose. Joint motion (and the rotation
    centre) is handled separately by the pivot-driven chain
    (``core.kinematics.chain_transforms``), so placement and pivot are independent.
    """
    s = float(cfg.get("scale", _DEFAULT_SCALE))
    c = np.asarray(center, float) if cfg.get("recenter", True) else np.zeros(3)
    return (
        _trans((cfg.get("tx", 0.0), cfg.get("ty", 0.0), cfg.get("tz", 0.0)))
        @ _rz(math.radians(cfg.get("rz", 0.0)))
        @ _ry(math.radians(cfg.get("ry", 0.0)))
        @ _rx(math.radians(cfg.get("rx", 0.0)))
        @ _scale(s)
        @ _trans(-c)
    )


def hinge_matrix(cfg: dict, grip_deg: float) -> np.ndarray:
    """Jaw articulation: rotate about `haxis` through pivot (hx,hy,hz), by hgain*grip deg.

    Expressed in the link (wrist) frame, applied left of the static calibration so the
    already-placed jaw swings about its physical hinge. Identity when hgain == 0.
    """
    gain = float(cfg.get("hgain", 0.0))
    if gain == 0.0:
        return np.eye(4)
    theta = math.radians(gain * float(grip_deg))
    rot = {"x": _rx, "y": _ry, "z": _rz}.get(cfg.get("haxis", "x"), _rx)(theta)
    p = (cfg.get("hx", 0.0), cfg.get("hy", 0.0), cfg.get("hz", 0.0))
    return _trans(p) @ rot @ _trans([-v for v in p])
