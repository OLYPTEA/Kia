"""Keyframe animation — pose sequences with time interpolation (no Qt, no hardware).

A pose is 5 joint angles in degrees: (base, shoulder, elbow, wrist, grip). A Sequence is
an ordered set of timed keyframes; `sample(t)` interpolates between them (linear, or a
cosine ease for smoother motion). Sequences serialize to JSON for save/load.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field

POSE_LEN = 5


def _lerp(a: float, b: float, u: float) -> float:
    return a + (b - a) * u


@dataclass(slots=True)
class Keyframe:
    t: float                       # seconds from sequence start (when the pose is reached)
    pose: tuple[float, float, float, float, float]   # base, shoulder, elbow, wrist, grip
    dwell: float = 0.0             # seconds to hold (stop) on this pose before resuming

    def as_dict(self) -> dict:
        return {"t": self.t, "pose": list(self.pose), "dwell": self.dwell}

    @staticmethod
    def from_dict(d: dict) -> "Keyframe":
        p = list(d["pose"])
        return Keyframe(float(d["t"]), tuple(float(x) for x in p[:POSE_LEN]),
                        float(d.get("dwell", 0.0)))


@dataclass(slots=True)
class Sequence:
    name: str = "sequence"
    keyframes: list[Keyframe] = field(default_factory=list)

    # ---- editing ----
    def add(self, t: float, pose) -> Keyframe:
        kf = Keyframe(float(t), tuple(float(x) for x in pose[:POSE_LEN]))
        self.keyframes.append(kf)
        self.sort()
        return kf

    def remove(self, index: int) -> None:
        if 0 <= index < len(self.keyframes):
            self.keyframes.pop(index)

    def set_time(self, index: int, t: float) -> None:
        if 0 <= index < len(self.keyframes):
            self.keyframes[index].t = max(0.0, float(t))
            self.sort()

    def set_dwell(self, index: int, dwell: float) -> None:
        if 0 <= index < len(self.keyframes):
            self.keyframes[index].dwell = max(0.0, float(dwell))

    def sort(self) -> None:
        self.keyframes.sort(key=lambda k: k.t)

    def clear(self) -> None:
        self.keyframes.clear()

    @property
    def duration(self) -> float:
        return self.keyframes[-1].t if self.keyframes else 0.0

    # ---- sampling ----
    def sample(self, t: float, smooth: bool = False) -> list[float] | None:
        """Interpolated pose at time `t` (clamped to the sequence span). None if empty."""
        n = len(self.keyframes)
        if n == 0:
            return None
        if n == 1 or t <= self.keyframes[0].t:
            return list(self.keyframes[0].pose)
        if t >= self.keyframes[-1].t:
            return list(self.keyframes[-1].pose)
        # find the bracketing pair
        for i in range(n - 1):
            k0, k1 = self.keyframes[i], self.keyframes[i + 1]
            if k0.t <= t <= k1.t:
                span = k1.t - k0.t
                u = 0.0 if span <= 1e-9 else (t - k0.t) / span
                if smooth:
                    u = (1 - math.cos(math.pi * u)) / 2.0
                return [_lerp(a, b, u) for a, b in zip(k0.pose, k1.pose)]
        return list(self.keyframes[-1].pose)

    # ---- serialization ----
    def to_dict(self) -> dict:
        return {"name": self.name, "version": 1,
                "keyframes": [k.as_dict() for k in self.keyframes]}

    @staticmethod
    def from_dict(d: dict) -> "Sequence":
        seq = Sequence(name=d.get("name", "sequence"))
        seq.keyframes = [Keyframe.from_dict(k) for k in d.get("keyframes", [])]
        seq.sort()
        return seq

    def save(self, path: str) -> str:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)
        return path

    @staticmethod
    def load(path: str) -> "Sequence":
        with open(path, "r", encoding="utf-8") as f:
            return Sequence.from_dict(json.load(f))
