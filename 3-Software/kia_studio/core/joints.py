"""Joint specifications — mirror of firmware main/app_config.h (limits, order, names)."""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class JointId(IntEnum):
    BASE     = 0
    SHOULDER = 1
    ELBOW    = 2
    WRIST    = 3
    GRIP     = 4


@dataclass(frozen=True, slots=True)
class JointSpec:
    id: JointId
    name: str
    min_deg: float
    max_deg: float
    home_deg: float = 0.0

    @property
    def span(self) -> float:
        return self.max_deg - self.min_deg

    def clamp(self, deg: float) -> float:
        return max(self.min_deg, min(self.max_deg, deg))


# Software joint limits (real arm). NOTE: diverges from firmware app_config.h KIA_LIM_*
# until the firmware sync task is done.
JOINTS: tuple[JointSpec, ...] = (
    JointSpec(JointId.BASE,     "Base",     -90.0,  90.0,  0.0),
    JointSpec(JointId.SHOULDER, "Shoulder", -90.0,  90.0,  90.0),
    JointSpec(JointId.ELBOW,    "Elbow",      0.0, 180.0,  0.0),
    JointSpec(JointId.WRIST,    "Wrist",    -90.0,  90.0,  0.0),
)

GRIP = JointSpec(JointId.GRIP, "Grip", 0.0, 90.0, 0.0)
