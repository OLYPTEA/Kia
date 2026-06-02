"""Kia Studio domain layer — kinematics, joint specs, observable model (UI-agnostic)."""

from .kinematics import (Geometry, Pose, fk_pose, fk_chain, fk_frames, chain_transforms,
                         chain_joint_points, DEFAULT_PIVOTS, ik, solve_ik, IkStatus, IkResult,
                         geometry_from_pivots, load_geometry, save_geometry,
                         firmware_geometry_header)
from .joints import JointSpec, JOINTS, GRIP, JointId
from .mesh_loader import Mesh, load_stl
from .animation import Keyframe, Sequence
from .trajectory import Trajectory, build_trajectory

__all__ = [
    "Geometry", "Pose", "fk_pose", "fk_chain", "fk_frames", "chain_transforms",
    "chain_joint_points", "DEFAULT_PIVOTS", "ik", "solve_ik", "IkStatus", "IkResult",
    "geometry_from_pivots", "load_geometry", "save_geometry", "firmware_geometry_header",
    "JointSpec", "JOINTS", "GRIP", "JointId", "Mesh", "load_stl",
    "Keyframe", "Sequence", "Trajectory", "build_trajectory",
]
