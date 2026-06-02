"""3D-layer tests that need no GL context: FK frames, STL loader, calibration matrix."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kia_studio.core import (Geometry, fk_chain, fk_frames, chain_transforms,  # noqa: E402
                             chain_joint_points, DEFAULT_PIVOTS, load_stl)
from kia_studio.ui.mesh_config import (LINK_META, LINK_ORDER, calib_matrix,  # noqa: E402
                                       default_calib, hinge_matrix)

_MESH_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         "kia_studio", "resources", "meshes")
G = Geometry()


def test_fk_frames_origins_match_chain():
    pose = (20.0, 70.0, -30.0, 15.0)
    pts = fk_chain(*pose, G)
    frames = fk_frames(*pose, G)
    # frame origins: base@floor, shoulder, elbow, wrist, grip(=wrist)
    expect = [pts[0], pts[1], pts[2], pts[3], pts[3]]
    for f, p in zip(frames, expect):
        assert np.allclose(f[:3, 3], p, atol=1e-6)


def test_fk_frames_are_orthonormal():
    frames = fk_frames(30.0, 95.0, -40.0, 10.0, G)
    for f in frames:
        R = f[:3, :3]
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
        assert math.isclose(np.linalg.det(R), 1.0, abs_tol=1e-9)


def test_link_local_z_points_along_link():
    # shoulder frame local +Z must point from shoulder toward elbow
    pose = (0.0, 60.0, 0.0, 0.0)
    pts = fk_chain(*pose, G)
    frames = fk_frames(*pose, G)
    z_axis = frames[1][:3, 2]
    seg = np.array(pts[2]) - np.array(pts[1])
    seg /= np.linalg.norm(seg)
    assert np.allclose(z_axis, seg, atol=1e-6)


def test_stl_loader_all_links():
    loaded = 0
    for name in LINK_ORDER:
        path = os.path.join(_MESH_DIR, LINK_META[name]["file"])
        if not os.path.exists(path):
            continue  # some links may not be re-exported yet
        m = load_stl(path)
        assert m.vertices.shape[0] > 3
        assert m.faces.shape[1] == 3
        assert m.faces.max() < m.vertices.shape[0]
        assert np.all(m.extent > 0)
        loaded += 1
    assert loaded >= 4


def test_calib_recenter_moves_centroid_to_origin():
    center = np.array([100.0, -50.0, 200.0])
    cfg = default_calib()["base"]  # recenter=True, no rot/trans, scale s
    M = calib_matrix(center, cfg)
    out = M @ np.array([center[0], center[1], center[2], 1.0])
    assert np.allclose(out[:3], 0.0, atol=1e-6)


def test_calib_scale_applied():
    cfg = dict(recenter=False, scale=0.1, rx=0, ry=0, rz=0, tx=0, ty=0, tz=0)
    M = calib_matrix(np.zeros(3), cfg)
    out = M @ np.array([10.0, 0.0, 0.0, 1.0])
    assert math.isclose(out[0], 1.0, abs_tol=1e-6)


def test_chain_identity_at_home():
    # at the home pose every link transform is identity (meshes stay at home placement)
    M = chain_transforms(0.0, 90.0, 0.0, 0.0, DEFAULT_PIVOTS)
    for name in ("bati", "base", "shoulder", "elbow", "wrist", "grip1", "grip2"):
        assert np.allclose(M[name], np.eye(4), atol=1e-9), name


def test_chain_downstream_follows_upstream_joint():
    # moving the elbow must move the wrist AND the grip transforms (priority chain)
    M0 = chain_transforms(0.0, 90.0, 0.0, 0.0, DEFAULT_PIVOTS)
    M1 = chain_transforms(0.0, 90.0, 40.0, 0.0, DEFAULT_PIVOTS)
    assert np.allclose(M0["base"], M1["base"], atol=1e-9)       # upstream unchanged
    assert not np.allclose(M0["wrist"], M1["wrist"], atol=1e-6)  # wrist follows elbow
    assert not np.allclose(M0["grip1"], M1["grip1"], atol=1e-6)  # grip follows wrist


def test_joint_rotates_about_its_pivot():
    # the elbow pivot point must stay fixed when only the elbow moves
    piv = dict(DEFAULT_PIVOTS)
    M = chain_transforms(0.0, 90.0, 35.0, 0.0, piv)
    p = np.append(piv["elbow"], 1.0)
    # elbow pivot carried by its parent (shoulder) — fixed because shoulder didn't move
    moved = M["elbow"] @ p
    assert np.allclose(moved[:3], piv["elbow"], atol=1e-6)


def test_chain_skeleton_wrist_follows_elbow():
    p0 = chain_joint_points(0.0, 90.0, 0.0, 0.0, DEFAULT_PIVOTS)
    p1 = chain_joint_points(0.0, 90.0, 45.0, 0.0, DEFAULT_PIVOTS)
    assert np.allclose(p0[2], p1[2], atol=1e-6)        # elbow joint fixed
    assert not np.allclose(p0[3], p1[3], atol=1e-6)    # wrist joint moved


def test_hinge_identity_when_gain_zero():
    assert np.allclose(hinge_matrix(dict(hgain=0.0), 90.0), np.eye(4))


def test_hinge_rotates_about_pivot_axis():
    # axis Z through pivot (5,0,0); a point on the pivot stays put, grip opens the jaw
    cfg = dict(hx=5.0, hy=0.0, hz=0.0, haxis="z", hgain=1.0)
    M = hinge_matrix(cfg, 90.0)             # 90 deg about Z @ (5,0,0)
    fixed = M @ np.array([5.0, 0.0, 0.0, 1.0])
    assert np.allclose(fixed[:3], [5.0, 0.0, 0.0], atol=1e-6)
    moved = M @ np.array([6.0, 0.0, 0.0, 1.0])   # +X from pivot -> +Y after +90 about Z
    assert np.allclose(moved[:3], [5.0, 1.0, 0.0], atol=1e-6)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn(); print(f"  ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")
