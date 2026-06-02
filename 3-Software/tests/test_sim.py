"""Device simulator tests — exercise the wire path end-to-end without Qt/hardware."""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kia_studio.proto import KiaClient  # noqa: E402
from kia_studio.proto.opcodes import Mode  # noqa: E402
from kia_studio.core.sim import SimTransport  # noqa: E402
from kia_studio.core.joints import JOINTS  # noqa: E402
from kia_studio.core.kinematics import fk_pose  # noqa: E402


def _wait(pred, timeout=2.0, dt=0.02):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return True
        time.sleep(dt)
    return False


def test_ping_pong():
    c = KiaClient(SimTransport())
    got = []
    c.on_pong = got.append
    c.ping()
    assert _wait(lambda: bool(got))
    assert got[0] == (1, 0, 0)


def test_telemetry_stream_and_motion():
    t = SimTransport(telemetry_hz=50)
    c = KiaClient(t)
    acks = []
    c.on_ack = acks.append
    assert _wait(lambda: c.last_telemetry is not None)
    c.set_joint(0, 45.0, 1.0)
    assert _wait(lambda: bool(acks))
    # current joint should slew toward the 45 deg target
    assert _wait(lambda: c.last_telemetry.joints[0] > 5.0, timeout=2.0)
    t.close()


def test_set_joint_is_clamped():
    t = SimTransport(telemetry_hz=50)
    c = KiaClient(t)
    spec = JOINTS[1]  # shoulder, max 90
    c.set_joint(1, spec.max_deg + 100.0)
    assert _wait(lambda: c.last_telemetry is not None
                 and abs(c.last_telemetry.joints[1] - spec.max_deg) < spec.span, timeout=2.0)
    # never exceeds the limit
    time.sleep(0.2)
    assert c.last_telemetry.joints[1] <= spec.max_deg + 1e-3
    t.close()


def test_home_resets():
    t = SimTransport(telemetry_hz=50)
    c = KiaClient(t)
    c.set_joint(0, 60.0)
    assert _wait(lambda: c.last_telemetry and c.last_telemetry.joints[0] > 10.0)
    c.home()
    assert _wait(lambda: c.last_telemetry and abs(c.last_telemetry.joints[0]) < 1.0, timeout=2.0)
    t.close()


def test_set_xyz_moves_via_ik():
    t = SimTransport(telemetry_hz=50)
    c = KiaClient(t)
    target = fk_pose(20, 80, -40, 10)   # a reachable pose
    c.set_xyz(target.x, target.y, target.z, target.pitch, 0.5)
    # arm should converge so its reported TCP approaches the target
    def near():
        st = c.last_telemetry
        return st and abs(st.xyz[0] - target.x) < 3 and abs(st.xyz[2] - target.z) < 3
    assert _wait(near, timeout=3.0)
    t.close()


def test_hold_blocks_motion_then_arm_resumes():
    t = SimTransport(telemetry_hz=50)
    c = KiaClient(t)
    nacks = []
    c.on_nack = lambda seq, r: nacks.append(r)
    assert _wait(lambda: c.last_telemetry is not None)
    # STOP: HOLD + MODE HOLD
    c.hold()
    c.mode_set(Mode.HOLD)
    c.set_joint(0, 45.0)
    assert _wait(lambda: 0x09 in nacks, timeout=1.5)   # BAD_STATE
    pos_before = c.last_telemetry.joints[0]
    time.sleep(0.2)
    assert abs(c.last_telemetry.joints[0] - pos_before) < 1e-2  # frozen
    # ARM re-enables
    c.arm()
    c.set_joint(0, 45.0)
    assert _wait(lambda: c.last_telemetry.joints[0] > pos_before + 3.0, timeout=2.0)
    t.close()


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn(); print(f"  ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")
