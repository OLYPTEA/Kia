"""Keyframe animation tests — interpolation, ordering, save/load."""
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kia_studio.core.animation import Keyframe, Sequence  # noqa: E402


def _seq():
    s = Sequence("test")
    s.add(0.0, [0, 90, 0, 0, 0])
    s.add(2.0, [40, 60, -30, 10, 90])
    return s


def test_endpoints_and_midpoint():
    s = _seq()
    assert s.duration == 2.0
    assert s.sample(0.0) == [0, 90, 0, 0, 0]
    assert s.sample(2.0) == [40, 60, -30, 10, 90]
    mid = s.sample(1.0)
    assert mid == [20, 75, -15, 5, 45]            # exact linear midpoint


def test_clamp_outside_range():
    s = _seq()
    assert s.sample(-5.0) == [0, 90, 0, 0, 0]
    assert s.sample(99.0) == [40, 60, -30, 10, 90]


def test_smoothing_matches_endpoints_but_differs_midway():
    s = _seq()
    assert s.sample(0.0, smooth=True) == [0, 90, 0, 0, 0]
    lin = s.sample(0.5)
    eased = s.sample(0.5, smooth=True)
    assert eased != lin                            # cosine ease bends the curve
    assert math.isclose(s.sample(1.0, smooth=True)[0], 20.0, abs_tol=1e-9)  # symmetric at center


def test_keyframes_kept_sorted():
    s = Sequence()
    s.add(2.0, [0, 0, 0, 0, 0])
    s.add(1.0, [1, 1, 1, 1, 1])
    assert [k.t for k in s.keyframes] == [1.0, 2.0]


def test_empty_sequence_samples_none():
    assert Sequence().sample(1.0) is None


def test_save_load_roundtrip():
    s = _seq()
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "seq.json")
        s.save(p)
        r = Sequence.load(p)
    assert r.name == s.name
    assert [k.t for k in r.keyframes] == [0.0, 2.0]
    assert r.sample(1.0) == s.sample(1.0)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn(); print(f"  ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")
