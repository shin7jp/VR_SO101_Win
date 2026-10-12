import numpy as np
import pytest

from vrso101.robot.joints import load_robot_spec
from vrso101.robot.kinematics import Kinematics

spec = load_robot_spec()


def test_joint_order_matches_lerobot_so101():
    assert spec.names == ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def test_limits_match_urdf():
    k = Kinematics(spec)
    for j in spec.joints:
        lo, up = k.urdf_joint_limits[j.name]
        assert j.lower == pytest.approx(lo, abs=1e-5), j.name
        assert j.upper == pytest.approx(up, abs=1e-5), j.name


def test_reorder_by_name():
    q = spec.reorder(["gripper", "shoulder_pan", "wrist_roll", "elbow_flex", "shoulder_lift", "wrist_flex"],
                     [5, 0, 4, 2, 1, 3])
    assert list(q) == [0, 1, 2, 3, 4, 5]


def test_reorder_rejects_missing_joint():
    with pytest.raises(KeyError):
        spec.reorder(["shoulder_pan"], [0.0])


def test_home_inside_limits():
    assert spec.within_limits(spec.home)


def test_limit_step_respects_velocity_and_bounds():
    q = spec.limit_step(np.zeros(6), np.full(6, 10.0), dt=0.1)
    assert np.all(q <= spec.vel_limit * 0.1 + 1e-12)
    assert spec.within_limits(q)
    q = spec.limit_step(spec.upper, spec.upper + 1, dt=1.0)
    assert np.all(q <= spec.upper + 1e-12)
