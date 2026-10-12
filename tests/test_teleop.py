import numpy as np
import pytest

from vrso101.robot.joints import load_robot_spec
from vrso101.robot.kinematics import Kinematics
from vrso101.teleop.controller import Status, TeleopController
from vrso101.teleop.transform import (FrameConfig, calibrate_yaw_from_forward, quat_to_matrix,
                                      xr_position_to_robot)

spec = load_robot_spec()
kin = Kinematics(spec)
IDENT_Q = (0.0, 0.0, 0.0, 1.0)


def test_xr_axes_map_to_ros():
    cfg = FrameConfig()
    # WebXR: 前方は -z, 右は +x, 上は +y  → ROS: 前 +x, 左 +y, 上 +z
    np.testing.assert_allclose(xr_position_to_robot([0, 0, -1], cfg), [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(xr_position_to_robot([1, 0, 0], cfg), [0, -1, 0], atol=1e-12)
    np.testing.assert_allclose(xr_position_to_robot([0, 1, 0], cfg), [0, 0, 1], atol=1e-12)


def test_scale_and_offset():
    cfg = FrameConfig(scale=0.5, origin_offset=[0.1, 0, 0])
    np.testing.assert_allclose(xr_position_to_robot([0, 0, -1], cfg), [0.6, 0, 0], atol=1e-12)


def test_rejects_absurd_scale():
    with pytest.raises(ValueError):
        FrameConfig(scale=100)


def test_quat_identity_and_normalisation():
    np.testing.assert_allclose(quat_to_matrix(IDENT_Q), np.eye(3), atol=1e-12)
    np.testing.assert_allclose(quat_to_matrix((0, 0, 0, 5)), np.eye(3), atol=1e-12)
    with pytest.raises(ValueError):
        quat_to_matrix((0, 0, 0, 0))


def test_calibration_makes_pointed_direction_robot_forward():
    # オペレータが WebXR の +x(右) を「前」と指した → その方向がロボット +x になる
    yaw = calibrate_yaw_from_forward([1, 0, 0])
    np.testing.assert_allclose(xr_position_to_robot([1, 0, 0], FrameConfig(yaw_offset=yaw)), [1, 0, 0], atol=1e-9)


def _ctl(**kw):
    return TeleopController(spec, kin, smoothing=0.0, **kw)


def test_clutch_does_not_jump_on_engage():
    c = _ctl()
    q0 = c.q.copy()
    out = c.update_vr([0.3, 1.2, -0.4], IDENT_Q, deadman=True)   # 任意の絶対位置で握る
    assert out.status in (Status.TRACKING, Status.WORKSPACE_CLAMPED)
    assert np.max(np.abs(out.q_target - q0)) < 0.05   # 握った瞬間は動かない


def test_follows_relative_motion_within_velocity_limit():
    c = _ctl()
    c.update_vr([0, 1.2, -0.3], IDENT_Q, deadman=True)
    q_prev = c.q.copy()
    for i in range(30):
        out = c.update_vr([0, 1.2, -0.3 - 0.004 * i], IDENT_Q, deadman=True)   # 前方へ 12cm
        assert np.all(np.abs(out.q_target - q_prev) <= spec.vel_limit * c.dt + 1e-9)
        q_prev = out.q_target
    x0 = kin.fk(spec.home)[0, 3]
    assert kin.fk(c.q)[0, 3] > x0 + 0.02   # 実際に前へ進んでいる


def test_deadman_release_holds_pose():
    c = _ctl()
    c.update_vr([0, 1, -0.3], IDENT_Q, deadman=True)
    c.update_vr([0, 1, -0.35], IDENT_Q, deadman=True)
    q = c.q.copy()
    out = c.update_vr([0, 1, -0.6], IDENT_Q, deadman=False)
    assert out.status == Status.IDLE
    np.testing.assert_array_equal(out.q_target, q)


def test_unreachable_target_is_clamped_and_never_nan():
    c = _ctl()
    c.update_vr([0, 1, 0], IDENT_Q, deadman=True)
    for i in range(60):
        out = c.update_vr([0, 1, -3.0], IDENT_Q, deadman=True)   # 3m 先
        assert np.all(np.isfinite(out.q_target))
        assert spec.within_limits(out.q_target)
    assert out.status in (Status.WORKSPACE_CLAMPED, Status.IK_FAILED, Status.TRACKING)
    assert kin.fk(c.q)[0, 3] <= c.workspace.upper[0] + 0.01


def test_stale_input_stops_tracking():
    t = [0.0]
    c = TeleopController(spec, kin, smoothing=0.0, clock=lambda: t[0], stale_timeout_s=0.5)
    c.update_vr([0, 1, -0.3], IDENT_Q, deadman=True)
    t[0] = 1.0
    assert c.check_stale()
    assert c.status == Status.STALE


def test_gripper_axis_maps_to_joint_range():
    c = _ctl()
    out = c.nudge([0, 0, 0], gripper=1.0)
    assert out.q_target[-1] > spec.lower[-1]
    for _ in range(100):
        out = c.nudge([0, 0, 0], gripper=1.0)
    assert out.q_target[-1] == pytest.approx(spec.upper[-1], abs=1e-6)


def test_nudge_keys_keep_working_after_orientation_becomes_unreachable():
    """人手の PC 操作で実測した不具合: W で奥まで動かした後、他のキーが全部 IK 失敗で固まった。
    pos_priority=True なら姿勢目標が実現不能でも位置キーが効き続ける。"""
    spec = load_robot_spec()
    kin = Kinematics(spec)
    c = TeleopController(spec, kin, rate_hz=25, smoothing=0.5)
    c.pos_priority = True
    dt, b = 1 / 25, 0.08
    for _ in range(50):
        c.nudge(np.array([b, 0, 0]) * dt)            # W を 2 秒
    p0 = kin.fk(c.q)[:3, 3].copy()
    for _ in range(50):
        c.nudge(np.array([0, -b, 0]) * dt)           # D を 2 秒
    assert np.linalg.norm(kin.fk(c.q)[:3, 3] - p0) > 0.08


def test_move_toward_follows_target_with_step_limit():
    """マウスドラッグ追従: 目標位置へ近づき、1 周期の移動量は max_step を超えない。"""
    spec = load_robot_spec()
    kin = Kinematics(spec)
    c = TeleopController(spec, kin, rate_hz=25, smoothing=0.0)
    c.pos_priority = True
    start = c._ee_target[:3, 3].copy()
    goal = start + np.array([0.0, 0.08, 0.0])
    max_step = 0.5 / 25
    first = c.move_toward(goal, max_step)
    assert np.linalg.norm(first.ee_target[:3, 3] - start) <= max_step + 1e-9
    for _ in range(40):
        c.move_toward(goal, max_step)
    assert np.linalg.norm(kin.fk(c.q)[:3, 3] - goal) < 0.01
