import numpy as np
import pytest

from vrso101.robot.joints import load_robot_spec
from vrso101.robot.kinematics import Kinematics
from vrso101.tasks.registry import get_task, sample_scene
from vrso101.teleop.controller import Status, TeleopController
from vrso101.teleop.scripted import GRASP_OFFSET_EE, ScriptedPickPlace, choose_closing_axis, pose_from, rho_for_axis

spec = load_robot_spec()
kin = Kinematics(spec)


def test_pose_from_is_proper_rotation_and_points_down():
    T = pose_from(np.array([0.2, 0.1, 0.05]), np.pi / 2, 0.3)
    R = T[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert np.linalg.det(R) == pytest.approx(1.0)
    np.testing.assert_allclose(R[:, 2], [0, 0, -1], atol=1e-12)
    assert abs(R[2, 0]) < 1e-12   # 顎の開閉軸は水平


def test_rho_for_axis_roundtrip():
    p = np.array([0.22, -0.07, 0.05])
    u = np.array([np.cos(0.4), np.sin(0.4)])
    rho = rho_for_axis(p, u)
    x_ee = pose_from(p, np.pi / 2, rho)[:3, 0]
    np.testing.assert_allclose(x_ee[:2], u, atol=1e-9)


def test_closing_axis_is_cube_face_normal():
    for yaw in (-0.8, 0.0, 0.5, 0.8):
        u = choose_closing_axis(np.array([0.2, -0.05]), yaw)
        ang = np.arctan2(u[1], u[0]) - yaw
        assert abs(np.sin(2 * ang)) < 1e-9   # yaw から 90° 刻み


@pytest.mark.parametrize("seed", range(12))
def test_scripted_plan_is_kinematically_feasible(seed):
    """人間の代わりに IK 経路だけで全フェーズが解けること (物理は含まない)。"""
    task = get_task("pick_place")
    sc = sample_scene(task, seed)
    c = sc["cube"]
    plan = ScriptedPickPlace((c["x"], c["y"], c["z"]), c["yaw"], (sc["target"]["x"], sc["target"]["y"]))
    ctl = TeleopController(spec, kin, rate_hz=25, smoothing=0.0)
    ctl.ik_rot_weight = 1.0
    ctl.workspace.lower[2] = -0.01
    fails, final = {}, {}
    for name, out in plan.run(ctl):
        if out.status == Status.IK_FAILED:
            fails[name] = fails.get(name, 0) + 1
        # 速度制限で q が遅れる分も含め、各区間の最終周期での実 EE 位置誤差を見る
        final[name] = np.linalg.norm(kin.fk(out.q_target)[:3, 3] - out.ee_target[:3, 3])
    # 開始直後 (ホーム → 真下姿勢への大きな再構成) の過渡的な IK 未収束は許容。把持以降は 1 度も失敗しないこと
    assert not {k: v for k, v in fails.items() if k != "pregrasp"}, f"IK 失敗 {fails} (seed={seed}, cube={c})"
    assert final["descend"] < 0.006, final
    assert final["lower"] < 0.006, final


def test_grasp_offset_places_cube_between_jaws():
    # 把持姿勢での EE 位置 + R@offset = 立方体中心
    R = pose_from(np.array([0.2, 0.0, 0.05]), np.pi / 2, rho_for_axis(np.array([0.2, 0.0, 0.05]), np.array([-1.0, 0.0])))[:3, :3]
    cube = np.array([0.2, 0.0, 0.015])
    p_ee = cube - R @ GRASP_OFFSET_EE
    assert p_ee[2] == pytest.approx(0.005)
