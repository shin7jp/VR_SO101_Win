import numpy as np
import pytest

from vrso101.robot.joints import load_robot_spec
from vrso101.robot.kinematics import Kinematics
from vrso101.tasks.door_geometry import door_geometry
from vrso101.tasks.registry import get_task, sample_scene
from vrso101.tasks.success import TaskEvaluator
from vrso101.teleop.controller import Status, TeleopController
from vrso101.teleop.scripted_door import ScriptedDoor

spec = load_robot_spec()
kin = Kinematics(spec)


def test_door_geometry_is_consistent():
    gl = door_geometry(get_task("lever_door"))
    # アーム先端は自由端を越えて受け金に掛かり、スピンドルは板内側
    assert gl["spindle_y"] < gl["w"] < gl["spindle_y"] + gl["lever_len"]
    # 先端が受け金 (下端 = アーム水平時の上面より 8mm 下) の下へ抜けるのに必要なレバー角は、成功しきい値 0.5 より小さい
    need = np.arcsin(0.008 / gl["lever_len"])
    assert 0 < need < get_task("lever_door")["success"]["lever_threshold"]
    # ペグの把持点は面と頭の間に収まる
    go = door_geometry(get_task("open_door"))
    assert -go["t"] / 2 - go["peg_len"] < go["handle_x"] < -go["t"] / 2


@pytest.mark.parametrize("task_id", ["open_door", "lever_door"])
@pytest.mark.parametrize("seed", range(6))
def test_door_plan_is_kinematically_feasible(task_id, seed):
    task = get_task(task_id)
    sc = sample_scene(task, seed)
    plan = ScriptedDoor(task, sc)
    ctl = TeleopController(spec, kin, rate_hz=25, smoothing=0.0)
    ctl.ik_rot_weight = 1.0
    ctl.workspace.lower[2] = -0.01
    ctl.reset(np.array(task["home"], dtype=float))
    fails, final = {}, {}
    for name, out in plan.run(ctl):
        if out.status == Status.IK_FAILED:
            fails[name] = fails.get(name, 0) + 1
        final[name] = np.linalg.norm(kin.fk(out.q_target)[:3, 3] - out.ee_target[:3, 3])
    assert not fails, f"IK 失敗 {fails} ({task_id} seed={seed})"
    assert final["push"] < 0.008, final


def test_home_pose_clear_of_door_panel():
    """ホーム姿勢のグリッパーがドア板に食い込まない (食い込むと物理が破綻した: 実測)。"""
    for tid in ("open_door", "lever_door"):
        task = get_task(tid)
        ee = kin.fk(np.array(task["home"], dtype=float))[:3, 3]
        for s in range(20):
            sc = sample_scene(task, s)
            hx, hy = sc["door"]["hinge_x"], sc["door"]["hinge_y"]
            w = task["scene"]["door"]["width"]
            assert not (hy - 0.02 < ee[1] < hy + w + 0.02 and abs(ee[0] - hx) < 0.06), (tid, s, ee)


def test_door_success_either_direction():
    ev = TaskEvaluator(get_task("open_door"))
    for _ in range(15):
        ev.update({"objects": {}, "joints": {"door_hinge": -0.8}})   # 押して開けた
    assert ev.success
    ev = TaskEvaluator(get_task("lever_door"))
    for _ in range(15):   # レバー未操作でドアだけ開く (ラッチ迂回) は成功にしない
        ev.update({"objects": {}, "joints": {"lever": 0.0, "door_hinge": -0.8}})
    assert not ev.success
