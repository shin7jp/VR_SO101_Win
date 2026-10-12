"""エピソード実行ループ (スクリプトデモ収録・ACT 推論で共通)。UI からも CLI からも同じ関数を使う。

人間のテレオペ収録は入力待ちのリアルタイムループになるため api/session.py 側に別ループがあるが、
記録は同じ EpisodeRecorder を通す。
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Iterator

import numpy as np

from .recording.recorder import EpisodeRecorder
from .recording.store import EpisodeStore
from .robot.joints import RobotSpec
from .robot.kinematics import Kinematics
from .sim.client import Observation
from .tasks.registry import get_task
from .teleop.controller import TeleopController
from .teleop.scripted import ScriptedPickPlace


def scripted_actions(spec: RobotSpec, kin: Kinematics, task_id: str, obs: Observation, fps: float) -> Iterator[np.ndarray]:
    """スクリプト指令列。TeleopController (IK/制限/平滑化) を通した関節指令を yield する。"""
    task = get_task(task_id)
    ctl = TeleopController(spec, kin, rate_hz=fps, smoothing=0.0)
    ctl.ik_rot_weight = 1.0
    ctl.workspace.lower[2] = -0.01     # 把持高さ (EE z=0.005) を許す
    ctl.reset(obs.q)
    if task_id == "pick_place":
        cu, tg = obs.scene["cube"], obs.scene["target"]
        plan = ScriptedPickPlace((cu["x"], cu["y"], cu["z"]), cu["yaw"], (tg["x"], tg["y"]))
    elif task["success"]["type"] in ("open_door", "lever_door"):
        from .teleop.scripted_door import ScriptedDoor
        plan = ScriptedDoor(task, obs.scene)
    else:
        raise NotImplementedError(f"スクリプトデモ未対応: {task_id}")
    for _name, out in plan.run(ctl):
        yield out.q_target
    for _ in range(15):                # 最終静止 (物体が落ち着いてから成否を見る)
        yield ctl.q


def run_episode(sim, spec: RobotSpec, store: EpisodeStore | None, task_id: str, seed: int,
                actions: Callable[[Observation], Iterator[np.ndarray]], *, meta: dict | None = None,
                max_steps: int | None = None, stop: threading.Event | None = None,
                on_step: Callable[[Observation, int], None] | None = None, record: bool = True,
                randomize: bool = True) -> dict:
    """reset → (指令 → step → 記録) を繰り返す。戻り値: {status, success, steps, episode_id, task, ...}"""
    task = get_task(task_id)
    fps = float(task["fps"])
    max_steps = max_steps or int(task["max_steps"])
    obs = sim.reset(seed=seed, randomize=randomize)
    rec = None
    if record and store is not None:
        rec = EpisodeRecorder(store, spec, task_id, fps, meta={
            "seed": seed, "scene": obs.scene, "backend": getattr(sim, "backend", "gazebo"), **(meta or {})})
        rec.prime(obs)
    gen = actions(obs)
    steps, status, reason = 0, "failure", None
    try:
        for q in gen:
            if stop is not None and stop.is_set():
                status, reason = "aborted", "停止要求"
                break
            if steps >= max_steps:
                reason = "timeout"
                break
            obs = sim.step(q)
            steps += 1
            if rec:
                rec.add_step(obs)
            if on_step:
                on_step(obs, steps)
        else:
            reason = reason or "plan_finished"
        if status != "aborted":
            status = "success" if obs.task.get("success") else "failure"
    except Exception as e:   # 通信切断などは「中断」として部分データを残す (消さない)
        status, reason = "aborted", f"{type(e).__name__}: {e}"
        if rec:
            rec.abort(reason)
        raise
    result = {"status": status, "success": bool(obs.task.get("success")), "steps": steps, "reason": reason,
              "task": obs.task, "seed": seed, "scene": obs.scene}
    if rec:
        m = rec.finish(status, task_result=obs.task, abort_reason=reason if status == "aborted" else None)
        result["episode_id"] = m["episode_id"]
        result["quality_ok"] = m["quality"]["ok"]
    return result


def make_sim(backend: str, spec: RobotSpec, kin: Kinematics | None = None):
    if backend == "mock":
        from .sim.mock import MockSim
        return MockSim(spec, kin)
    if backend == "gazebo":
        from .sim.client import SimClient
        return SimClient()
    raise ValueError(f"未知の backend: {backend} (gazebo | mock)")


def timed(fn):
    t0 = time.perf_counter()
    r = fn()
    return r, time.perf_counter() - t0
