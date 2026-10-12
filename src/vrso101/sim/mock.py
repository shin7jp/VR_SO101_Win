"""モックのシム (SimClient 互換)。Gazebo/GPU の無い CI や UI 開発用。

重要: これは Gazebo ではない。物理はなく、関節は一次遅れ、物体は「顎が閉じて近ければ付いてくる」だけの
疑似的なもの。モックで通ったテストを Gazebo 連携の合格とは見なさない (backend="mock" で必ず区別する)。
"""
from __future__ import annotations

import numpy as np

from ..robot.joints import RobotSpec
from ..robot.kinematics import Kinematics
from ..tasks.registry import get_task, sample_scene
from ..tasks.success import TaskEvaluator
from .client import Observation

IMG_W, IMG_H = 160, 120   # モックは小さい画像で十分 (速度優先)


class MockSim:
    backend = "mock"

    def __init__(self, spec: RobotSpec, kin: Kinematics | None = None):
        self.spec = spec
        self.kin = kin or Kinematics(spec)
        self.task = None
        self.evaluator: TaskEvaluator | None = None
        self.connected = True
        self.fail_next_step = False   # テスト用: 通信切断を注入する

    def connect(self):
        return {"backend": "mock", "joint_names": self.spec.names, "units": self.spec.units}

    def close(self):
        self.connected = False

    def load_task(self, task_id: str, fps: float | None = None):
        from .client import SimCommandError
        if get_task(task_id)["success"]["type"] != "pick_place":
            raise SimCommandError("MOCK_UNSUPPORTED", f"モックは pick_place のみ対応です ({task_id} には Gazebo が必要)")
        self.task = {**get_task(task_id), **({"fps": fps} if fps else {})}
        self.evaluator = TaskEvaluator(self.task)
        return {"task_id": task_id, "fps": self.task["fps"], "backend": "mock"}

    def reset(self, seed: int = 0, randomize: bool = True) -> Observation:
        self.scene = sample_scene(self.task, seed, randomize)
        self.q = self.spec.home.copy()
        self.q_cmd = self.q.copy()
        self.t = 0.0
        self.k = 0
        cu = self.scene.get("cube")
        self.cube = np.array([cu["x"], cu["y"], cu["z"]]) if cu else None
        self.held = False
        self.max_lift = 0.0
        self.evaluator.reset()
        self._evaluate()
        return self._obs(scene=self.scene, wall=None)

    def step(self, q) -> Observation:
        if self.fail_next_step:
            from .client import SimUnavailable
            raise SimUnavailable("モック: 通信切断を注入")
        dt = 1.0 / self.task["fps"]
        self.q_cmd = self.spec.clip_to_limits(np.asarray(q, dtype=np.float64))
        self.q = self.q + 0.6 * (self.q_cmd - self.q)                         # 一次遅れ
        self.t += dt
        self.k += 1
        if self.cube is not None:
            ee = self.kin.fk(self.q)[:3, 3]
            closed = self.q[-1] < 0.35
            near = np.linalg.norm(ee - self.cube) < 0.035
            self.held = (self.held and closed) or (closed and near)
            if self.held:
                self.cube = ee.copy()
            else:
                self.cube[2] = 0.015
        self._evaluate()
        return self._obs(wall=1.0)

    def get_obs(self) -> Observation:
        return self._obs(wall=None)

    def _evaluate(self):
        st = {"objects": {"cube": {"pos": list(self.cube)}} if self.cube is not None else {}, "joints": {}}
        if self.task["success"]["type"] == "pick_place":
            self.evaluator.update(st)

    def _image(self) -> np.ndarray:
        img = np.full((IMG_H, IMG_W, 3), 170, np.uint8)
        # 関節角に依存する見た目 (学習・形状検証用): 帯の位置が q で動く
        for i, v in enumerate(self.q[:5]):
            x = int((np.clip(v, -2, 2) + 2) / 4 * (IMG_W - 8))
            img[10 + i * 20: 18 + i * 20, x: x + 8] = (255, 80 + 30 * i, 40)
        if self.cube is not None:
            cx = int(np.clip(self.cube[0] / 0.4 * IMG_W, 0, IMG_W - 6))
            cy = int(np.clip(IMG_H - 10 - (self.cube[1] + 0.2) / 0.4 * 60, 0, IMG_H - 6))
            img[cy:cy + 5, cx:cx + 5] = (200, 20, 20)
        return img

    def _obs(self, scene=None, wall=None) -> Observation:
        ev = self.evaluator.last
        return Observation(sim_time=self.t, step=self.k, q=self.q.copy(), qd=np.zeros(self.spec.dof),
                           q_cmd=self.q_cmd.copy(), image=self._image(), joint_stamp=self.t, image_stamp=self.t,
                           task={"success": self.evaluator.success, "metrics": ev.metrics, "detail": ev.detail},
                           scene=scene, step_wall_ms=wall,
                           objects=({"cube": {"pos": list(self.cube), "quat": [0, 0, 0, 1]}} if self.cube is not None else {}))
