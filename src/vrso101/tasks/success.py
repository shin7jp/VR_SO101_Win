"""タスクの成功判定。シミュレータの物理状態 (物体位置・ヒンジ角) だけを入力にする。

見た目や ACT の出力で成功扱いにしない、という要件のため、入力は sim が返す `state` dict のみ。
標準ライブラリ + numpy のみ (WSL の Gazebo ブリッジ側からも import される)。

state の形:
  {"objects": {"cube": {"pos": [x,y,z]}, ...},
   "joints": {"door_hinge": 0.3, "lever": 0.1}}   # 物体側関節の角度 [rad]
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


@dataclass
class Evaluation:
    success: bool
    metrics: dict[str, float] = field(default_factory=dict)
    detail: str = ""


class TaskEvaluator:
    """ステップごとに update() を呼ぶ。履歴 (最大持ち上げ高さ・レバー先行など) を内部に持つ。"""

    def __init__(self, task_cfg: dict[str, Any]):
        self.cfg = task_cfg
        self.crit = task_cfg["success"]
        self.kind = self.crit["type"]
        self.reset()

    def reset(self) -> None:
        self._streak = 0
        self._max_lift = 0.0
        self._lever_released_at: int | None = None
        self._max_door = 0.0
        self._max_lever = 0.0
        self._step = 0
        self._success = False
        self.last = Evaluation(False)

    @property
    def success(self) -> bool:
        return self._success

    def update(self, state: dict[str, Any]) -> Evaluation:
        self._step += 1
        fn = {"pick_place": self._pick_place, "open_door": self._open_door, "lever_door": self._lever_door}.get(self.kind)
        if fn is None:
            raise ValueError(f"未知の success.type: {self.kind}")
        ok, metrics, detail = fn(state)
        self._streak = self._streak + 1 if ok else 0
        # 成功は一度確定したら維持 (その後に物体が動いても成功エピソードを取り消さない)
        if self._streak >= int(self.crit.get("settle_steps", 1)):
            self._success = True
        self.last = Evaluation(self._success, metrics, detail)
        return self.last

    # ---- Pick and Place ----
    def _pick_place(self, s):
        cube = s["objects"]["cube"]["pos"]
        tgt = self.cfg["scene"]["target"]["center"]
        half = self.cfg["scene"]["cube"]["size"] / 2
        self._max_lift = max(self._max_lift, cube[2] - half)
        dist = math.hypot(cube[0] - tgt[0], cube[1] - tgt[1])
        at_rest_z = abs(cube[2] - half) <= self.crit["rest_height_tol"]
        lifted = self._max_lift >= self.crit["lift_threshold"]
        ok = dist <= self.crit["place_tolerance"] and at_rest_z and lifted
        detail = "" if ok else ("未持ち上げ" if not lifted else "目標外" if dist > self.crit["place_tolerance"] else "設置前")
        return ok, {"place_error": dist, "max_lift": self._max_lift, "grasped_lifted": float(lifted)}, detail

    # ---- Open Door ----
    def _open_door(self, s):
        a = abs(s["joints"]["door_hinge"])   # 押しても引いても「開いた」(ヒンジは両方向に回れる)
        self._max_door = max(self._max_door, a)
        ok = a >= self.crit["target_angle"]
        return ok, {"door_angle": a, "max_door_angle": self._max_door}, "" if ok else "開放角不足"

    # ---- Rotating Lever Door ----
    def _lever_door(self, s):
        lev, door = s["joints"]["lever"], abs(s["joints"]["door_hinge"])
        self._max_lever = max(self._max_lever, lev)
        self._max_door = max(self._max_door, door)
        if self._lever_released_at is None and lev >= self.crit["lever_threshold"]:
            self._lever_released_at = self._step
        door_open = door >= self.crit["target_angle"]
        ordered = (not self.crit.get("require_order", True)) or self._lever_released_at is not None
        ok = door_open and ordered
        detail = "" if ok else ("レバー未操作" if self._lever_released_at is None else "ドア開放角不足")
        return ok, {"lever_angle": lev, "max_lever_angle": self._max_lever, "door_angle": door,
                    "max_door_angle": self._max_door, "latch_released": float(self._lever_released_at is not None)}, detail
