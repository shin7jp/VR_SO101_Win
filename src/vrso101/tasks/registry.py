"""タスク定義 (config/tasks/*.yaml) の読み込みと、シードからのシーン初期化パラメータ生成。"""
from __future__ import annotations

import math
from functools import lru_cache
from typing import Any

import numpy as np
import yaml

from ..paths import CONFIG_DIR

REQUIRED = ("task_id", "name", "fps", "max_steps", "camera", "scene", "success")


@lru_cache(maxsize=1)
def load_tasks() -> dict[str, dict[str, Any]]:
    tasks = {}
    for p in sorted((CONFIG_DIR / "tasks").glob("*.yaml")):
        cfg = yaml.safe_load(p.read_text(encoding="utf-8"))
        missing = [k for k in REQUIRED if k not in cfg]
        if missing:
            raise ValueError(f"{p.name}: 必須キー不足 {missing}")
        if cfg["task_id"] in tasks:
            raise ValueError(f"task_id 重複: {cfg['task_id']}")
        tasks[cfg["task_id"]] = cfg
    return tasks


def get_task(task_id: str) -> dict[str, Any]:
    t = load_tasks()
    if task_id not in t:
        raise KeyError(f"未知のタスク: {task_id} (有効: {sorted(t)})")
    return t[task_id]


def sample_scene(task_cfg: dict[str, Any], seed: int, randomize: bool = True) -> dict[str, Any]:
    """シードから初期配置を決める。範囲は yaml の randomize に限定し、到達不能な配置を作らない。"""
    rng = np.random.default_rng(seed)
    out: dict[str, Any] = {"seed": int(seed)}
    rz = task_cfg.get("randomize", {})
    tid = task_cfg["task_id"]

    def uni(lo_hi, default):
        return float(rng.uniform(*lo_hi)) if randomize else float(default)

    if tid == "pick_place":
        half = task_cfg["scene"]["cube"]["size"] / 2
        cx, cy = rz["cube_xy"]["x"], rz["cube_xy"]["y"]
        out["cube"] = {"x": uni(cx, sum(cx) / 2), "y": uni(cy, sum(cy) / 2), "z": half,
                       "yaw": uni(rz["cube_yaw"], 0.0)}
        out["target"] = dict(zip(("x", "y"), task_cfg["scene"]["target"]["center"]))
    else:
        hx, hy = rz["door_hinge_xy"]["x"], rz["door_hinge_xy"]["y"]
        out["door"] = {"hinge_x": uni(hx, sum(hx) / 2), "hinge_y": uni(hy, sum(hy) / 2)}
    return out


def reachable(scene: dict[str, Any], max_reach: float = 0.36) -> bool:
    """水平距離が腕の到達半径に収まるか (ランダム化の健全性テスト用)。"""
    for k in ("cube", "door"):
        if k in scene:
            x = scene[k].get("x", scene[k].get("hinge_x"))
            y = scene[k].get("y", scene[k].get("hinge_y"))
            if math.hypot(x, y) > max_reach:
                return False
    return True
