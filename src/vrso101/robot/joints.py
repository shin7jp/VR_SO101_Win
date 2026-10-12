"""関節マッピング・関節/速度制限。config/robot/so101.yaml を唯一の定義とする。"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np
import yaml

from ..paths import CONFIG_DIR


@dataclass(frozen=True)
class Joint:
    name: str
    lower: float
    upper: float
    vel_limit: float  # rad/s


class RobotSpec:
    def __init__(self, raw: dict):
        self.robot_id: str = raw["robot_id"]
        self.units: str = raw["units"]
        self.urdf: str = raw["urdf"]
        self.base_link: str = raw["base_link"]
        self.end_effector_link: str = raw["end_effector_link"]
        self.joints: list[Joint] = [Joint(**j) for j in raw["joints"]]
        self.home = np.asarray(raw["home"], dtype=np.float64)
        if len(self.home) != len(self.joints):
            raise ValueError("home の次元が関節数と一致しません")
        names = [j.name for j in self.joints]
        if len(set(names)) != len(names):
            raise ValueError("関節名が重複しています")
        for j in self.joints:
            if not j.lower < j.upper:
                raise ValueError(f"{j.name}: lower<upper でありません")
        self.lower = np.array([j.lower for j in self.joints])
        self.upper = np.array([j.upper for j in self.joints])
        self.vel_limit = np.array([j.vel_limit for j in self.joints])

    @property
    def names(self) -> list[str]:
        return [j.name for j in self.joints]

    @property
    def dof(self) -> int:
        return len(self.joints)

    @property
    def arm_names(self) -> list[str]:
        return self.names[:-1]  # 末尾がグリッパー

    def index(self, name: str) -> int:
        return self.names.index(name)

    # --- 並び替え: 外部 (ROS JointState 等) は順序が保証されないので名前で引き直す ---
    def reorder(self, names: list[str], values) -> np.ndarray:
        """任意順の (names, values) を設定上の関節順に並べ替える。欠損・余剰はエラー。"""
        if len(names) != len(values):
            raise ValueError("names と values の長さが違います")
        m = dict(zip(names, values))
        missing = [n for n in self.names if n not in m]
        if missing:
            raise KeyError(f"関節が不足: {missing}")
        return np.array([float(m[n]) for n in self.names], dtype=np.float64)

    def clip_to_limits(self, q) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=np.float64), self.lower, self.upper)

    def within_limits(self, q, tol: float = 1e-6) -> bool:
        q = np.asarray(q)
        return bool(np.all(q >= self.lower - tol) and np.all(q <= self.upper + tol))

    def limit_step(self, q_prev, q_target, dt: float) -> np.ndarray:
        """速度制限: 1 ステップで動ける量を vel_limit*dt に抑え、関節上下限にも収める。"""
        q_prev = np.asarray(q_prev, dtype=np.float64)
        d = np.asarray(q_target, dtype=np.float64) - q_prev
        m = self.vel_limit * dt
        return self.clip_to_limits(q_prev + np.clip(d, -m, m))


@lru_cache(maxsize=4)
def load_robot_spec(path: str | None = None) -> RobotSpec:
    p = Path(path) if path else CONFIG_DIR / "robot" / "so101.yaml"
    return RobotSpec(yaml.safe_load(p.read_text(encoding="utf-8")))
