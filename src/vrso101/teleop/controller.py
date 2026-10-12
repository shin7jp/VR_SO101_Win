"""テレオペ制御: 入力 (VR/PC/タッチ) → EE 目標 → IK → 制限・平滑化 → 関節目標値。

入力源が違っても IK・制限・平滑化・安全停止は 1 か所を通す (VR だけ安全で PC は素通り、を避ける)。
VR は「クラッチ方式」: デッドマン (グリップ) を握った瞬間のコントローラ/EE 姿勢を基準にし、
以降は相対移動だけを反映する。ロボットが突然コントローラ位置へ飛ぶのを防ぐため。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from ..robot.joints import RobotSpec
from ..robot.kinematics import IKResult, Kinematics
from .transform import FrameConfig, xr_position_to_robot, xr_rotation_to_robot


class Status(str, Enum):
    IDLE = "idle"                    # デッドマン未押下 (関節目標は保持)
    TRACKING = "tracking"
    IK_FAILED = "ik_failed"          # 解なし。直前の目標を保持
    WORKSPACE_CLAMPED = "workspace_clamped"
    STALE = "stale"                  # 入力途絶 (通信切断) → 停止


@dataclass
class Workspace:
    """EE 目標位置の許容範囲 [m] (ロボット base_link 座標)。作業台や自己干渉の目安。"""
    lower: np.ndarray = field(default_factory=lambda: np.array([0.05, -0.35, 0.0]))
    upper: np.ndarray = field(default_factory=lambda: np.array([0.40, 0.35, 0.40]))

    def clamp(self, p: np.ndarray) -> tuple[np.ndarray, bool]:
        c = np.clip(p, self.lower, self.upper)
        return c, bool(np.any(c != p))


@dataclass
class TeleopOutput:
    q_target: np.ndarray
    status: Status
    ik: IKResult | None = None
    ee_target: np.ndarray | None = None


class TeleopController:
    def __init__(self, spec: RobotSpec, kin: Kinematics, *, rate_hz: float = 30.0,
                 frame: FrameConfig | None = None, workspace: Workspace | None = None,
                 smoothing: float = 0.5, stale_timeout_s: float = 0.5, clock=time.monotonic):
        self.spec, self.kin = spec, kin
        self.dt = 1.0 / rate_hz
        self.frame = frame or FrameConfig()
        self.workspace = workspace or Workspace()
        self.smoothing = float(np.clip(smoothing, 0.0, 0.95))   # 大きいほど滑らか(遅れる)
        self.stale_timeout_s = stale_timeout_s
        self._clock = clock
        self.ik_rot_weight = 0.3                                  # スクリプトデモは姿勢を厳密に (1.0) 使う
        self.pos_priority = False                                 # True: 姿勢が実現不能なら位置優先で解き直す (人手操作用)
        self.speed = 1.0                                          # 動作速度倍率 (vel_limit に掛ける)
        self.q = spec.home.copy()                                 # 現在の指令関節 (ここから速度制限を掛ける)
        self.status = Status.IDLE
        self._anchor: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None
        self._last_input = clock()
        self._ee_target = kin.fk(self.q)

    # ---- 状態管理 ----
    def reset(self, q: np.ndarray | None = None) -> None:
        self.q = (self.spec.home if q is None else np.asarray(q, dtype=np.float64)).copy()
        self._ee_target = self.kin.fk(self.q)
        self._anchor = None
        self.status = Status.IDLE

    def set_speed(self, s: float) -> None:
        self.speed = float(np.clip(s, 0.1, 2.0))

    def release(self) -> None:
        """デッドマン解除: 即座に追従を止め、現在の指令値を保持する。"""
        self._anchor = None
        self.status = Status.IDLE

    def check_stale(self) -> bool:
        """入力が途絶えていたら追従をやめる。呼び出し側 (制御ループ) が毎周期呼ぶ。"""
        if self._anchor is not None and self._clock() - self._last_input > self.stale_timeout_s:
            self._anchor = None
            self.status = Status.STALE
            return True
        return False

    # ---- 入力: VR コントローラ ----
    def update_vr(self, pos_xr, quat_xr, *, deadman: bool, gripper: float | None = None) -> TeleopOutput:
        self._last_input = self._clock()
        if not deadman:
            self.release()
            return self._emit(None, gripper)
        p_r = xr_position_to_robot(pos_xr, self.frame)
        R_r = xr_rotation_to_robot(quat_xr, self.frame)
        if self._anchor is None:   # 握った瞬間を基準にする
            T = self.kin.fk(self.q)
            self._anchor = (p_r, R_r, T[:3, 3].copy(), T[:3, :3].copy())
        p0, R0, pe0, Re0 = self._anchor
        target = np.eye(4)
        target[:3, 3] = pe0 + (p_r - p0)
        # コントローラの回転変化量をクラッチ時の EE 姿勢へ左から掛ける
        target[:3, :3] = (R_r @ R0.T) @ Re0 if self.frame.rot_scale == 1.0 else _scale_rot(R_r @ R0.T, self.frame.rot_scale) @ Re0
        return self._emit(target, gripper)

    # ---- 入力: PC / タッチ (EE を相対移動) ----
    def nudge(self, dpos, drot=(0.0, 0.0, 0.0), gripper: float | None = None) -> TeleopOutput:
        self._last_input = self._clock()
        target = self._ee_target.copy()
        target[:3, 3] += np.asarray(dpos, dtype=np.float64)
        if np.any(np.asarray(drot) != 0):
            from ..robot.kinematics import axis_angle
            w = np.asarray(drot, dtype=np.float64)
            target[:3, :3] = axis_angle(w, float(np.linalg.norm(w))) @ target[:3, :3]
        self._anchor = None
        return self._emit(target, gripper)

    def move_toward(self, pos, max_step: float, gripper: float | None = None) -> TeleopOutput:
        """EE の目標位置 (絶対, ロボット座標) へ 1 周期あたり max_step [m] 以内で近づく。マウスドラッグ追従用。
        目標が遠くても一気には動かさない (速度の上限は max_step / dt)。姿勢は nudge と同じく保つ。"""
        d = np.asarray(pos, dtype=np.float64) - self._ee_target[:3, 3]
        n = float(np.linalg.norm(d))
        if n > max_step > 0:
            d = d * (max_step / n)
        return self.nudge(d, gripper=gripper)

    def hold(self, gripper: float | None = None) -> TeleopOutput:
        """入力なし: 現在の指令を保持する (グリッパーだけは更新できる)。"""
        return self._emit(None, gripper)

    # ---- 入力: EE 目標をそのまま指定 (スクリプトデモ用。VR/PC と同じ安全経路を通す) ----
    def track_ee(self, target: np.ndarray, gripper: float | None = None) -> TeleopOutput:
        self._last_input = self._clock()
        self._anchor = None
        return self._emit(target, gripper)

    # ---- 共通: IK → 制限 → 平滑化 ----
    def _emit(self, target: np.ndarray | None, gripper: float | None) -> TeleopOutput:
        q_goal = self.q.copy()
        ik_res = None
        status = Status.IDLE
        if target is not None:
            pos, clamped = self.workspace.clamp(target[:3, 3])
            target = target.copy()
            target[:3, 3] = pos
            ik_res = self.kin.ik(target, self.q, max_iter=60, rot_weight=self.ik_rot_weight)
            # 位置が許容内に収まる解だけ採用。失敗時は直前の指令を保持 (飛ばさない)
            if not (ik_res.converged or ik_res.pos_err < 5e-3) and self.pos_priority:
                # 5 自由度では姿勢を保ったまま位置を動かせないことがある。目標姿勢が実現不能になると位置キーまで
                # 全部 IK 失敗で固まる (人手の PC 操作で実測) ので、位置優先で解き直し、目標姿勢を実現できた姿勢へ寄せる。
                ik2 = self.kin.ik(target, self.q, max_iter=60, rot_weight=0.02)
                if ik2.converged or ik2.pos_err < 5e-3:
                    target = target.copy()
                    target[:3, :3] = self.kin.fk(ik2.q)[:3, :3]
                    ik_res = ik2
            if ik_res.converged or ik_res.pos_err < 5e-3:
                q_goal = ik_res.q
                self._ee_target = target
                status = Status.WORKSPACE_CLAMPED if clamped else Status.TRACKING
            else:
                status = Status.IK_FAILED
                # 解なしでも、現在姿勢より目標に近づく最良近似なら採用する。「失敗したら保持」だけだと、
                # 目標が先へ進んだ後に局所解から出られず固まる (スクリプトデモで実測)。
                e0 = float(np.linalg.norm(target[:3, 3] - self.kin.fk(self.q)[:3, 3]))
                if ik_res.pos_err < e0 - 1e-4:
                    q_goal = ik_res.q
        return self._finish(q_goal, gripper, status, ik_res)

    def track_joints(self, q_goal, gripper: float | None = None) -> TeleopOutput:
        """関節目標を直接指定 (スクリプトデモの大きな再構成用)。IK は通さないが、平滑化・速度/関節制限は同じ経路。"""
        self._last_input = self._clock()
        self._anchor = None
        q_goal = self.spec.clip_to_limits(np.asarray(q_goal, dtype=np.float64))
        self._ee_target = self.kin.fk(q_goal)
        return self._finish(q_goal.copy(), gripper, Status.TRACKING, None)

    def _finish(self, q_goal: np.ndarray, gripper: float | None, status: Status, ik_res: IKResult | None) -> TeleopOutput:
        if gripper is not None:
            g = self.spec.lower[-1] + float(np.clip(gripper, 0.0, 1.0)) * (self.spec.upper[-1] - self.spec.lower[-1])
            q_goal[-1] = g
        # 平滑化 → 速度制限 (制限は平滑化後に掛けるので、必ず制限を超えない)
        q_s = self.smoothing * self.q + (1.0 - self.smoothing) * q_goal
        q_new = self.spec.limit_step(self.q, q_s, self.dt * self.speed)
        if not np.all(np.isfinite(q_new)):  # NaN を絶対に出さない
            q_new, status = self.q.copy(), Status.IK_FAILED
        self.q = q_new
        self.status = status
        return TeleopOutput(self.q.copy(), status, ik_res, self._ee_target.copy())


def _scale_rot(R: np.ndarray, s: float) -> np.ndarray:
    from ..robot.kinematics import axis_angle, rot_log
    w = rot_log(R) * s
    n = float(np.linalg.norm(w))
    return np.eye(3) if n < 1e-12 else axis_angle(w, n)
