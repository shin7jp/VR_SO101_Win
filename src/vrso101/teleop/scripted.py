"""スクリプトによる Pick and Place のデモ生成 (人間のテレオペの代替ではなく、検証・E2E テスト用)。

TeleopController.track_ee() を通すので、IK・関節/速度制限・平滑化は人間のテレオペと同じ経路。
データには collection_mode="scripted" を残し、人間のデモと混ざらないようにする。

グリッパー形状 (scripts/analyze_gripper.py の実測、EE=gripper_frame_link 座標):
  固定顎の内面 x=0、動顎の内面 x=-0.016(q=0) / -0.034(q=0.4) / -0.052(q=0.8)。顎は EE の x 軸方向に開閉する。
  指先帯は z_ee∈[-0.04,0]。→ 立方体の中心を EE 座標 (-0.015, 0, -0.01) に置くと両顎が側面を挟む。

姿勢の補間について (実測した落とし穴): 5 自由度の腕は任意の姿勢には届かない。z_ee は「パン方位を含む
鉛直面内」にしか向けられない。2 つの到達可能姿勢を回転行列の slerp で補間すると途中の姿勢が到達不能になり、
姿勢を優先する IK が位置を犠牲にして停滞する。そこで姿勢を (俯角 a, 手首ロール ρ) で表し、毎周期
EE 位置の方位 φ から「届く姿勢」を組み立てる。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .controller import TeleopController

GRASP_OFFSET_EE = np.array([-0.015, 0.0, -0.01])   # 立方体中心の EE 座標


def pose_from(p: np.ndarray, a: float, rho: float) -> np.ndarray:
    """EE 位置 p (方位 φ)、アプローチ俯角 a(90°=真下)、手首ロール ρ から、到達可能な姿勢を作る。
    ρ=0 のとき顎の開閉軸 x_ee は鉛直面内 (真下アプローチでは -radial 向き = 内向き)。"""
    phi = np.arctan2(p[1], p[0])
    r_hat = np.array([np.cos(phi), np.sin(phi), 0.0])
    z = np.array([np.cos(a) * r_hat[0], np.cos(a) * r_hat[1], -np.sin(a)])
    x0 = np.array([-np.sin(a) * r_hat[0], -np.sin(a) * r_hat[1], -np.cos(a)])   # z と直交する面内ベクトル
    x = np.cos(rho) * x0 + np.sin(rho) * np.cross(z, x0)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, y, z])
    T[:3, 3] = p
    return T


def rho_for_axis(p: np.ndarray, u_xy: np.ndarray) -> float:
    """真下アプローチ (a=90°) で、顎の開閉軸を水平方向 u に向けるロール ρ。"""
    phi = np.arctan2(p[1], p[0])
    x0 = np.array([-np.cos(phi), -np.sin(phi), 0.0])            # ρ=0 の x_ee
    t0 = np.array([-np.sin(phi), np.cos(phi), 0.0])             # = z × x0 (z は真下)
    u = np.array([u_xy[0], u_xy[1], 0.0])
    return float(np.arctan2(u @ t0, u @ x0))


def current_a_rho(T: np.ndarray) -> tuple[float, float]:
    """現在の EE 姿勢から (俯角 a, ロール ρ) を逆算する (pose_from の逆)。"""
    z, x = T[:3, 2], T[:3, 0]
    a = float(np.arcsin(np.clip(-z[2], -1, 1)))
    phi = np.arctan2(T[1, 3], T[0, 3])
    r_hat = np.array([np.cos(phi), np.sin(phi), 0.0])
    x0 = np.array([-np.sin(a) * r_hat[0], -np.sin(a) * r_hat[1], -np.cos(a)])
    return a, float(np.arctan2(x @ np.cross(z, x0), x @ x0))


def solve_ik_multistart(kin, T: np.ndarray, q0, *, tries: int = 60, seed: int = 0):
    """ランダム初期値つき IK (局所解対策)。収束解のうち q0 に最も近いものを返す。無ければ None。"""
    rng = np.random.default_rng(seed)
    spec = kin.spec
    best = None
    for k in range(tries):
        start = np.asarray(q0, dtype=np.float64) if k == 0 else rng.uniform(spec.lower, spec.upper) * 0.8
        r = kin.ik(T, start, rot_weight=1.0, max_iter=150, rot_tol=0.06, pos_tol=0.003)
        if r.converged:
            d = float(np.linalg.norm(r.q[:5] - np.asarray(q0)[:5]))
            if best is None or d < best[0]:
                best = (d, r)
            if k == 0:
                break
    return best[1] if best else None


def choose_closing_axis(cube_xy: np.ndarray, cube_yaw: float) -> np.ndarray:
    """立方体の面に直交する 4 候補のうち、「内向きの動径方向 (-radial)」に最も近いもの。
    +radial 側は ρ≈±π が要り手首ロールの関節上限 (-2.74..2.84) を超えるので IK が解けない (実測)。"""
    inward = -cube_xy / np.linalg.norm(cube_xy)
    cands = [np.array([np.cos(cube_yaw + k * np.pi / 2), np.sin(cube_yaw + k * np.pi / 2)]) for k in range(4)]
    return max(cands, key=lambda c: float(c @ inward))


def _smooth(s: float) -> float:   # min-jerk
    return 10 * s**3 - 15 * s**4 + 6 * s**5


@dataclass
class Segment:
    name: str
    p: np.ndarray         # 目標 EE 位置
    gripper: float        # 0=閉 1=開 (TeleopController の正規化)
    ticks: int


class ScriptedPickPlace:
    OPEN, CLOSED = 0.45, 0.0     # 0.45*(1.745+0.175)-0.175 ≈ 0.69rad → 顎の隙間 ~0.045m (立方体 0.03m に余裕)

    def __init__(self, cube_xyz, cube_yaw: float, target_xy, *, speed: float = 1.0, hover: float = 0.05):
        cube = np.asarray(cube_xyz, dtype=np.float64)
        self.u = choose_closing_axis(cube[:2], cube_yaw)
        # オフセットは「真下アプローチ + 軸 u」で固定のワールドベクトルとして扱う (運搬中も不変)
        x = np.array([self.u[0], self.u[1], 0.0])
        R = np.column_stack([x, np.cross([0, 0, -1.0], x), [0, 0, -1.0]])
        off = R @ GRASP_OFFSET_EE
        p_grasp = cube - off
        p_tgt = np.array([target_xy[0], target_xy[1], cube[2]]) - off
        up = np.array([0, 0, hover])
        t = lambda n: max(int(n / speed), 2)  # noqa: E731
        self.segments = [
            Segment("pregrasp", p_grasp + up, self.OPEN, t(40)),
            Segment("descend", p_grasp, self.OPEN, t(25)),
            Segment("close", p_grasp, self.CLOSED, t(20)),
            Segment("lift", p_grasp + up, self.CLOSED, t(25)),
            Segment("transport", p_tgt + up, self.CLOSED, t(40)),
            Segment("lower", p_tgt + np.array([0, 0, 0.004]), self.CLOSED, t(25)),
            Segment("release", p_tgt + np.array([0, 0, 0.004]), self.OPEN, t(15)),
            Segment("retreat", p_tgt + up, self.OPEN, t(25)),
        ]
        self.total_ticks = sum(s.ticks for s in self.segments)

    def run(self, ctl: TeleopController):
        """1 周期ごとに (フェーズ名, TeleopOutput) を yield する。呼び出し側が step して次を要求する。"""
        T0 = ctl.kin.fk(ctl.q)
        p_prev = T0[:3, 3].copy()
        # 開始姿勢の俯角 a を現在の EE 姿勢から逆算 (ホームでは a≈34°, ρ≈0)
        a_prev = float(np.arcsin(np.clip(-T0[2, 2], -1, 1)))
        rho_prev = 0.0
        g_prev = float(np.clip((ctl.q[-1] - ctl.spec.lower[-1]) / (ctl.spec.upper[-1] - ctl.spec.lower[-1]), 0, 1))
        for i, seg in enumerate(self.segments):
            a_end = np.pi / 2
            rho_end = rho_for_axis(seg.p, self.u)
            for k in range(1, seg.ticks + 1):
                s = _smooth(k / seg.ticks)
                p = p_prev + (seg.p - p_prev) * s
                if i == 0:                      # 最初の区間だけ (a, ρ) を補間。以後は真下 + 軸 u 固定
                    a, rho = a_prev + (a_end - a_prev) * s, rho_prev + (rho_end - rho_prev) * s
                else:
                    a, rho = a_end, rho_for_axis(p, self.u)
                g = g_prev + (seg.gripper - g_prev) * s
                yield seg.name, ctl.track_ee(pose_from(p, a, rho), g)
            p_prev, g_prev = seg.p, seg.gripper
