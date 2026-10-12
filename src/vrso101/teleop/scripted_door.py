"""スクリプトによる Open Door / Rotating Lever Door のデモ (検証・E2E 用。TeleopController の安全経路を通す)。

ドア座標系: 原点=ヒンジ軸と作業台面の交点、+y=自由端方向、-x=ロボット側の面 (寸法は tasks/door_geometry.py)。
ドアが角度 θ だけ開いたとき、ドア座標の点 (lx, ly) のワールド位置は H + Rz(θ)·(lx, ly)。

「押して開ける」デモにしている理由 (実測した経緯):
  縦棒/ペグを顎で挟んだまま蝶番の円弧に沿って引くと、EE の追従誤差が蝶番の拘束と喧嘩して接触が破綻した
  (1 反復 80ms まで遅くなり、顎が逆に開かれる)。押すだけなら拘束が片側 (一方向の接触) になり安定する。
  ヒンジは両方向に回れ、成功判定は |角度| なので、押しても引いても成功になる (人間のテレオペは引いてもよい)。

  open_door : 閉じた顎のブロックで自由端付近のロボット側の面を押す
  lever_door: ① 閉じた顎の先端でレバーを押し下げる (摩擦で下がったまま留まる) → ② ドアを押して開ける
"""
from __future__ import annotations

from typing import Iterator

import numpy as np

from ..tasks.door_geometry import door_geometry
from .controller import TeleopController
from .scripted import _smooth, pose_from, rho_for_axis, solve_ik_multistart

# 閉じた顎ブロックの EE 座標での範囲 (scripts/analyze_gripper.py の実測): y_ee ∈ [-0.028, 0.024]。
# 顎の開閉軸 x_ee をドア面に沿う向きに取ると、+y_ee 側の平らな側面 (剛体な固定顎側) がドア面法線を向く。
# 動顎の面 (x_ee 側) で押すと、接触力で動顎が開かれ、ドアが逆に回った (実測)。
JAW_FRONT = 0.026     # +y_ee 側の張り出し


def _rz(th: float) -> np.ndarray:
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s], [s, c]])


class ScriptedDoor:
    def __init__(self, task: dict, scene: dict, *, speed: float = 1.0, theta_end: float = -0.7, hover: float = 0.035):
        self.kind = task["success"]["type"]
        if self.kind not in ("open_door", "lever_door"):
            raise NotImplementedError(self.kind)
        self.g = door_geometry(task)
        self.H = np.array([scene["door"]["hinge_x"], scene["door"]["hinge_y"]])
        self.theta_end = theta_end          # 負 = ロボットから遠ざかる向き (押す)
        self.hover = hover
        self.t = lambda n: max(int(n / speed), 2)  # noqa: E731
        # 閉じた顎の側面 (y_ee 正側) でドア面を押す: 顎の開閉軸はドア面に沿う向き (+y) で、ドアと一緒に回す
        self.u_local = np.array([0.0, 1.0])
        self.r_g = 0.015                      # レバーを押す位置 (スピンドルからの距離)

    def _ee(self, local_xy, z: float, theta: float) -> np.ndarray:
        """ドア座標の点 local_xy にブロック前面が来る EE 位置 (u はドアと一緒に回す)。"""
        xy = self.H + _rz(theta) @ np.asarray(local_xy)
        u = _rz(theta) @ self.u_local          # x_ee (水平)
        nrm = _rz(theta) @ np.array([1.0, 0.0])   # ドア面の法線 (ロボットから遠ざかる向き) = +y_ee
        p = np.array([xy[0] - JAW_FRONT * nrm[0], xy[1] - JAW_FRONT * nrm[1], z])
        return p, u

    def _push_local(self) -> np.ndarray:
        g = self.g
        return np.array([-g["t"] / 2 - 0.004, g["w"] / 2 - 0.005])   # 面から 4mm 手前、パネル中央 (ペグ / レバーの y 範囲と重ならない)

    def run(self, ctl: TeleopController) -> Iterator[tuple[str, object]]:
        g = self.g
        q0 = ctl.q.copy()
        CLOSED = 0.0
        g0 = float(np.clip((ctl.q[-1] - ctl.spec.lower[-1]) / (ctl.spec.upper[-1] - ctl.spec.lower[-1]), 0, 1))

        def joint_move(name: str, q_goal: np.ndarray, n: int, grip: float):
            q_from = ctl.q.copy()
            gg0 = float(np.clip((q_from[-1] - ctl.spec.lower[-1]) / (ctl.spec.upper[-1] - ctl.spec.lower[-1]), 0, 1))
            for k in range(1, n + 1):
                s = _smooth(k / n)
                yield name, ctl.track_joints(q_from + (q_goal - q_from) * s, gg0 + (grip - gg0) * s)

        def cart_move(name: str, p0, p1, rho0, rho1, n: int, grip: float):
            for k in range(1, n + 1):
                s = _smooth(k / n)
                p = p0 + (p1 - p0) * s
                yield name, ctl.track_ee(pose_from(p, np.pi / 2, rho0 + (rho1 - rho0) * s), grip)

        def solve(p, u):
            T = pose_from(p, np.pi / 2, rho_for_axis(p, u))
            ik = solve_ik_multistart(ctl.kin, T, ctl.q)
            if ik is None:
                raise RuntimeError("到達できない姿勢です (ドア配置が到達範囲外)")
            return ik

        if self.kind == "lever_door":
            # ① レバーを押し下げる: 先端付近の真上から、閉じた顎の先端 (フラットな下面) で押す
            sy, sz, lx, r = g["spindle_y"], g["spindle_z"], g["lever_x"], g["lever_r"]
            top0 = sz + r                                           # アーム上面の高さ (水平時)
            def press_point(phi):   # φ だけ下がったアーム上面の押す位置 (ドア座標の x,y と z)
                return np.array([lx, sy + self.r_g * np.cos(phi)]), top0 - self.r_g * np.sin(phi) + 0.002
            phi_end = 0.85
            xy, zt = press_point(0.0)
            p_hover, u = self._ee_at(xy, zt + 0.015)   # 真下アプローチは高さ 0.1 付近で届かないので、アーム上面の 15mm 上で待つ
            yield from joint_move("approach_lever", solve(p_hover, u).q, self.t(55), CLOSED)
            n = self.t(40)
            for k in range(1, n + 1):          # 押し下げ: 先端がアーム上面に沿って円弧を下る
                s = _smooth(k / n)
                phi = phi_end * s
                xy, zt = press_point(phi)
                z = zt + 0.015 * (1 - min(1.0, s * 3))   # 最初の 1/3 でアーム上面まで降り、残りで φ に沿って押す
                p, u = self._ee_at(xy, z)
                yield "press_lever", ctl.track_ee(pose_from(p, np.pi / 2, rho_for_axis(p, u)), CLOSED)
            xy, zt = press_point(phi_end)
            p_last, u = self._ee_at(xy, zt)
            p_up = p_last + np.array([0, 0, 0.02])
            yield from cart_move("release_lever", p_last, p_up, rho_for_axis(p_last, u), rho_for_axis(p_up, u), self.t(25), CLOSED)
            z_push = 0.07                                          # 押す高さ: 下がったレバーアームの上を通す
        else:
            z_push = 0.065
        # ② ドアを押して開ける: ドア面の手前へ関節空間で回り込み → 円弧に沿って押す
        loc = self._push_local()
        p_start, u0 = self._ee(loc, z_push, 0.0)
        p_pre = p_start + np.array([-0.03, 0.0, 0.0])   # 面から 3cm 手前で待つ
        yield from joint_move("approach_door", solve(p_pre, u0).q, self.t(55), CLOSED)
        rho_a = rho_for_axis(p_pre, u0)
        rho_b = rho_for_axis(p_start, u0)
        yield from cart_move("contact", p_pre, p_start, rho_a, rho_b, self.t(20), CLOSED)
        n = self.t(80)
        for k in range(1, n + 1):
            s = _smooth(k / n)
            th = self.theta_end * s
            p, u = self._ee(loc, z_push, th)
            yield "push", ctl.track_ee(pose_from(p, np.pi / 2, rho_for_axis(p, u)), CLOSED)
        p_end, _ = self._ee(loc, z_push, self.theta_end)
        for k in range(1, self.t(20) + 1):    # 静止して余韻を見る (ドアは慣性でもう少し開く)
            yield "settle", ctl.track_ee(pose_from(p_end, np.pi / 2, rho_for_axis(p_end, u)), CLOSED)

    def _ee_at(self, local_xy, z: float):
        """ブロック前面ではなく、ドア座標の点の真上に EE (顎の中心) を置く (レバー押しで使う)。"""
        xy = self.H + local_xy
        u = self.u_local
        p = np.array([xy[0] + 0.012 * u[0], xy[1] + 0.012 * u[1], z])
        return p, u
