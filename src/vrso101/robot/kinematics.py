"""URDF からの順/逆運動学 (numpy のみ)。

IK ソルバーを自前で持つ理由: SO-101 は 5 自由度 + グリッパーで、LeRobot 側の placo は
Windows ネイティブでの導入が重く依存も増える。SO-101 は単純な直列鎖なので、URDF から
チェーンを組んで減衰最小二乗 (DLS) で解く程度で足りる。ただし 5 自由度なので 6D 姿勢は
一般に達成できない → 位置を最優先し、姿勢は重みを下げて「できる範囲で」合わせる。
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

from ..paths import resolve_project_path
from .joints import RobotSpec


def rpy_to_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle(axis: np.ndarray, th: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def rot_log(R: np.ndarray) -> np.ndarray:
    """回転行列 → 回転ベクトル (姿勢誤差の計算用)。"""
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    if np.pi - th < 1e-5:  # π 近傍は sin(th)≈0 で割れないので対称部から軸を取る
        w = np.sqrt(np.maximum((np.diag(R) + 1) / 2, 0))
        k = int(np.argmax(w))
        for i in range(3):
            if i != k:
                w[i] = np.copysign(w[i], R[k, i] + R[i, k])
        return th * w / np.linalg.norm(w)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return th / (2 * np.sin(th)) * v


@dataclass
class _Link:
    T0: np.ndarray            # 親フレームから見た関節原点
    axis: np.ndarray | None   # 固定関節は None
    jindex: int | None        # 関節ベクトル中の位置


@dataclass
class IKResult:
    q: np.ndarray
    converged: bool
    pos_err: float
    rot_err: float
    iterations: int


class Kinematics:
    def __init__(self, spec: RobotSpec):
        self.spec = spec
        root = ET.parse(resolve_project_path(spec.urdf)).getroot()
        joints = {}
        self.urdf_joint_limits: dict[str, tuple[float, float]] = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz", "0 0 0").split()]) if o is not None else np.zeros(3)
            rpy = [float(v) for v in o.get("rpy", "0 0 0").split()] if o is not None else [0.0, 0.0, 0.0]
            T = np.eye(4)
            T[:3, :3] = rpy_to_matrix(*rpy)
            T[:3, 3] = xyz
            ax = j.find("axis")
            lim = j.find("limit")
            if j.get("type") == "revolute" and lim is not None:
                self.urdf_joint_limits[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
            joints[j.find("child").get("link")] = dict(
                name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"), T=T,
                axis=np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else None)
        chain, link = [], spec.end_effector_link
        while link != spec.base_link:
            if link not in joints:
                raise ValueError(f"URDF に {spec.base_link} へ至る鎖がありません ({link})")
            chain.append(joints[link])
            link = joints[link]["parent"]
        chain.reverse()
        self._chain: list[_Link] = []
        for c in chain:
            if c["type"] == "fixed":
                self._chain.append(_Link(c["T"], None, None))
            else:
                self._chain.append(_Link(c["T"], c["axis"], spec.index(c["name"])))
        self.n_chain_joints = sum(1 for c in self._chain if c.jindex is not None)

    def fk(self, q) -> np.ndarray:
        """4x4 同次変換 (base_link → end_effector_link)。"""
        q = np.asarray(q, dtype=np.float64)
        T = np.eye(4)
        for L in self._chain:
            T = T @ L.T0
            if L.jindex is not None:
                R = np.eye(4)
                R[:3, :3] = axis_angle(L.axis, q[L.jindex])
                T = T @ R
        return T

    def jacobian(self, q) -> np.ndarray:
        """6 x dof の幾何ヤコビアン (上 3 行: 位置, 下 3 行: 角速度)。EE 鎖に無い関節の列は 0。"""
        q = np.asarray(q, dtype=np.float64)
        T = np.eye(4)
        cols = []
        for L in self._chain:
            T = T @ L.T0
            if L.jindex is not None:
                cols.append((T[:3, 3].copy(), T[:3, :3] @ (L.axis / np.linalg.norm(L.axis)), L.jindex))
                R = np.eye(4)
                R[:3, :3] = axis_angle(L.axis, q[L.jindex])
                T = T @ R
        p_e = T[:3, 3]
        J = np.zeros((6, self.spec.dof))
        for o, z, i in cols:
            J[:3, i] = np.cross(z, p_e - o)
            J[3:, i] = z
        return J

    def ik(self, target: np.ndarray, q0, *, rot_weight: float = 0.3, max_iter: int = 100,
           damping: float = 0.05, pos_tol: float = 2e-3, rot_tol: float = 0.35,
           max_step: float = 0.15) -> IKResult:
        """target: 4x4。収束 = 位置が pos_tol 以内 かつ 姿勢誤差が rot_tol 以内。
        収束しなくても関節上下限内の最良近似 q を返す (呼び出し側が converged を見て採否を決める)。"""
        q = self.spec.clip_to_limits(q0).copy()
        n = self.n_chain_joints  # 鎖の関節は先頭 n 個 (SO-101 はグリッパーが末尾)
        W = np.diag([1, 1, 1, rot_weight, rot_weight, rot_weight])
        pe = re = float("inf")
        for it in range(1, max_iter + 1):
            T = self.fk(q)
            ep = target[:3, 3] - T[:3, 3]
            er = rot_log(target[:3, :3] @ T[:3, :3].T)
            pe, re = float(np.linalg.norm(ep)), float(np.linalg.norm(er))
            if pe < pos_tol and re < rot_tol:
                return IKResult(q, True, pe, re, it)
            err = W @ np.concatenate([ep, er])
            J = W @ self.jacobian(q)[:, :n]
            dq = J.T @ np.linalg.solve(J @ J.T + (damping ** 2) * np.eye(6), err)
            nrm = np.linalg.norm(dq)
            if nrm > max_step:
                dq *= max_step / nrm
            q[:n] = np.clip(q[:n] + dq, self.spec.lower[:n], self.spec.upper[:n])
        return IKResult(q, False, pe, re, max_iter)
