"""WebXR コントローラ座標 → ロボット座標の変換。

WebXR (local-floor) は右手系・y 上・-z が前方・単位 m。ロボット (ROS/URDF) は右手系・x 前・y 左・z 上。
どちらも右手系なので「軸の入れ替え」だけで済むが、オペレータがロボットの横や後ろから見て
操作したいことがあるので、yaw オフセット・スケール・原点オフセットを設定可能にしてある。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# WebXR(x右, y上, z後) → ROS(x前, y左, z上):  x_r = -z_xr, y_r = -x_xr, z_r = y_xr
R_XR_TO_ROS = np.array([[0.0, 0.0, -1.0],
                        [-1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]])


def quat_to_matrix(q) -> np.ndarray:
    """q = (x, y, z, w) (WebXR の順)。正規化してから変換する。"""
    x, y, z, w = np.asarray(q, dtype=np.float64)
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        raise ValueError("ゼロクォータニオン")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_matrix(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


@dataclass
class FrameConfig:
    scale: float = 1.0                       # コントローラ移動量 → ロボット移動量
    yaw_offset: float = 0.0                  # [rad] オペレータの向きの補正 (キャリブレーションで設定)
    origin_offset: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [m] ロボット座標への加算
    rot_scale: float = 1.0                   # 1.0 でコントローラの回転をそのまま反映

    def __post_init__(self):
        self.origin_offset = np.asarray(self.origin_offset, dtype=np.float64)
        if not (0.05 <= self.scale <= 5.0):
            raise ValueError("scale は 0.05〜5.0 の範囲で指定してください")

    @property
    def R(self) -> np.ndarray:
        """WebXR ベクトルをロボット座標へ移す回転 (yaw 補正込み)。"""
        return yaw_matrix(self.yaw_offset) @ R_XR_TO_ROS


def xr_position_to_robot(p_xr, cfg: FrameConfig) -> np.ndarray:
    return cfg.scale * (cfg.R @ np.asarray(p_xr, dtype=np.float64)) + cfg.origin_offset


def xr_rotation_to_robot(q_xr, cfg: FrameConfig) -> np.ndarray:
    """コントローラ姿勢 (WebXR) → ロボット座標系での姿勢行列 (基底変換: R_map · R · R_mapᵀ)。"""
    Rm = cfg.R
    return Rm @ quat_to_matrix(q_xr) @ Rm.T


def calibrate_yaw_from_forward(forward_xr) -> float:
    """「ここがロボットの前」とオペレータが指した方向 (WebXR のベクトル, y 成分は無視) から yaw を決める。
    指した方向がロボットの +x に来るように回す。"""
    f = np.asarray(forward_xr, dtype=np.float64)
    v = R_XR_TO_ROS @ f
    if np.hypot(v[0], v[1]) < 1e-3:
        raise ValueError("水平方向の成分が小さすぎてキャリブレーションできません")
    return float(-np.arctan2(v[1], v[0]))
