"""WSL の Gazebo ブリッジ (または mock) への同期クライアント。

1 本の TCP 接続を排他的に使う。FastAPI からは単一の SimSession 経由でしか触らない
(複数の呼び出し元が step を奪い合うと lock-step の時刻が壊れるため)。
"""
from __future__ import annotations

import os
import socket
import threading
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from . import protocol as P


class SimUnavailable(ConnectionError):
    pass


class SimCommandError(RuntimeError):
    def __init__(self, code: str, msg: str):
        super().__init__(f"[{code}] {msg}")
        self.code = code


@dataclass
class Observation:
    sim_time: float
    step: int
    q: np.ndarray           # 計測した関節角 (観測側)
    qd: np.ndarray
    q_cmd: np.ndarray       # 直前に実際に適用した指令 (クリップ後)
    image: np.ndarray       # (H, W, 3) uint8 RGB — Gazebo カメラ
    joint_stamp: float
    image_stamp: float
    task: dict[str, Any] = field(default_factory=dict)
    scene: dict[str, Any] | None = None
    step_wall_ms: float | None = None
    prof: dict[str, float] = field(default_factory=dict)   # ブリッジ内の所要時間内訳 (診断用)
    ext_joints: dict[str, float] = field(default_factory=dict)   # ドア/レバー等の関節角 [rad] (符号付き)
    objects: dict[str, Any] = field(default_factory=dict)   # 物体姿勢 {name: {pos, quat}} (表示用。成否判定には使わない)


class SimClient:
    def __init__(self, host: str | None = None, port: int | None = None, timeout: float = 30.0):
        # ポートは環境変数でも指定できる (開発用の 2 本目のブリッジ 9101 を使うため)
        self.host, self.timeout = host or os.environ.get("VRSO101_BRIDGE_HOST", "localhost"), timeout
        self.port = int(port or os.environ.get("VRSO101_BRIDGE_PORT", P.DEFAULT_PORT))
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._id = 0

    # ---- 接続 ----
    def connect(self) -> dict[str, Any]:
        try:
            s = socket.create_connection((self.host, self.port), timeout=5)
        except OSError as e:
            raise SimUnavailable(
                f"シムブリッジ {self.host}:{self.port} に接続できません ({e})。"
                "WSL で scripts/wsl/start_bridge.sh を起動してください。") from e
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(self.timeout)
        self._sock = s
        return self.call("hello")[0]

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                finally:
                    self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def call(self, op: str, timeout: float | None = None, **args) -> tuple[dict[str, Any], bytes]:
        with self._lock:
            if self._sock is None:
                raise SimUnavailable("シムに未接続です")
            self._id += 1
            try:
                if timeout is not None:
                    self._sock.settimeout(timeout)
                P.send_frame(self._sock, {"id": self._id, "op": op, **args})
                head, payload = P.recv_frame(self._sock)
            except (OSError, ConnectionError) as e:
                self._sock = None   # 壊れた接続は再利用しない (応答のずれ込みを防ぐ)
                raise SimUnavailable(f"シムとの通信が切断されました: {e}") from e
            finally:
                if self._sock is not None and timeout is not None:
                    self._sock.settimeout(self.timeout)
        if head.get("id") != self._id:
            self._sock = None
            raise SimUnavailable("応答 id が一致しません (ストリームずれ)。再接続してください")
        if not head.get("ok"):
            raise SimCommandError(head.get("code", "ERR"), head.get("error", "不明なエラー"))
        return head, payload

    # ---- 高水準 ----
    def load_task(self, task_id: str, fps: float | None = None) -> dict[str, Any]:
        return self.call("load_task", timeout=180.0, task_id=task_id, **({"fps": fps} if fps else {}))[0]

    def reset(self, seed: int = 0, randomize: bool = True) -> Observation:
        h, pl = self.call("reset", timeout=120.0, seed=seed, randomize=randomize)
        return _obs(h, pl)

    def step(self, q) -> Observation:
        h, pl = self.call("step", q=[float(v) for v in np.asarray(q)])
        return _obs(h, pl)

    def get_obs(self) -> Observation:
        h, pl = self.call("get_obs")
        return _obs(h, pl)


def _obs(h: dict[str, Any], payload: bytes) -> Observation:
    return Observation(
        sim_time=h["sim_time"], step=h["step"], q=np.array(h["q"]), qd=np.array(h["qd"]),
        q_cmd=np.array(h["q_cmd"]), image=P.image_from_payload(h["image"], payload),
        joint_stamp=h["joint_stamp"], image_stamp=h["image_stamp"], task=h.get("task", {}),
        scene=h.get("scene"), step_wall_ms=h.get("step_wall_ms"), prof=h.get("prof", {}), objects=h.get("objects", {}), ext_joints=h.get("ext_joints", {}))
