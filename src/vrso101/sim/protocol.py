"""Windows(FastAPI/ACT) ↔ WSL(Gazebo sim bridge) のワイヤプロトコル。

標準ライブラリ + numpy だけで書く: WSL 側は ROS 2 の system python で動かすため、
Windows の venv の依存 (pydantic 等) を持ち込めない。このファイルは両側から import される。

フレーム = [u32 header_len][header JSON utf-8][u32 payload_len][payload bytes]
  リクエスト: header = {"id": n, "op": "...", ...args}, payload は通常空
  レスポンス: header = {"id": n, "ok": bool, ...}, 画像があれば payload に RGB8 生バイト
サーバーは WSL 側に置く (Windows→WSL の localhost 転送は通るが、逆は通らないため)。
"""
from __future__ import annotations

import json
import socket
import struct
from typing import Any

import numpy as np

DEFAULT_PORT = 9100
MAX_HEADER = 1 << 20
MAX_PAYLOAD = 64 << 20

OPS = ("hello", "load_task", "reset", "step", "get_obs", "task_state", "shutdown")


class ProtocolError(RuntimeError):
    pass


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("接続が閉じられました")
        buf += chunk
    return bytes(buf)


def send_frame(sock: socket.socket, header: dict[str, Any], payload: bytes = b"") -> None:
    h = json.dumps(header, separators=(",", ":")).encode("utf-8")
    sock.sendall(struct.pack(">I", len(h)) + h + struct.pack(">I", len(payload)) + payload)


def recv_frame(sock: socket.socket) -> tuple[dict[str, Any], bytes]:
    (hl,) = struct.unpack(">I", _recv_exact(sock, 4))
    if hl > MAX_HEADER:
        raise ProtocolError(f"header が大きすぎます: {hl}")
    header = json.loads(_recv_exact(sock, hl).decode("utf-8"))
    (pl,) = struct.unpack(">I", _recv_exact(sock, 4))
    if pl > MAX_PAYLOAD:
        raise ProtocolError(f"payload が大きすぎます: {pl}")
    return header, (_recv_exact(sock, pl) if pl else b"")


def image_from_payload(meta: dict[str, Any], payload: bytes) -> np.ndarray:
    """meta = {"w","h","fmt":"rgb8"} → (h, w, 3) uint8。サイズ不一致は壊れたフレームとして弾く。"""
    if meta.get("fmt") != "rgb8":
        raise ProtocolError(f"未対応の画像形式: {meta.get('fmt')}")
    w, h = int(meta["w"]), int(meta["h"])
    if len(payload) != w * h * 3:
        raise ProtocolError(f"画像サイズ不一致: {len(payload)} != {w}*{h}*3")
    return np.frombuffer(payload, dtype=np.uint8).reshape(h, w, 3)
