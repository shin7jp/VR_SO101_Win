"""環境診断: 何が動いていて何が動かないかを具体的な復旧手順つきで返す (Settings 画面 / vrso101 doctor)。"""
from __future__ import annotations

import shutil
import socket
import sys
from pathlib import Path

from ..paths import ROOT, data_dir
from ..sim import protocol as P


def check(name: str, ok: bool, detail: str, fix: str | None = None, level: str | None = None) -> dict:
    return {"name": name, "ok": ok, "level": level or ("ok" if ok else "error"), "detail": detail, "fix": fix}


def run_diagnostics(host: str = "localhost", port: int = P.DEFAULT_PORT) -> dict:
    items = [check("Python", sys.version_info >= (3, 12), sys.version.split()[0], "Python 3.12 の venv を作り直す (SETUP.md)")]
    try:
        import torch
        cuda = torch.cuda.is_available()
        items.append(check("PyTorch", True, f"{torch.__version__}"))
        items.append(check("CUDA", cuda, torch.cuda.get_device_name(0) if cuda else "利用不可 (学習は CPU になり非常に遅い)",
                           "NVIDIA ドライバ更新 / CUDA 版 torch を入れ直す (SETUP.md)", None if cuda else "warning"))
    except Exception as e:
        items.append(check("PyTorch", False, f"{type(e).__name__}: {e}", "SETUP.md の torch 導入手順"))
    try:
        import lerobot
        items.append(check("LeRobot", True, lerobot.__version__))
    except Exception as e:
        items.append(check("LeRobot", False, str(e), "uv pip install lerobot[training]==0.6.1"))
    try:
        import av
        items.append(check("PyAV (動画)", True, av.__version__))
    except Exception as e:
        items.append(check("PyAV (動画)", False, str(e), "uv pip install av"))
    ffmpeg = shutil.which("ffmpeg")
    items.append(check("ffmpeg (任意)", bool(ffmpeg), ffmpeg or "PATH に無い (PyAV 同梱で動くので必須ではない)", None,
                       "ok" if ffmpeg else "info"))
    d = data_dir()
    try:
        d.mkdir(parents=True, exist_ok=True)
        t = d / ".write_test"
        t.write_text("x")
        t.unlink()
        free = shutil.disk_usage(d).free / 1e9
        items.append(check("データ保存先", True, f"{d} (空き {free:.0f} GB)", None, "ok" if free > 10 else "warning"))
    except Exception as e:
        items.append(check("データ保存先", False, f"{d}: {e}", "VRSO101_DATA_DIR を書き込める場所に設定"))
    # Gazebo ブリッジ (WSL) 到達性
    try:
        s = socket.create_connection((host, port), timeout=2)
        s.close()
        items.append(check("Gazebo ブリッジ (WSL)", True, f"{host}:{port} に接続できる"))
    except OSError as e:
        items.append(check("Gazebo ブリッジ (WSL)", False, f"{host}:{port} に接続できない ({e})",
                           "powershell scripts\\bridge.ps1 start (WSL に ROS 2 + Gazebo が必要: SETUP.md)"))
    items.append(check("SO-101 モデル", (ROOT / "sim" / "models" / "so101" / "model.sdf").exists(),
                       "sim/models/so101/model.sdf",
                       "python sim/tools/build_model.py で生成"))
    return {"ok": all(i["ok"] or i["level"] in ("warning", "info") for i in items), "items": items}
