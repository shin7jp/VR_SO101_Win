"""学習済みモデルの保管と検証。

  data/models/<task_id>/<model_id>/
      vrso101_model.json      データセット ID・学習条件・結果・特徴量/単位 (重みだけを置かない)
      lerobot/checkpoints/<step>/pretrained_model/{config.json, model.safetensors, policy_*processor*.json, train_config.json}
      lerobot/checkpoints/last                (ディレクトリジャンクション)
      job.log

正規化統計は pretrained_model 内の processor (safetensors) に入っており、推論時は同じ processor を読み込む。
"""
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

from ..paths import data_dir
from ..recording.store import atomic_write_json

META = "vrso101_model.json"


def models_root() -> Path:
    return data_dir() / "models"


def model_dir(task_id: str, model_id: str) -> Path:
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", model_id) or ".." in task_id or "/" in task_id or "\\" in task_id:
        raise ValueError("不正な ID")
    return models_root() / task_id / model_id


def list_checkpoints(mdir: Path) -> list[dict[str, Any]]:
    ck = mdir / "lerobot" / "checkpoints"
    out = []
    if ck.exists():
        for d in sorted(p for p in ck.iterdir() if p.is_dir() and p.name.isdigit()):
            pm = d / "pretrained_model"
            out.append({"name": d.name, "step": int(d.name), "path": str(pm), "ok": verify_checkpoint(pm)[0],
                        "size_mb": round(sum(f.stat().st_size for f in pm.rglob("*") if f.is_file()) / 1e6, 1)
                        if pm.exists() else 0})
    return out


def resolve_checkpoint(mdir: Path, which: str = "last") -> Path:
    cks = list_checkpoints(mdir)
    if not cks:
        raise FileNotFoundError(f"チェックポイントがありません: {mdir}")
    if which == "last":
        return Path(cks[-1]["path"])
    for c in cks:
        if c["name"] == which or c["name"].lstrip("0") == which.lstrip("0"):
            return Path(c["path"])
    raise FileNotFoundError(f"チェックポイント {which} が見つかりません (有効: {[c['name'] for c in cks]})")


def verify_checkpoint(pm: Path) -> tuple[bool, str]:
    """壊れたチェックポイント (書き込み途中・容量不足・手動削除) を推論前に弾く。"""
    need = ["config.json", "model.safetensors", "policy_preprocessor.json", "policy_postprocessor.json"]
    for f in need:
        if not (pm / f).exists():
            return False, f"{f} がありません"
    try:
        json.loads((pm / "config.json").read_text(encoding="utf-8"))
        from safetensors import safe_open
        with safe_open(str(pm / "model.safetensors"), framework="pt") as f:
            keys = list(f.keys())
        if not keys:
            return False, "model.safetensors にテンソルがありません"
    except Exception as e:
        return False, f"破損: {type(e).__name__}: {e}"
    return True, "ok"


def list_models(task_id: str | None = None) -> list[dict[str, Any]]:
    out = []
    root = models_root()
    if not root.exists():
        return out
    for tdir in sorted(p for p in root.iterdir() if p.is_dir()):
        if task_id and tdir.name != task_id:
            continue
        for mdir in sorted((p for p in tdir.iterdir() if p.is_dir()), reverse=True):
            f = mdir / META
            if not f.exists():
                continue
            try:
                m = json.loads(f.read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                continue
            m["checkpoints"] = list_checkpoints(mdir)
            out.append(m)
    return out


def load_model_meta(task_id: str, model_id: str) -> dict[str, Any]:
    d = model_dir(task_id, model_id)
    m = json.loads((d / META).read_text(encoding="utf-8"))
    m["checkpoints"] = list_checkpoints(d)
    return m


def update_model_meta(task_id: str, model_id: str, **fields) -> dict[str, Any]:
    d = model_dir(task_id, model_id)
    m = json.loads((d / META).read_text(encoding="utf-8"))
    m.update(fields)
    atomic_write_json(d / META, m)
    return m
