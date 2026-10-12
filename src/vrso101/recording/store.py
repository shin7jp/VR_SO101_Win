"""生エピソードの永続化 (ローカルファイル)。差し替え可能なストレージの最小インターフェースを兼ねる。

  data/episodes/<task_id>/<episode_id>/
      meta.json     episode_id / task / seed / scene / fps / status / label / excluded / quality ...
      states.npz    sim_time, q (観測), q_cmd (実際に適用した指令), qd, image_stamp, joint_stamp, step_wall_ms
      frames.mkv    Gazebo カメラフレーム (FFV1 可逆)
      preview.mp4   閲覧用 H.264

meta.status: recording → (success | failure | aborted)  自動判定 (シム物理) とは別に label (人手評価) を持つ。
"""
from __future__ import annotations

import json
import os
import shutil
import tempfile
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from ..paths import data_dir

STATUSES = ("recording", "success", "failure", "aborted")
LABELS = (None, "success", "failure", "aborted")


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def new_episode_id(task_id: str) -> str:
    # 日時 + 短い乱数: 並行/連続起動でもファイル名が重複しない
    return f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:6]}"


def atomic_write_json(path: Path, obj: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(obj, f, ensure_ascii=False, indent=2)
        # 途中で落ちても meta.json が壊れた状態にならない。Windows は読み手が開いている間 replace が
        # PermissionError になるので短くリトライする (API の GET と監視スレッドの書き込みが衝突する)
        for i in range(40):
            try:
                os.replace(tmp, path)
                break
            except PermissionError:
                if i == 39:
                    raise
                time.sleep(0.025)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


class EpisodeStore:
    def __init__(self, root: Path | None = None):
        self.root = Path(root) if root else data_dir() / "episodes"

    def dir(self, task_id: str, episode_id: str) -> Path:
        if not episode_id.replace("-", "").isalnum() or ".." in task_id or "/" in task_id or "\\" in task_id:
            raise ValueError("不正な ID")   # パストラバーサル対策 (API から ID を受けるため)
        return self.root / task_id / episode_id

    def create(self, task_id: str, meta: dict[str, Any]) -> tuple[str, Path]:
        eid = new_episode_id(task_id)
        d = self.dir(task_id, eid)
        d.mkdir(parents=True, exist_ok=False)
        meta = {"episode_id": eid, "task_id": task_id, "status": "recording", "label": None, "excluded": False,
                "started_at": now_iso(), **meta}
        atomic_write_json(d / "meta.json", meta)
        return eid, d

    def load_meta(self, task_id: str, episode_id: str) -> dict[str, Any]:
        return json.loads((self.dir(task_id, episode_id) / "meta.json").read_text(encoding="utf-8"))

    def update_meta(self, task_id: str, episode_id: str, **fields) -> dict[str, Any]:
        m = self.load_meta(task_id, episode_id)
        m.update(fields)
        atomic_write_json(self.dir(task_id, episode_id) / "meta.json", m)
        return m

    def list(self, task_id: str | None = None) -> list[dict[str, Any]]:
        out = []
        if not self.root.exists():
            return out
        for tdir in sorted(p for p in self.root.iterdir() if p.is_dir()):
            if task_id and tdir.name != task_id:
                continue
            for edir in sorted(p for p in tdir.iterdir() if p.is_dir()):
                f = edir / "meta.json"
                if not f.exists():
                    continue
                try:
                    m = json.loads(f.read_text(encoding="utf-8"))
                except json.JSONDecodeError:
                    m = {"episode_id": edir.name, "task_id": tdir.name, "status": "corrupt", "excluded": True}
                # 再起動前に録画中のまま落ちたものは「中断」として見せる (消さない)
                if m.get("status") == "recording" and not (edir / ".active").exists():
                    m["status"] = "aborted"
                    m.setdefault("abort_reason", "プロセス終了により中断")
                out.append(m)
        return out

    def load_arrays(self, task_id: str, episode_id: str) -> dict[str, np.ndarray]:
        with np.load(self.dir(task_id, episode_id) / "states.npz") as z:
            return {k: z[k] for k in z.files}

    def delete(self, task_id: str, episode_id: str) -> None:
        shutil.rmtree(self.dir(task_id, episode_id), ignore_errors=False)

    def set_excluded(self, task_id: str, episode_id: str, excluded: bool) -> dict[str, Any]:
        return self.update_meta(task_id, episode_id, excluded=bool(excluded))

    def set_label(self, task_id: str, episode_id: str, label: str | None) -> dict[str, Any]:
        if label not in LABELS:
            raise ValueError(f"label は {LABELS} のいずれか")
        return self.update_meta(task_id, episode_id, label=label)
