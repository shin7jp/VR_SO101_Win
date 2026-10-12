"""長時間ジョブ (学習・推論) の管理。HTTP リクエストを待たせず、ジョブ ID で状態を引く。

- ジョブは固定の argv (サーバー側で組み立てたもの) だけを起動する。ブラウザから任意コマンドは渡せない。
- 状態は data/jobs/<id>/job.json に永続化 → ブラウザを閉じても、バックエンドを再起動しても追える。
  再起動後、まだ生きているプロセスはログ追跡を再開し、居なければ「中断」にする。
- 同じ key の実行中ジョブがあれば新規起動せず既存ジョブを返す (連打・二重起動の防止 = 冪等)。
- 停止は taskkill /PID /T /F (GPU を掴むのは孫プロセスなので、親だけ terminate しても解放されない)。
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Callable

import psutil

from ..paths import data_dir
from ..recording.store import atomic_write_json, now_iso

TERMINAL = ("succeeded", "failed", "stopped")


def child_env(extra: dict[str, str] | None = None) -> dict[str, str]:
    env = dict(os.environ)
    env.update({"PYTHONIOENCODING": "utf-8", "PYTHONUTF8": "1", "PYTHONUNBUFFERED": "1",   # E:\claude 共通ルール
                # 外部送信・自動公開をしない既定: Hub 通信と W&B を止める
                "HF_HUB_OFFLINE": "1", "HF_HUB_DISABLE_TELEMETRY": "1", "WANDB_MODE": "disabled",
                "DO_NOT_TRACK": "1", "TQDM_MININTERVAL": "2"})
    env.update(extra or {})
    return env


def kill_tree(pid: int) -> None:
    if sys.platform == "win32":
        subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True)
    else:   # pragma: no cover
        try:
            p = psutil.Process(pid)
            for c in p.children(recursive=True):
                c.kill()
            p.kill()
        except psutil.NoSuchProcess:
            pass


class JobManager:
    def __init__(self, root: Path | None = None, notify: Callable[[dict], None] | None = None):
        self.root = Path(root) if root else data_dir() / "jobs"
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._procs: dict[str, subprocess.Popen] = {}
        self._parsers: dict[str, Callable[[str, dict], None]] = {}
        self._finish: dict[str, Callable[[dict], None]] = {}
        self.notify = notify or (lambda ev: None)
        self._recover()

    # ---- 永続化 ----
    def _path(self, job_id: str) -> Path:
        if not re.fullmatch(r"[A-Za-z0-9_-]+", job_id):
            raise ValueError("不正なジョブ ID")
        return self.root / job_id

    def get(self, job_id: str) -> dict[str, Any]:
        f = self._path(job_id) / "job.json"
        for i in range(40):   # 書き込み側の replace と衝突したら少し待って読み直す
            try:
                return json.loads(f.read_text(encoding="utf-8"))
            except (PermissionError, json.JSONDecodeError):
                if i == 39:
                    raise
                time.sleep(0.025)

    def _save(self, job: dict) -> None:
        atomic_write_json(self._path(job["id"]) / "job.json", job)
        self.notify({"type": "job", "job_id": job["id"], "state": job["state"], "progress": job.get("progress"),
                     "kind": job["kind"]})

    def list(self, kind: str | None = None) -> list[dict]:
        out = []
        for d in sorted(self.root.iterdir(), reverse=True):
            f = d / "job.json"
            if f.exists():
                try:
                    j = json.loads(f.read_text(encoding="utf-8"))
                except json.JSONDecodeError:
                    continue
                if kind is None or j.get("kind") == kind:
                    out.append(j)
        return out

    def active(self, key: str) -> dict | None:
        for j in self.list():
            if j.get("key") == key and j["state"] in ("queued", "running"):
                return j
        return None

    # ---- 起動 ----
    def start(self, kind: str, argv: list[str], *, key: str, params: dict | None = None,
              parse_line: Callable[[str, dict], None] | None = None, env: dict | None = None,
              cwd: str | None = None, total: int | None = None,
              on_finish: Callable[[dict], None] | None = None) -> dict[str, Any]:
        with self._lock:
            existing = self.active(key)
            if existing:
                return {**existing, "already_running": True}
            job_id = f"{kind}-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:5]}"
            d = self._path(job_id)
            d.mkdir(parents=True)
            log = d / "job.log"
            job = {"id": job_id, "kind": kind, "key": key, "state": "running", "params": params or {},
                   "created_at": now_iso(), "started_at": now_iso(), "ended_at": None, "pid": None,
                   "progress": {"step": 0, "total": total}, "metrics": [], "log": str(log), "error": None,
                   "result": None, "argv": argv}
            flags = 0
            if sys.platform == "win32":
                # 新しいプロセスグループ + 独立: バックエンドの Ctrl-C で道連れにしない (再起動後も追えるように)
                flags = subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW
            logf = open(log, "ab")
            try:
                p = subprocess.Popen(argv, env=child_env(env), cwd=cwd, stdout=logf, stderr=subprocess.STDOUT,
                                     creationflags=flags, stdin=subprocess.DEVNULL)
            except OSError as e:
                job.update(state="failed", ended_at=now_iso(), error=f"起動に失敗: {e}")
                self._save(job)
                return job
            finally:
                logf.close()
            job["pid"] = p.pid
            self._procs[job_id] = p
            if parse_line:
                self._parsers[job_id] = parse_line
            if on_finish:
                self._finish[job_id] = on_finish
            self._save(job)
        threading.Thread(target=self._watch, args=(job_id, log, 0), daemon=True, name=f"job-{job_id}").start()
        return job

    # ---- 監視 (ログ追跡 → 進捗/メトリクス更新 → 終了判定) ----
    def _alive(self, job_id: str, pid: int | None) -> bool:
        p = self._procs.get(job_id)
        if p is not None:
            return p.poll() is None
        return bool(pid) and psutil.pid_exists(pid) and self._is_ours(pid)

    @staticmethod
    def _is_ours(pid: int) -> bool:
        try:
            return psutil.Process(pid).status() != psutil.STATUS_ZOMBIE
        except psutil.NoSuchProcess:
            return False

    def _watch(self, job_id: str, log: Path, offset: int) -> None:
        parse = self._parsers.get(job_id)
        buf = ""
        last_save = 0.0
        while True:
            job = self.get(job_id)
            if job["state"] in TERMINAL:
                return
            alive = self._alive(job_id, job["pid"])
            try:
                with open(log, "rb") as f:
                    f.seek(offset)
                    chunk = f.read()
                    offset += len(chunk)
            except FileNotFoundError:
                chunk = b""
            if chunk:
                buf += chunk.decode("utf-8", errors="replace").replace("\r", "\n")
                *lines, buf = buf.split("\n")
                if parse:
                    for line in lines:
                        try:
                            parse(line, job)
                        except Exception:
                            pass
                if time.monotonic() - last_save > 1.0 or not alive:
                    self._save(job)
                    last_save = time.monotonic()
            if not alive:
                time.sleep(0.2)   # 最後の出力を取りこぼさないよう一度だけ追加読み
                try:
                    with open(log, "rb") as f:
                        f.seek(offset)
                        tail = f.read().decode("utf-8", errors="replace").replace("\r", "\n")
                    if parse:
                        for line in (buf + tail).split("\n"):
                            parse(line, job)
                except Exception:
                    pass
                code = self._procs[job_id].returncode if job_id in self._procs else None
                job = self.get_fresh(job_id, job)
                if job["state"] not in TERMINAL:
                    ok = (code == 0) if code is not None else self._finished_ok(job)
                    stopped = bool((job.get("result") or {}).get("stopped"))   # 利用者の停止要求で正常終了した (動画は保存済み)
                    job.update(state="stopped" if (ok and stopped) else "succeeded" if ok else "failed", ended_at=now_iso(), exit_code=code)
                    if not ok:
                        job["error"] = job.get("error") or f"プロセスが異常終了しました (exit={code})。ログ末尾を確認してください"
                self._save(job)
                cb = self._finish.pop(job_id, None)
                if cb:
                    try:
                        cb(job)
                    except Exception as e:   # 後始末の失敗でジョブ状態を壊さない
                        job["error"] = (job.get("error") or "") + f" [finish hook: {e}]"
                        self._save(job)
                return
            time.sleep(0.5)

    def get_fresh(self, job_id: str, local: dict) -> dict:
        disk = self.get(job_id)
        if disk["state"] in TERMINAL:   # stop() が先に確定していたらそれを尊重
            return disk
        return local

    @staticmethod
    def _finished_ok(job: dict) -> bool:
        return bool(job.get("result", {}) and job["result"].get("ok"))

    # ---- 停止 ----
    def stop(self, job_id: str) -> dict:
        with self._lock:
            job = self.get(job_id)
            if job["state"] in TERMINAL:
                return job   # 冪等
            job.update(state="stopped", ended_at=now_iso(), error="利用者が停止")
            self._save(job)
            if job.get("pid"):
                kill_tree(job["pid"])
        cb = self._finish.pop(job_id, None)
        if cb:
            try:
                cb(job)
            except Exception:
                pass
        return job

    # ---- 再起動後の復元 ----
    def _recover(self) -> None:
        for j in self.list():
            if j["state"] not in ("queued", "running"):
                continue
            if j.get("pid") and psutil.pid_exists(j["pid"]) and self._is_ours(j["pid"]):
                threading.Thread(target=self._watch, args=(j["id"], Path(j["log"]), 0), daemon=True).start()
            else:
                j.update(state="failed", ended_at=now_iso(),
                         error="バックエンド再起動前にプロセスが終了していました (中断)")
                self._save(j)

    def tail(self, job_id: str, lines: int = 80) -> str:
        try:
            txt = Path(self.get(job_id)["log"]).read_text(encoding="utf-8", errors="replace").replace("\r", "\n")
        except FileNotFoundError:
            return ""
        return "\n".join([l for l in txt.split("\n") if l.strip()][-lines:])
