"""ACT 学習 (LeRobot 公式 lerobot-train を固定 argv で起動)。UI/CLI 共通。

外部送信なし: push_to_hub=false / wandb 無効 / HF_HUB_OFFLINE=1 (jobs.manager.child_env)。
ResNet18 の ImageNet 事前学習重みだけは torchvision が初回に取得する (公開重みのダウンロードのみ・送信なし)。
"""
from __future__ import annotations

import json
import re
import sys
import time
from pathlib import Path
from typing import Any

from pydantic import BaseModel, Field, field_validator

from ..jobs.manager import JobManager, child_env
from ..paths import data_dir
from ..recording.store import atomic_write_json, now_iso
from .. import __version__
from ..models.registry import META, model_dir

_NUM = r"([0-9.]+[KMB]?)"
LOG_RE = re.compile(rf"step:{_NUM}\s+smpl:{_NUM}.*?loss:([0-9.eE+-]+)\s+grdn:([0-9.eE+-]+)\s+lr:([0-9.eE+-]+)")
L1_RE = re.compile(r"l1_loss:([0-9.eE+-]+)")
KLD_RE = re.compile(r"kld_loss:([0-9.eE+-]+)")


def _num(s: str) -> float:
    mult = {"K": 1e3, "M": 1e6, "B": 1e9}.get(s[-1], 1.0)
    return float(s.rstrip("KMB")) * mult


TQDM_RE = re.compile(r"Training:\s+\d+%\|[^|]*\|\s*(\d+)/(\d+)")


def parse_train_line(line: str, job: dict) -> None:
    t = TQDM_RE.search(line)
    if t:   # lerobot のログは step を "6K" のように丸めるので、正確なステップは tqdm の進捗行から取る
        job["progress"]["step"] = int(t.group(1))
        job["progress"]["total"] = int(t.group(2))
        return
    m = LOG_RE.search(line)
    if not m:
        return
    step = max(int(round(_num(m.group(1)))), job["progress"].get("step", 0)) if m.group(1)[-1] in "KMB" else int(round(_num(m.group(1))))
    rec = {"step": step, "loss": float(m.group(3)), "grad_norm": float(m.group(4)), "lr": float(m.group(5))}
    if (a := L1_RE.search(line)):
        rec["l1_loss"] = float(a.group(1))
    if (a := KLD_RE.search(line)):
        rec["kld_loss"] = float(a.group(1))
    job["progress"]["step"] = max(job["progress"].get("step", 0), step)
    rec["step"] = job["progress"]["step"]
    job["metrics"].append(rec)
    if len(job["metrics"]) > 5000:   # job.json が肥大しないよう間引き
        job["metrics"] = job["metrics"][::2]


class TrainConfig(BaseModel):
    task_id: str
    dataset: str                               # data/datasets/<task_id>/<dataset>
    steps: int = Field(2000, ge=1, le=1_000_000)
    batch_size: int = Field(8, ge=1, le=256)
    device: str = "cuda"
    save_freq: int = Field(1000, ge=1)
    chunk_size: int = Field(50, ge=1, le=200)
    num_workers: int = Field(2, ge=0, le=16)
    seed: int = 1000
    lr: float | None = None
    name: str | None = None

    @field_validator("device")
    @classmethod
    def _dev(cls, v):
        if not re.fullmatch(r"cpu|mps|cuda(:\d+)?", v):
            raise ValueError("device は cpu / mps / cuda[:N]")
        return v

    @field_validator("name")
    @classmethod
    def _name(cls, v):
        if v is not None and not re.fullmatch(r"[A-Za-z0-9_-]{1,40}", v):
            raise ValueError("name は英数字と _- のみ (40 文字まで)")
        return v


def available_devices() -> dict[str, Any]:
    """実際に使えるデバイスを検証して返す (CUDA が無いのに cuda を選べないように)。"""
    import torch
    devs = {"cpu": True, "cuda": torch.cuda.is_available(),
            "mps": bool(getattr(torch.backends, "mps", None) and torch.backends.mps.is_available())}
    info = {"devices": devs, "torch": torch.__version__}
    if devs["cuda"]:
        info["cuda_name"] = torch.cuda.get_device_name(0)
    return info


def prepare_run(cfg: TrainConfig) -> dict[str, Any]:
    """検証 → モデルディレクトリ作成 → argv を返す。失敗は ValueError (具体的な復旧メッセージ付き)。"""
    from ..dataset.lerobot_export import validate_dataset
    from ..robot.joints import load_robot_spec

    root = data_dir() / "datasets" / cfg.task_id / cfg.dataset
    if not root.exists():
        raise ValueError(f"データセットが見つかりません: {root}。先に変換 (export) してください")
    res = validate_dataset(root, load_robot_spec(), check_decode=True)
    if not res["ok"]:
        raise ValueError("データセット検証に失敗: " + " / ".join(res["problems"]))
    n_ep = res["num_episodes"]
    if n_ep < 2:
        raise ValueError(f"エピソードが少なすぎます ({n_ep})。ACT の学習には最低でも数十エピソードを推奨します")
    dev = cfg.device
    if dev.startswith("cuda") or dev == "mps":
        info = available_devices()
        if not info["devices"]["cuda" if dev.startswith("cuda") else "mps"]:
            raise ValueError(f"デバイス {dev} はこの環境で使えません (利用可能: {info['devices']})")
    model_id = f"{cfg.task_id}-{time.strftime('%Y%m%d-%H%M%S')}" + (f"-{cfg.name}" if cfg.name else "")
    mdir = model_dir(cfg.task_id, model_id)
    mdir.mkdir(parents=True)
    out = mdir / "lerobot"   # lerobot は output_dir が存在すると拒否するので、親だけ作る
    argv = [sys.executable, "-m", "vrso101.training.launcher",
            f"--dataset.repo_id=local/{cfg.task_id}_{cfg.dataset}", f"--dataset.root={root}",
            "--dataset.video_backend=pyav", "--policy.type=act", f"--policy.device={dev}",
            "--policy.push_to_hub=false", f"--policy.chunk_size={cfg.chunk_size}",
            f"--policy.n_action_steps={cfg.chunk_size}", f"--output_dir={out}", f"--job_name={model_id}",
            f"--steps={cfg.steps}", f"--batch_size={cfg.batch_size}", f"--save_freq={cfg.save_freq}",
            "--log_freq=10", "--wandb.enable=false", f"--num_workers={cfg.num_workers}", f"--seed={cfg.seed}"]
    if cfg.lr is not None:
        argv.append(f"--policy.optimizer_lr={cfg.lr}")
    vm = res["meta"]
    meta = {"model_id": model_id, "task_id": cfg.task_id, "status": "training", "created_at": now_iso(),
            "generator_version": __version__, "lerobot_version": vm.get("lerobot_version"),
            "train_config": cfg.model_dump(), "dataset": {"name": cfg.dataset, "root": str(root), "repo_id": f"local/{cfg.task_id}_{cfg.dataset}",
                                                         "num_episodes": n_ep, "num_frames": res["num_frames"],
                                                         "seeds": vm.get("seeds", []), "collection_modes": vm.get("collection_modes"),
                                                         "excluded_count": vm.get("num_skipped")},
            "robot": {"id": vm["robot_id"], "joint_names": vm["joint_names"], "units": vm["units"],
                      "joint_limits": vm["joint_limits"]},
            "fps": vm["fps"], "camera": vm["camera"], "features": vm["features"], "result": None}
    atomic_write_json(mdir / META, meta)
    return {"model_id": model_id, "model_dir": str(mdir), "argv": argv, "meta": meta}


def finalize_model(task_id: str, model_id: str, job: dict) -> None:
    from ..models.registry import list_checkpoints, update_model_meta
    cks = list_checkpoints(model_dir(task_id, model_id))
    metrics = job.get("metrics") or []
    status = {"succeeded": "trained", "stopped": "stopped"}.get(job["state"], "failed")
    if status == "trained" and not any(c["ok"] for c in cks):
        status = "failed"
    update_model_meta(task_id, model_id, status=status, ended_at=now_iso(), job_id=job["id"], result={
        "state": job["state"], "error": job.get("error"), "last_step": job["progress"].get("step"),
        "final_loss": metrics[-1]["loss"] if metrics else None,
        "min_loss": min((m["loss"] for m in metrics), default=None),
        "loss_history": metrics[:: max(1, len(metrics) // 200)], "checkpoints": [c["name"] for c in cks]})


def start_training(jm: JobManager, cfg: TrainConfig) -> dict[str, Any]:
    key = "gpu:train"   # 学習は同時に 1 つ (GPU 競合と連打による二重起動を防ぐ)
    active = jm.active(key)
    if active:
        return {**active, "already_running": True}
    run = prepare_run(cfg)
    job = jm.start("train", run["argv"], key=key, params={"model_id": run["model_id"], **cfg.model_dump()},
                   parse_line=parse_train_line, total=cfg.steps,
                   on_finish=lambda j: finalize_model(cfg.task_id, run["model_id"], j))
    return {**job, "model_id": run["model_id"]}


def run_training_foreground(task: str, dataset: str, *, steps: int, batch_size: int, device: str, save_freq: int,
                            chunk_size: int, name: str | None, num_workers: int) -> int:
    """CLI 用: ジョブマネージャ経由で起動し、ログを流しながら完了を待つ (UI と同じ経路を使う)。"""
    jm = JobManager()
    cfg = TrainConfig(task_id=task, dataset=dataset, steps=steps, batch_size=batch_size, device=device,
                      save_freq=save_freq, chunk_size=chunk_size, num_workers=num_workers, name=name)
    job = start_training(jm, cfg)
    print(f"job={job['id']} model={job.get('model_id')} pid={job['pid']}", flush=True)
    last = 0
    while True:
        j = jm.get(job["id"])
        if j["progress"]["step"] != last:
            last = j["progress"]["step"]
            m = j["metrics"][-1] if j["metrics"] else {}
            print(f"  step {last}/{cfg.steps} loss={m.get('loss')}", flush=True)
        if j["state"] in ("succeeded", "failed", "stopped"):
            break
        time.sleep(2)
    j = jm.get(job["id"])
    print(f"状態: {j['state']} {j.get('error') or ''}")
    if j["state"] != "succeeded":
        print(jm.tail(job["id"], 30))
    return 0 if j["state"] == "succeeded" else 1
