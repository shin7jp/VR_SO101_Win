"""ACT 推論ランタイム: 学習済みチェックポイントを読み込み、Gazebo(または mock) の SO-101 を制御する。

LeRobot 公式の processor を pretrained_model から読み込むので、正規化・画像前処理は学習時と同一。
自前で正規化を再実装しない (統計値の食い違いが一番起きやすい事故のため)。

制御 1 周期 = 観測取得 → 前処理 → (キューが空なら) アクションチャンク推論 → 後処理 → 安全制限 → step。
チャンクを丸ごと盲目的に実行しない: n_action_steps ごとに再推論する (policy.config.n_action_steps)。
"""
from __future__ import annotations

import json
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .. import __version__
from ..models.registry import load_model_meta, model_dir, resolve_checkpoint, verify_checkpoint
from ..paths import data_dir
from ..recording.store import atomic_write_json, now_iso
from ..robot.joints import RobotSpec
from ..sim.client import Observation, SimUnavailable
from ..tasks.registry import get_task
from ..video import VideoWriter, probe_video

IMAGE_KEY = "observation.images.front"
SEED_EVAL_MIN = 10000   # 学習用シード (< 10000) と評価用シードを分離する規約


class PolicyLoadError(RuntimeError):
    pass


class ACTRunner:
    def __init__(self, ckpt: Path, spec: RobotSpec, *, device: str = "cuda", n_action_steps: int = 10):
        import torch
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors

        ok, why = verify_checkpoint(ckpt)
        if not ok:
            raise PolicyLoadError(f"チェックポイントが壊れています: {ckpt} ({why})。別のチェックポイントを選ぶか再学習してください")
        if device.startswith("cuda") and not torch.cuda.is_available():
            raise PolicyLoadError("CUDA が使えません。device=cpu を指定するか GPU ドライバを確認してください")
        self.torch, self.device, self.spec = torch, device, spec
        try:
            self.policy = ACTPolicy.from_pretrained(str(ckpt))
        except Exception as e:
            raise PolicyLoadError(f"ACT の読み込みに失敗: {type(e).__name__}: {e}") from e
        cfg = self.policy.config
        if n_action_steps < 1 or n_action_steps > cfg.chunk_size:
            raise PolicyLoadError(f"n_action_steps は 1〜chunk_size({cfg.chunk_size}) の範囲で指定してください")
        cfg.n_action_steps = n_action_steps
        cfg.temporal_ensemble_coeff = None   # 時間アンサンブルと n_action_steps は排他
        self.policy.to(device).eval()
        self.pre, self.post = make_pre_post_processors(
            cfg, pretrained_path=str(ckpt), preprocessor_overrides={"device_processor": {"device": device}})
        self._check_features(cfg)
        self.chunk_size, self.n_action_steps = cfg.chunk_size, n_action_steps
        self.infer_calls = 0
        self.infer_ms: list[float] = []

    def _check_features(self, cfg) -> None:
        feats = cfg.input_features
        if IMAGE_KEY not in feats or "observation.state" not in feats:
            raise PolicyLoadError(f"ポリシーの入力特徴量が想定と違います: {list(feats)}")
        if tuple(feats["observation.state"].shape) != (self.spec.dof,):
            raise PolicyLoadError(f"state 次元 {feats['observation.state'].shape} が関節数 {self.spec.dof} と一致しません")
        if tuple(cfg.output_features["action"].shape) != (self.spec.dof,):
            raise PolicyLoadError("action 次元が関節数と一致しません")
        self.image_shape = tuple(feats[IMAGE_KEY].shape)   # (C, H, W)

    def reset(self) -> None:
        self.policy.reset()

    def act(self, obs: Observation) -> np.ndarray:
        torch = self.torch
        img = obs.image
        c, h, w = self.image_shape
        if img.shape != (h, w, 3):
            raise PolicyLoadError(f"観測画像 {img.shape} が学習時 {(h, w, 3)} と一致しません (解像度/カメラ設定を確認)")
        # 学習データと同じ: RGB, HWC uint8 → CHW float32 [0,1] (LeRobot の video デコードと同じ規約)
        frame = torch.from_numpy(np.array(img, copy=True)).permute(2, 0, 1).float().div_(255.0)
        batch = {IMAGE_KEY: frame, "observation.state": torch.from_numpy(obs.q.astype(np.float32))}
        t0 = time.perf_counter()
        with torch.inference_mode():
            batch = self.pre(batch)
            action = self.policy.select_action(batch)
            action = self.post(action)
        a = action.detach().float().cpu().numpy().reshape(-1)
        self.infer_ms.append((time.perf_counter() - t0) * 1000)
        if a.shape != (self.spec.dof,):
            raise PolicyLoadError(f"推論出力の形状が不正: {a.shape}")
        return a.astype(np.float64)


def _inference_dir(task_id: str, episode_id: str) -> Path:
    return data_dir() / "inference" / task_id / episode_id


def run_inference_episode(sim, spec: RobotSpec, runner: ACTRunner, task_id: str, model_id: str, ckpt_name: str,
                          seed: int, *, max_steps: int | None = None, speed_scale: float = 1.0,
                          stop: threading.Event | None = None, on_step: Callable[[Observation, int], None] | None = None,
                          max_step_latency_s: float = 5.0, randomize: bool = True) -> dict[str, Any]:
    """1 エピソードを実行して動画 (MP4) とメタデータ (JSON) を保存する。保存を検証できて初めて saved=True。"""
    task = get_task(task_id)
    fps = float(task["fps"])
    max_steps = int(max_steps or task["max_steps"])
    dt = 1.0 / fps
    episode_id = f"{datetime.now().strftime('%Y%m%d-%H%M%S')}-s{seed}-{uuid.uuid4().hex[:5]}"
    out = _inference_dir(task_id, episode_id)
    out.mkdir(parents=True, exist_ok=True)
    started = now_iso()
    t_wall0 = time.perf_counter()
    reason, steps, vw, obs = "running", 0, None, None
    walls: list[float] = []
    mmeta = {}
    try:
        mmeta = load_model_meta(task_id, model_id)
    except FileNotFoundError:
        pass
    try:
        obs = sim.reset(seed=seed, randomize=randomize)
        runner.reset()
        h, w = obs.image.shape[:2]
        vw = VideoWriter(out / "inference.mp4", fps, w, h, codec="libx264", crf=20,
                         container_opts={"movflags": "+faststart"})
        vw.write(obs.image)
        q_prev = obs.q.copy()
        while True:
            if stop is not None and stop.is_set():
                reason = "user_stop"
                break
            if obs.task.get("success"):
                reason = "success"
                break
            if steps >= max_steps:
                reason = "timeout"
                break
            a = runner.act(obs)
            if not np.all(np.isfinite(a)):
                reason = "invalid_action"   # NaN/inf を絶対にシムへ送らない
                break
            # 安全制限: 速度制限 (vel_limit × speed_scale) と関節上下限。指令の基準は直前の指令 (実測ではなく)
            q_cmd = spec.limit_step(q_prev if steps else obs.q, a, dt * speed_scale)
            t0 = time.perf_counter()
            obs = sim.step(q_cmd)
            lat = time.perf_counter() - t0
            walls.append(lat * 1000)
            if lat > max_step_latency_s:
                reason = "control_delay"
                break
            q_prev = q_cmd
            steps += 1
            vw.write(obs.image)
            if on_step:
                on_step(obs, steps)
        else:  # pragma: no cover
            pass
    except SimUnavailable as e:
        reason = f"sim_disconnected: {e}"
    except PolicyLoadError as e:
        reason = f"policy_error: {e}"
    except Exception as e:
        reason = f"error: {type(e).__name__}: {e}"
    finally:
        if vw is not None:
            try:
                vw.close()
            except Exception as e:   # pragma: no cover
                reason = f"video_close_failed: {e}"
    success = bool(obs is not None and obs.task.get("success"))
    video_info, video_err = None, None
    try:
        video_info = probe_video(out / "inference.mp4")   # 実際にデコードして確認してから『保存済み』にする
        if vw is not None and video_info["frames"] != vw.frames:
            video_err = f"フレーム数不一致: 書込 {vw.frames} / 読出 {video_info['frames']}"
    except Exception as e:
        video_err = f"動画の保存/検証に失敗: {type(e).__name__}: {e}"
    wall = time.perf_counter() - t_wall0
    train_seeds = set(mmeta.get("dataset", {}).get("seeds", []))
    meta = {
        "episode_id": episode_id, "task_id": task_id, "task_name": task["name"], "saved": video_err is None,
        "save_error": video_err, "success": success,
        # outcome: success | timeout | stopped (利用者の停止) | abnormal (異常停止: 通信切断・無効な指令・制御遅延・例外)
        "outcome": ("success" if success else "timeout" if reason == "timeout" else "stopped" if reason == "user_stop" else "abnormal"),
        "reason": reason, "policy": {"model_id": model_id, "checkpoint": ckpt_name,
                                      "n_action_steps": runner.n_action_steps, "chunk_size": runner.chunk_size,
                                      "device": runner.device, "speed_scale": speed_scale},
        "dataset": mmeta.get("dataset", {}).get("repo_id"), "dataset_name": mmeta.get("dataset", {}).get("name"),
        "started_at": started, "ended_at": now_iso(), "wall_seconds": round(wall, 3),
        "sim_seconds": round(steps * dt, 3), "frames": video_info["frames"] if video_info else 0,
        "control_steps": steps, "inference_fps": round(steps / wall, 2) if wall > 0 else None,
        "policy_inference_ms": {"count": len(runner.infer_ms),
                                "mean": round(float(np.mean(runner.infer_ms)), 2) if runner.infer_ms else None,
                                "max": round(float(np.max(runner.infer_ms)), 2) if runner.infer_ms else None},
        "step_ms": {"median": round(float(np.median(walls)), 2) if walls else None,
                    "max": round(float(np.max(walls)), 2) if walls else None},
        "sim": {"backend": getattr(sim, "backend", "gazebo"), "fps": fps, "max_steps": max_steps,
                "camera": task["camera"], "randomize": randomize},
        "seed": seed, "scene": getattr(obs, "scene", None) if obs else None,
        "task_result": obs.task if obs else None,
        "video": video_info, "generator_version": __version__,
        "evaluation_split": "eval" if seed >= SEED_EVAL_MIN else "train-range",
        "train_eval_overlap": seed in train_seeds,
    }
    atomic_write_json(out / "metadata.json", meta)
    return meta


def run_inference_cli(a) -> int:
    from ..robot.kinematics import Kinematics
    from ..robot.joints import load_robot_spec
    from ..runner import make_sim

    spec = load_robot_spec()
    mdir = model_dir(a.task, a.model)
    ckpt = resolve_checkpoint(mdir, a.checkpoint)
    runner = ACTRunner(ckpt, spec, device=a.device, n_action_steps=a.n_action_steps)
    sim = make_sim(a.backend, spec, Kinematics(spec))
    sim.connect()
    sim.load_task(a.task)
    ok = 0
    for seed in a.seeds:
        m = run_inference_episode(sim, spec, runner, a.task, a.model, ckpt.parent.name, seed, max_steps=a.max_steps)
        ok += m["success"]
        print(f"seed={seed} {m['outcome']} reason={m['reason']} steps={m['control_steps']} frames={m['frames']} "
              f"saved={m['saved']} wall={m['wall_seconds']}s infer={m['policy_inference_ms']['mean']}ms "
              f"-> data/inference/{a.task}/{m['episode_id']}", flush=True)
    print(f"成功 {ok}/{len(a.seeds)}")
    return 0
