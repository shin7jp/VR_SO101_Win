"""推論ワーカー (別プロセス): ACT/PyTorch を API プロセスから分離する。

API はシム接続を手放してからこのプロセスを起動する (ブリッジは 1 クライアントのみ)。
`@@` で始まる行が API への通知 (JSON)。ライブ映像は job_dir/latest.jpg、停止は job_dir/stop.flag で受ける。
"""
from __future__ import annotations

import json
import sys
import threading
import time
from pathlib import Path

import cv2

from ..models.registry import model_dir, resolve_checkpoint
from ..robot.joints import load_robot_spec
from ..robot.kinematics import Kinematics
from ..runner import make_sim
from .runtime import ACTRunner, PolicyLoadError, run_inference_episode


def emit(kind: str, **kw) -> None:
    print("@@" + json.dumps({"kind": kind, **kw}, ensure_ascii=False), flush=True)


def main(argv=None) -> int:
    cfg = json.loads(Path((argv or sys.argv)[1]).read_text(encoding="utf-8"))
    job_dir = Path(cfg["job_dir"])
    stop = threading.Event()

    def watch_flag():
        while not stop.is_set():
            if (job_dir / "stop.flag").exists():
                stop.set()
            time.sleep(0.3)
    threading.Thread(target=watch_flag, daemon=True).start()

    spec = load_robot_spec()
    task_id, model_id = cfg["task_id"], cfg["model_id"]
    seeds, ckpts = cfg["seeds"], cfg["checkpoints"]
    total = len(seeds) * len(ckpts)
    emit("start", total=total)
    try:
        mdir = model_dir(task_id, model_id)
        runners = []
        for c in ckpts:   # 先に全チェックポイントを検証 (途中で壊れていて無駄に走らせない)
            path = resolve_checkpoint(mdir, c)
            runners.append((path.parent.name, ACTRunner(path, spec, device=cfg.get("device", "cuda"),
                                                       n_action_steps=int(cfg.get("n_action_steps", 10)))))
        sim = make_sim(cfg.get("backend", "gazebo"), spec, Kinematics(spec))
        sim.connect()
        sim.load_task(task_id)
    except (PolicyLoadError, FileNotFoundError, ValueError) as e:
        emit("error", message=str(e))
        return 2
    except Exception as e:
        emit("error", message=f"{type(e).__name__}: {e}")
        return 3

    done = 0
    last_jpg = [0.0]

    def on_step(obs, steps):
        now = time.monotonic()
        if now - last_jpg[0] > 0.08:
            last_jpg[0] = now
            ok, buf = cv2.imencode(".jpg", cv2.cvtColor(obs.image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                tmp = job_dir / "latest.tmp"
                tmp.write_bytes(buf.tobytes())
                tmp.replace(job_dir / "latest.jpg")
            emit("step", step=steps, q=[round(float(v), 4) for v in obs.q], success=bool(obs.task.get("success")),
                 sim_time=round(obs.sim_time, 3), metrics=obs.task.get("metrics"))

    code = 0
    for ck_name, runner in runners:
        for seed in seeds:
            if stop.is_set():
                break
            emit("episode_begin", checkpoint=ck_name, seed=seed, index=done)
            m = run_inference_episode(sim, spec, runner, task_id, model_id, ck_name, seed,
                                      max_steps=cfg.get("max_steps"), speed_scale=float(cfg.get("speed_scale", 1.0)),
                                      stop=stop, on_step=on_step)
            done += 1
            emit("episode", done=done, total=total, episode_id=m["episode_id"], checkpoint=ck_name, seed=seed,
                 outcome=m["outcome"], reason=m["reason"], success=m["success"], saved=m["saved"],
                 save_error=m["save_error"], sim_seconds=m["sim_seconds"], frames=m["frames"])
            if not m["saved"]:
                code = 4
            if m["reason"].startswith("sim_disconnected"):
                emit("error", message=m["reason"])
                return 5
    emit("finished", done=done, total=total, stopped=stop.is_set())
    try:
        sim.close()
    except Exception:
        pass
    return code


if __name__ == "__main__":
    sys.exit(main())
