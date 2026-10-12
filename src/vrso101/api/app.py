"""FastAPI バックエンド。機能は CLI 側にあり、ここは薄い API + WebSocket。

任意のシェルコマンドを実行する API は作らない: ジョブは固定の argv (サーバー側で組み立て、入力は検証済み) だけ。
"""
from __future__ import annotations

import asyncio
import json
import os
import shutil
import sys
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Literal

import numpy as np
import yaml
from fastapi import Depends, FastAPI, HTTPException, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.exceptions import RequestValidationError
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .. import __version__
from ..eval import aggregate
from ..jobs.manager import JobManager
from ..models import registry as mreg
from ..paths import CONFIG_DIR, ROOT, data_dir
from ..recording.store import EpisodeStore
from ..tasks.registry import load_tasks
from ..training import trainer
from .diagnostics import run_diagnostics
from .events import EventBus
from .session import ApiError, SimSession

TOKEN = os.environ.get("VRSO101_TOKEN")


class TaskReq(BaseModel):
    task_id: str


class ResetReq(BaseModel):
    seed: int | None = Field(None, ge=0, le=2**31 - 1)
    randomize: bool = True


class RecordStartReq(BaseModel):
    seed: int | None = Field(None, ge=0, le=9999)    # 収録は 0..9999 (評価用 10000+ と分離)
    auto_stop: bool = True


class RecordStopReq(BaseModel):
    discard: bool = False


class EpisodePatch(BaseModel):
    label: Literal["success", "failure", "aborted"] | None = None
    excluded: bool | None = None


class CalibrateReq(BaseModel):
    forward: list[float] = Field(min_length=3, max_length=3)
    scale: float | None = Field(None, ge=0.05, le=5.0)


class ExportReq(BaseModel):
    task_id: str
    name: str = Field(pattern=r"^[A-Za-z0-9_-]{1,40}$")
    only_success: bool = True
    collection_mode: Literal["human", "scripted"] | None = None


class InferReq(BaseModel):
    task_id: str
    model_id: str = Field(pattern=r"^[A-Za-z0-9_.-]+$")
    checkpoints: list[str] = Field(default_factory=lambda: ["last"], min_length=1, max_length=8)
    seeds: list[int] = Field(default_factory=lambda: [10000], min_length=1, max_length=200)
    n_action_steps: int = Field(10, ge=1, le=200)
    speed_scale: float = Field(1.0, ge=0.1, le=2.0)
    max_steps: int | None = Field(None, ge=10, le=5000)
    device: str = Field("cuda", pattern=r"^(cpu|mps|cuda(:\d+)?)$")


def create_app(backend: str | None = None) -> FastAPI:
    backend = backend or os.environ.get("VRSO101_BACKEND", "gazebo")
    bus = EventBus()
    store = EpisodeStore()
    jm = JobManager(notify=bus.publish)
    session = SimSession(bus, store, backend=backend)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        bus.bind_loop(asyncio.get_running_loop())
        session.start()
        yield
        session.shutdown()

    app = FastAPI(title="VR SO-101 ACT", version=__version__, lifespan=lifespan)

    @app.middleware("http")
    async def token_guard(request: Request, call_next):
        """VRSO101_TOKEN が設定されていれば、変更系 (/api の GET 以外) にトークンを要求する (LAN 公開時の最低限の防御)。"""
        if TOKEN and request.url.path.startswith("/api") and request.method not in ("GET", "HEAD", "OPTIONS"):
            if request.headers.get("x-vrso101-token") != TOKEN:
                return JSONResponse({"error": {"code": "UNAUTHORIZED", "message": "トークンが必要です (X-VRSO101-Token)"}}, status_code=401)
        return await call_next(request)

    app.state.session, app.state.jm, app.state.bus, app.state.store = session, jm, bus, store

    @app.exception_handler(ApiError)
    async def _api_error(_, e: ApiError):
        return JSONResponse({"error": {"code": e.code, "message": e.message}}, status_code=e.status)

    @app.exception_handler(ValueError)
    async def _value_error(_, e: ValueError):
        return JSONResponse({"error": {"code": "INVALID", "message": str(e)}}, status_code=400)

    @app.exception_handler(FileNotFoundError)
    async def _nf(_, e: FileNotFoundError):
        return JSONResponse({"error": {"code": "NOT_FOUND", "message": str(e)}}, status_code=404)

    @app.exception_handler(RequestValidationError)
    async def _validation(_, e: RequestValidationError):
        return JSONResponse({"error": {"code": "INVALID", "message": "入力が不正です", "detail": json.loads(json.dumps(e.errors(), default=str))}}, status_code=422)

    # ------------------------------------------------------------ system
    @app.get("/api/system")
    def system():
        return {"version": __version__, "backend": backend, "session": session.status(), "data_dir": str(data_dir()),
                "tasks": list(load_tasks()), "server_time": time.time()}

    @app.get("/api/system/diagnostics")
    def diagnostics():
        return run_diagnostics()

    @app.get("/api/system/devices")
    def devices():
        return trainer.available_devices()

    @app.get("/api/tasks")
    def tasks():
        return [{"task_id": k, "name": v["name"], "description": v["description"], "instruction": v["instruction"],
                 "fps": v["fps"], "max_steps": v["max_steps"], "camera": v["camera"], "success": v["success"], "scene": v["scene"],
                 # モックは pick_place のみ。ドアは Gazebo のシーンがある (config の implemented: true)
                 "implemented": v["success"]["type"] == "pick_place" or (v.get("implemented", False) and backend != "mock")}
                for k, v in load_tasks().items()]

    @app.get("/api/robot")
    def robot():
        s = session.spec
        return {"robot_id": s.robot_id, "joint_names": s.names, "units": s.units, "lower": s.lower.tolist(),
                "upper": s.upper.tolist(), "vel_limit": s.vel_limit.tolist(), "home": s.home.tolist()}

    @app.get("/api/robot/urdf/{path:path}")
    def robot_urdf(path: str):
        """3D 表示用に URDF とメッシュを配信 (vendor の SO-101 ディレクトリ配下のみ・読み取り専用)。"""
        base = (ROOT / "vendor" / "SO-ARM100" / "Simulation" / "SO101").resolve()
        target = (base / path).resolve()
        if base not in target.parents and target != base:
            raise ApiError(400, "BAD_PATH", "不正なパス")
        if target.suffix.lower() not in (".urdf", ".stl") or not target.is_file():
            raise ApiError(404, "NOT_FOUND", "ファイルがありません")
        # 描画用に軽量化したメッシュ (sim/tools/decimate_visuals.py。外郭のずれ ≤0.3mm) があればそちらを返す
        light = ROOT / "sim" / "models" / "so101_web" / path
        if target.suffix.lower() == ".stl" and light.is_file():
            target = light
        # 差し替えが次回リロードで反映されるよう、毎回 ETag で再検証させる (長期キャッシュだと古い重いメッシュが残る)
        return FileResponse(target, headers={"Cache-Control": "no-cache"})

    # ------------------------------------------------------------ sim
    @app.get("/api/sim/status")
    def sim_status():
        return session.status()

    @app.post("/api/sim/connect")
    def sim_connect():
        return session.connect()

    @app.post("/api/sim/task")
    def sim_task(req: TaskReq):
        return session.load_task(req.task_id)

    @app.post("/api/sim/reset")
    def sim_reset(req: ResetReq):
        return session.reset(req.seed, req.randomize)

    @app.get("/api/sim/frame.jpg")
    def frame():
        if session.jpeg is None:
            raise ApiError(404, "NO_FRAME", "カメラ画像がまだありません (シム未接続か、タスク未選択)")
        return Response(session.jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    def _mjpeg(get_next):
        async def gen():
            last = None
            while True:
                data = await asyncio.get_running_loop().run_in_executor(None, get_next, last)
                if data is None:
                    await asyncio.sleep(0.2)
                    continue
                last = data
                yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n"
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame",
                                 headers={"Cache-Control": "no-store"})

    @app.get("/api/sim/camera.mjpg")
    def camera():
        def nxt(last):
            with session.jcv:
                session.jcv.wait_for(lambda: session.jpeg is not None and session.jpeg is not last, timeout=1.0)
                return session.jpeg if session.jpeg is not last else None
        return _mjpeg(nxt)

    @app.post("/api/teleop/calibrate")
    def calibrate(req: CalibrateReq):
        return session.calibrate(req.forward, req.scale)

    # ------------------------------------------------------------ episodes
    @app.get("/api/episodes")
    def episodes(task_id: str | None = None):
        items = store.list(task_id)
        total = len(items)
        ok = sum(1 for m in items if (m.get("label") or m.get("status")) == "success")
        frames = sum(m.get("num_frames", 0) or 0 for m in items)
        return {"items": items, "stats": {"total": total, "success": ok, "success_rate": ok / total if total else None,
                                           "recorded_seconds": sum(m.get("duration_s", 0) or 0 for m in items), "frames": frames}}

    @app.post("/api/episodes/start")
    def ep_start(req: RecordStartReq):
        return session.start_recording(req.seed, req.auto_stop)

    @app.post("/api/episodes/stop")
    def ep_stop(req: RecordStopReq):
        return session.stop_recording(discard=req.discard)

    @app.get("/api/episodes/{task_id}/{episode_id}")
    def ep_get(task_id: str, episode_id: str, points: int = Query(200, ge=10, le=2000)):
        m = store.load_meta(task_id, episode_id)
        try:
            a = store.load_arrays(task_id, episode_id)
            n = len(a["q"])
            idx = np.linspace(0, n - 1, min(points, n)).astype(int) if n else []
            traj = {"t": (a["sim_time"][idx] - a["sim_time"][0]).tolist(), "q": a["q"][idx].tolist(), "q_cmd": a["q_cmd"][idx].tolist()}
        except FileNotFoundError:
            traj = None
        return {"meta": m, "trajectory": traj}

    @app.get("/api/episodes/{task_id}/{episode_id}/video")
    def ep_video(task_id: str, episode_id: str):
        p = store.dir(task_id, episode_id) / "preview.mp4"
        if not p.exists():
            raise ApiError(404, "NO_VIDEO", "プレビュー動画がありません (収録中/中断/生成失敗)")
        return FileResponse(p, media_type="video/mp4")

    @app.patch("/api/episodes/{task_id}/{episode_id}")
    def ep_patch(task_id: str, episode_id: str, req: EpisodePatch):
        if req.excluded is not None:
            store.set_excluded(task_id, episode_id, req.excluded)
        if "label" in req.model_fields_set:
            store.set_label(task_id, episode_id, req.label)
        return store.load_meta(task_id, episode_id)

    @app.delete("/api/episodes/{task_id}/{episode_id}")
    def ep_delete(task_id: str, episode_id: str):
        if session.recorder is not None and session.rec.get("episode_id") == episode_id:
            raise ApiError(409, "RECORDING", "収録中のエピソードは削除できません")
        try:
            store.delete(task_id, episode_id)
        except FileNotFoundError:
            pass   # 冪等
        return {"deleted": episode_id}

    # ------------------------------------------------------------ datasets
    def _datasets() -> list[dict]:
        out = []
        root = data_dir() / "datasets"
        if root.exists():
            for t in sorted(p for p in root.iterdir() if p.is_dir()):
                for d in sorted(p for p in t.iterdir() if p.is_dir()):
                    f = d / "vrso101_meta.json"
                    if f.exists():
                        m = json.loads(f.read_text(encoding="utf-8"))
                        out.append({"task_id": t.name, "name": d.name, "path": str(d), **{k: m.get(k) for k in (
                            "num_episodes", "num_frames", "num_skipped", "outcome_breakdown", "collection_modes", "fps",
                            "created_at", "lerobot_version", "skip_reasons", "seeds", "units", "joint_names")}})
        return out

    @app.get("/api/datasets")
    def datasets(task_id: str | None = None):
        return [d for d in _datasets() if task_id in (None, d["task_id"])]

    @app.post("/api/datasets/export")
    def ds_export(req: ExportReq):
        if req.task_id not in load_tasks():
            raise ApiError(404, "UNKNOWN_TASK", f"未知のタスク {req.task_id}")
        if (data_dir() / "datasets" / req.task_id / req.name).exists():
            raise ApiError(409, "EXISTS", f"データセット {req.name} は既に存在します。別の名前にしてください")
        argv = [sys.executable, "-m", "vrso101.cli", "export", "--task", req.task_id, "--name", req.name]
        if not req.only_success:
            argv.append("--include-failures")
        if req.collection_mode:
            argv += ["--mode", req.collection_mode]
        return jm.start("export", argv, key=f"export:{req.task_id}:{req.name}", params=req.model_dump(), cwd=str(ROOT))

    @app.get("/api/datasets/{task_id}/{name}/validate")
    def ds_validate(task_id: str, name: str):
        from ..dataset.lerobot_export import validate_dataset
        root = data_dir() / "datasets" / task_id / name
        if not root.exists():
            raise ApiError(404, "NOT_FOUND", "データセットが存在しません")
        return validate_dataset(root, session.spec)

    # ------------------------------------------------------------ training
    @app.post("/api/training/start")
    def train_start(req: trainer.TrainConfig):
        try:
            return trainer.start_training(jm, req)
        except ValueError as e:
            raise ApiError(400, "TRAIN_PRECHECK", str(e))

    @app.post("/api/jobs/{job_id}/stop")
    def job_stop(job_id: str):
        return jm.stop(job_id)

    @app.get("/api/jobs")
    def jobs(kind: str | None = None):
        return [{k: v for k, v in j.items() if k not in ("metrics", "argv")} | {"last_metric": (j.get("metrics") or [None])[-1]}
                for j in jm.list(kind)]

    @app.get("/api/jobs/{job_id}")
    def job_get(job_id: str, log_lines: int = Query(60, ge=0, le=500)):
        j = jm.get(job_id)
        return {**j, "log_tail": jm.tail(job_id, log_lines) if log_lines else ""}

    # ------------------------------------------------------------ models
    @app.get("/api/models")
    def models(task_id: str | None = None):
        return mreg.list_models(task_id)

    @app.get("/api/models/{task_id}/{model_id}")
    def model_get(task_id: str, model_id: str):
        return mreg.load_model_meta(task_id, model_id)

    @app.delete("/api/models/{task_id}/{model_id}")
    def model_delete(task_id: str, model_id: str):
        d = mreg.model_dir(task_id, model_id)
        j = jm.active("gpu:train")
        if j and j["params"].get("model_id") == model_id:
            raise ApiError(409, "TRAINING", "学習中のモデルは削除できません")
        shutil.rmtree(d, ignore_errors=True)
        return {"deleted": model_id}

    # ------------------------------------------------------------ inference
    @app.post("/api/inference/start")
    def infer_start(req: InferReq):
        if jm.active("sim:inference"):
            return {**jm.active("sim:inference"), "already_running": True}
        if session.recorder is not None:
            raise ApiError(409, "RECORDING", "収録中は推論できません")
        mdir = mreg.model_dir(req.task_id, req.model_id)
        if not (mdir / mreg.META).exists():
            raise ApiError(404, "NOT_FOUND", "モデルが見つかりません")
        mm = mreg.load_model_meta(req.task_id, req.model_id)
        for c in req.checkpoints:   # 開始前に全チェックポイントを検証 (壊れていれば理由を返して開始しない)
            ck = mreg.resolve_checkpoint(mdir, c)
            ok, why = mreg.verify_checkpoint(ck)
            if not ok:
                raise ApiError(400, "BAD_CHECKPOINT", f"チェックポイント {c} が使えません: {why}")
        if req.device.startswith("cuda") and not trainer.available_devices()["devices"]["cuda"]:
            raise ApiError(400, "NO_CUDA", "CUDA が使えません。device=cpu を選んでください")
        train_seeds = set(mm.get("dataset", {}).get("seeds", []))
        overlap = sorted(set(req.seeds) & train_seeds)
        jdir_id = f"cfg-{int(time.time() * 1000)}"
        cfgdir = data_dir() / "jobs" / "_infer_cfg"
        cfgdir.mkdir(parents=True, exist_ok=True)
        work = cfgdir / jdir_id
        work.mkdir()
        cfg = {**req.model_dump(), "backend": session.backend_pref, "job_dir": str(work)}
        (work / "config.json").write_text(json.dumps(cfg), encoding="utf-8")
        session.release_sim_for_worker()
        total = len(req.seeds) * len(req.checkpoints)

        def parse(line: str, job: dict):
            if not line.startswith("@@"):
                return
            ev = json.loads(line[2:])
            k = ev["kind"]
            res = job.setdefault("result", {}) or {}
            job["result"] = res
            res.setdefault("episodes", [])
            if k == "episode":
                res["episodes"].append(ev)
                job["progress"]["step"] = ev["done"]
            elif k == "error":
                job["error"] = ev["message"]
            elif k == "step":
                res["live"] = ev
            elif k == "episode_begin":
                res["current"] = ev
            elif k == "finished":
                res["ok"] = True
                res["stopped"] = bool(ev.get("stopped"))
            bus.publish({"type": "inference", "job_id": job["id"], **{kk: vv for kk, vv in ev.items() if kk != "q"}})

        def done(job):
            session.reclaim_sim_after_worker()

        argv = [sys.executable, "-m", "vrso101.inference.worker", str(work / "config.json")]
        job = jm.start("infer", argv, key="sim:inference", params={**req.model_dump(), "work": str(work)},
                       parse_line=parse, total=total, cwd=str(ROOT), on_finish=done)
        return {**job, "warnings": ([f"シード {overlap} は学習データと重複しています (評価に使うと過大評価になります)"] if overlap else [])}

    @app.post("/api/inference/stop")
    def infer_stop():
        j = jm.active("sim:inference")
        if not j:
            return {"stopped": False}   # 冪等
        work = Path(j["params"]["work"])
        (work / "stop.flag").write_text("1")   # 優雅な停止 (動画を保存してから終了)
        def hard():
            time.sleep(8)
            cur = jm.get(j["id"])
            if cur["state"] == "running":   # 8 秒で終わらなければ強制停止
                jm.stop(j["id"])
        threading.Thread(target=hard, daemon=True).start()
        return {"stopped": True, "job_id": j["id"]}

    @app.get("/api/inference/live.mjpg")
    def infer_live():
        def nxt(last):
            j = jm.active("sim:inference")
            time.sleep(0.1)
            if not j:
                return None
            p = Path(j["params"]["work"]) / "latest.jpg"
            try:
                data = p.read_bytes()
            except FileNotFoundError:
                return None
            return data if data != last else None
        return _mjpeg(nxt)

    @app.get("/api/inference/videos")
    def infer_videos(task_id: str | None = None, success: bool | None = None, model_id: str | None = None):
        runs = aggregate.load_inference_runs(task_id)
        if success is not None:
            runs = [r for r in runs if bool(r.get("success")) == success]
        if model_id:
            runs = [r for r in runs if r.get("policy", {}).get("model_id") == model_id]
        return runs

    def _inf_dir(task_id: str, episode_id: str) -> Path:
        if not episode_id.replace("-", "").replace("_", "").isalnum() or "/" in task_id or ".." in task_id:
            raise ValueError("不正な ID")
        return data_dir() / "inference" / task_id / episode_id

    @app.get("/api/inference/videos/{task_id}/{episode_id}")
    def infer_meta(task_id: str, episode_id: str):
        return json.loads((_inf_dir(task_id, episode_id) / "metadata.json").read_text(encoding="utf-8"))

    @app.get("/api/inference/videos/{task_id}/{episode_id}/video")
    def infer_video(task_id: str, episode_id: str, download: bool = False):
        d = _inf_dir(task_id, episode_id)
        p = d / "inference.mp4"
        if not p.exists():
            raise ApiError(404, "NO_VIDEO", "動画がありません (保存失敗の可能性: metadata.json の save_error を確認)")
        name = f"{episode_id.split('-s')[0]}_{task_id}_{episode_id}.mp4"
        return FileResponse(p, media_type="video/mp4", filename=name if download else None,
                            content_disposition_type="attachment" if download else "inline")

    @app.delete("/api/inference/videos/{task_id}/{episode_id}")
    def infer_delete(task_id: str, episode_id: str):
        shutil.rmtree(_inf_dir(task_id, episode_id), ignore_errors=True)
        return {"deleted": episode_id}

    # ------------------------------------------------------------ evaluation
    @app.get("/api/evaluation")
    def evaluation(task_id: str):
        eps = store.list(task_id)
        used = [m for m in eps if not m.get("excluded")]
        q_issues: dict[str, int] = {}
        for m in eps:
            for i in (m.get("quality") or {}).get("issues", []):
                q_issues[i["code"]] = q_issues.get(i["code"], 0) + 1
        models = mreg.list_models(task_id)
        train_seeds = sorted({s for m in models for s in m.get("dataset", {}).get("seeds", [])})
        runs = aggregate.load_inference_runs(task_id)
        overlap = sorted({r["seed"] for r in runs if r.get("train_eval_overlap")})
        return {
            "task_id": task_id,
            "data_quality": {"episodes": len(eps), "used": len(used), "issues": q_issues,
                             "dropped_frames": sum((m.get("quality") or {}).get("dropped_frames") or 0 for m in eps),
                             "frames": sum(m.get("num_frames") or 0 for m in eps),
                             "frame_drop_rate": (sum((m.get("quality") or {}).get("dropped_frames") or 0 for m in eps) /
                                                 max(1, sum(m.get("num_frames") or 0 for m in eps)))},
            "training": [{"model_id": m["model_id"], "status": m.get("status"), "result": {k: v for k, v in (m.get("result") or {}).items() if k != "loss_history"},
                          "steps": m["train_config"]["steps"], "dataset": m["dataset"]["name"]} for m in models],
            "inference": aggregate.summarize(runs), "comparison": aggregate.compare(task_id),
            "warnings": ([f"評価に使ったシード {overlap} が学習データと重複しています"] if overlap else []),
            "train_seeds": train_seeds,
        }

    # ------------------------------------------------------------ settings
    @app.get("/api/settings/export")
    def settings_export():
        out: dict[str, Any] = {"version": __version__, "robot": yaml.safe_load((CONFIG_DIR / "robot" / "so101.yaml").read_text(encoding="utf-8")),
                               "sim": yaml.safe_load((CONFIG_DIR / "sim" / "gazebo.yaml").read_text(encoding="utf-8")),
                               "tasks": load_tasks()}
        return JSONResponse(out, headers={"Content-Disposition": "attachment; filename=vrso101_settings.json"})

    # ------------------------------------------------------------ websocket
    @app.websocket("/ws")
    async def ws(sock: WebSocket, since: int | None = None, token: str | None = None):
        if TOKEN and token != TOKEN:
            await sock.close(code=4401)
            return
        await sock.accept()
        q, replay, resync = bus.subscribe(since)
        session.clients += 1
        try:
            await sock.send_json({"type": "hello", "session_id": bus.session_id, "seq": bus.seq,
                                  "resync_required": resync, "status": session.status()})
            for ev in replay:
                await sock.send_json(ev)
            if bus.latest_state:
                await sock.send_json(bus.latest_state)

            async def sender():
                while True:
                    ev = await q.get()
                    await sock.send_json(ev)

            task = asyncio.create_task(sender())
            try:
                while True:
                    msg = await sock.receive_json()
                    t = msg.get("type")
                    if t in ("vr", "nudge", "ee", "release", "speed"):
                        try:
                            session.set_input(msg)
                        except (KeyError, TypeError, ValueError):
                            await sock.send_json({"type": "error", "code": "BAD_INPUT", "message": "入力メッセージが不正です"})
                    elif t == "ping":
                        await sock.send_json({"type": "pong", "t": msg.get("t")})
            finally:
                task.cancel()
        except WebSocketDisconnect:
            pass
        finally:
            session.clients = max(0, session.clients - 1)
            bus.unsubscribe(q)
            if session.clients == 0:
                session.set_input({"type": "release"})   # 全クライアント切断 = 追従停止

    # ------------------------------------------------------------ static (build 済みフロント)
    dist = ROOT / "frontend" / "dist"
    if dist.exists():
        app.mount("/", StaticFiles(directory=dist, html=True), name="ui")
    return app


app = None


def get_app() -> FastAPI:   # uvicorn --factory 用
    return create_app()
