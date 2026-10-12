"""シムセッション: シムの唯一の所有者。制御ループ (lock-step) を 1 本のスレッドで回す。

なぜ 1 スレッドか: Gazebo ブリッジは同時に 1 クライアントで、step を複数の呼び出し元が奪い合うと
観測と指令の時刻対応が壊れる。REST/WS からの操作はコマンドキュー経由で制御スレッドに渡し、
テレオペ・収録・リセットはすべてこのループの中で直列に実行する。

VR / PC / タッチの入力は「最新入力」として保持し、毎周期 (fps) 1 回だけ IK→制限→平滑化→step へ流す。
入力が途絶えたら追従をやめて現在姿勢を保持する (通信切断時の停止)。
"""
from __future__ import annotations

import logging
import queue
import threading
import time
from concurrent.futures import Future
from typing import Any, Callable

import cv2
import numpy as np

from ..recording.recorder import EpisodeRecorder
from ..recording.store import EpisodeStore
from ..robot.joints import load_robot_spec
from ..robot.kinematics import Kinematics
from ..runner import make_sim
from ..sim.client import Observation, SimCommandError, SimUnavailable
from ..tasks.registry import get_task, load_tasks
from ..teleop.controller import Status, TeleopController
from ..teleop.transform import FrameConfig, calibrate_yaw_from_forward
from .events import EventBus

log = logging.getLogger("vrso101.session")
STATE_HZ = 10.0
JPEG_HZ = 12.0
INPUT_STALE_S = 0.4
EE_DRAG_MAX_SPEED = 0.5   # マウス追従時の EE 最大速度 [m/s] (カーソルを急に飛ばしても一気には動かさない)


class ApiError(Exception):
    def __init__(self, status: int, code: str, message: str):
        super().__init__(message)
        self.status, self.code, self.message = status, code, message


class SimSession:
    def __init__(self, bus: EventBus, store: EpisodeStore, backend: str = "gazebo"):
        self.bus, self.store = bus, store
        self.spec = load_robot_spec()
        self.kin = Kinematics(self.spec)
        self.backend_pref = backend
        self.sim = None
        self.backend: str | None = None
        self.sim_info: dict[str, Any] = {}
        self.task_id: str | None = None
        self.task: dict | None = None
        self.mode = "disconnected"            # disconnected | idle | recording | inference | error
        self.last_error: dict | None = None
        self.obs: Observation | None = None
        self.controller: TeleopController | None = None
        self.frame = FrameConfig()
        self._calib = {"yaw": 0.0, "scale": 1.0}
        self._input: dict[str, Any] = {"source": None, "t": 0.0}
        self._ilock = threading.Lock()
        self._cmdq: queue.Queue[tuple[Callable, Future]] = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="sim-loop")
        self.recorder: EpisodeRecorder | None = None
        self.rec: dict[str, Any] = {}
        self.jpeg: bytes | None = None
        self.jcv = threading.Condition()
        self.tick = 0
        self.overruns = 0
        self.fail_streak = 0
        self.last_tick_ms = 0.0
        self.paused_for_inference = False
        self._last_reconnect = 0.0
        self.clients = 0
        self.seed_counter = 0

    # ---------- ライフサイクル ----------
    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self._stop.set()
        self._thread.join(timeout=5)
        if self.recorder is not None:
            try:
                self.recorder.abort("サーバー停止")
            except Exception:
                pass
        if self.sim is not None:
            try:
                self.sim.close()
            except Exception:
                pass

    def submit(self, fn: Callable[[], Any], timeout: float = 180.0) -> Any:
        """制御スレッドで fn を実行して結果を返す (他スレッドからの全操作はこれを通す)。"""
        fut: Future = Future()
        self._cmdq.put((fn, fut))
        return fut.result(timeout=timeout)

    # ---------- 接続・タスク ----------
    def _connect(self) -> None:
        if self.sim is not None:
            try:
                self.sim.close()
            except Exception:
                pass
        sim = make_sim(self.backend_pref, self.spec, self.kin)
        info = sim.connect()
        self.sim, self.backend, self.sim_info = sim, self.backend_pref, info
        self.mode = "idle" if self.task_id else "idle"

    def connect(self) -> dict:
        def _do():
            try:
                self._connect()
                self.last_error = None
            except SimUnavailable as e:
                self.mode = "disconnected"
                raise ApiError(503, "SIM_UNAVAILABLE", str(e)) from e
            return self.status()
        return self.submit(_do)

    def load_task(self, task_id: str) -> dict:
        if task_id not in load_tasks():
            raise ApiError(404, "UNKNOWN_TASK", f"未知のタスク {task_id}")

        def _do():
            if self.recorder is not None:
                raise ApiError(409, "RECORDING", "収録中はタスクを切り替えられません。先に停止してください")
            if self.paused_for_inference:
                raise ApiError(409, "INFERENCE_RUNNING", "推論実行中です")
            if self.sim is None or not self.sim.connected:
                self._connect_or_raise()
            self.task = get_task(task_id)
            try:
                self.sim.load_task(task_id)
            except (SimCommandError, SimUnavailable) as e:
                raise ApiError(503, "LOAD_FAILED", f"タスクの読み込みに失敗: {e}") from e
            self.task_id = task_id
            self.controller = TeleopController(self.spec, self.kin, rate_hz=self.task["fps"], frame=self.frame,
                                               smoothing=0.5)
            self.controller.pos_priority = True   # 人手操作は姿勢が実現不能になっても位置キーが効き続けること
            self._do_reset(seed=None, randomize=True)
            self.bus.publish({"type": "mode", "mode": self.mode, "task_id": task_id})
            return self.status()
        return self.submit(_do)

    def _connect_or_raise(self) -> None:
        try:
            self._connect()
        except SimUnavailable as e:
            self.mode = "disconnected"
            raise ApiError(503, "SIM_UNAVAILABLE", str(e)) from e

    def _next_seed(self) -> int:
        # 収録用シードは 0..9999 (評価用 >= 10000 と重ならない)。呼ぶたびに進める
        self.seed_counter = (self.seed_counter + 1) % 10000
        return int(time.time()) % 5000 + self.seed_counter

    def _do_reset(self, seed: int | None, randomize: bool) -> Observation:
        if self.task_id is None:
            raise ApiError(409, "NO_TASK", "タスクが選択されていません")
        seed = self._next_seed() % 10000 if seed is None else int(seed)
        try:
            obs = self.sim.reset(seed=seed, randomize=randomize)
        except SimUnavailable as e:
            self._enter_error("SIM_DISCONNECTED", str(e))
            raise ApiError(503, "SIM_UNAVAILABLE", str(e)) from e
        except SimCommandError as e:
            raise ApiError(500, "RESET_FAILED", f"リセットに失敗: {e}") from e
        self.obs = obs
        self.controller.reset(obs.q)
        self.mode = "idle"
        self.fail_streak = 0
        self._input = {"source": None, "t": 0.0}
        self.bus.publish({"type": "mode", "mode": "idle", "seed": seed, "task_id": self.task_id})
        return obs

    def reset(self, seed: int | None = None, randomize: bool = True) -> dict:
        def _do():
            if self.recorder is not None:
                raise ApiError(409, "RECORDING", "収録中です。停止または破棄してからリセットしてください")
            self._do_reset(seed, randomize)   # 冪等: 何度呼んでも同じ初期化に収束する
            return self.status()
        return self.submit(_do)

    # ---------- 入力 ----------
    def set_input(self, msg: dict[str, Any]) -> None:
        """WS スレッドから呼ぶ。最新入力だけ保持 (ループが周期ごとに消費する)。"""
        kind = msg.get("type")
        with self._ilock:
            now = time.monotonic()
            if kind == "vr":
                self._input = {"source": "vr", "t": now, "pos": msg["pos"], "quat": msg["quat"],
                               "deadman": bool(msg.get("deadman")), "gripper": msg.get("gripper")}
            elif kind == "nudge":   # PC/タッチ: EE 速度指令 (m/s, rad/s)。押している間だけ送られる
                self._input = {"source": "nudge", "t": now, "v": msg.get("v", [0, 0, 0]), "w": msg.get("w", [0, 0, 0]),
                               "gripper": msg.get("gripper")}
            elif kind == "ee":      # マウスドラッグ: EE の目標位置 (絶対, ロボット座標 [m])。カーソルへ追従する
                self._input = {"source": "ee", "t": now, "pos": msg["pos"], "gripper": None}
            elif kind == "release":
                self._input = {"source": None, "t": now}
            elif kind == "speed" and self.controller:
                self.controller.set_speed(float(msg.get("value", 1.0)))

    def calibrate(self, forward_xr, scale: float | None = None) -> dict:
        yaw = calibrate_yaw_from_forward(forward_xr)
        self.frame.yaw_offset = yaw
        if scale is not None:
            self.frame.scale = float(scale)
        if self.controller:
            self.controller.frame = self.frame
            self.controller.release()   # 基準 (クラッチ) を取り直す
        return {"yaw_offset": yaw, "scale": self.frame.scale}

    # ---------- 収録 ----------
    def start_recording(self, seed: int | None, auto_stop: bool = True, mode: str = "human") -> dict:
        def _do():
            if self.task_id is None or self.obs is None:
                raise ApiError(409, "NO_TASK", "先にタスクを選択してください")
            if self.recorder is not None:
                return {**self.status(), "already_recording": True}   # 冪等 (連打しても 1 本)
            if self.paused_for_inference:
                raise ApiError(409, "INFERENCE_RUNNING", "推論実行中は収録できません")
            if self.mode not in ("idle",):
                raise ApiError(409, "NOT_READY", f"収録を開始できない状態です (mode={self.mode})")
            obs = self._do_reset(seed, True)    # 毎エピソード初期状態から (リセット済みでも新しいシードで)
            self.recorder = EpisodeRecorder(self.store, self.spec, self.task_id, float(self.task["fps"]), meta={
                "seed": self.last_seed(obs), "scene": obs.scene, "backend": self.backend,
                "collection_mode": mode, "task_name": self.task["name"], "camera": self.task["camera"],
                "max_steps": self.task["max_steps"]})
            self.recorder.prime(obs)
            self.rec = {"episode_id": self.recorder.episode_id, "auto_stop": auto_stop, "success_ticks": 0,
                        "started": time.monotonic(), "seed": self.last_seed(obs)}
            self.mode = "recording"
            self.bus.publish({"type": "mode", "mode": "recording", "episode_id": self.recorder.episode_id})
            return self.status()
        return self.submit(_do)

    @staticmethod
    def last_seed(obs: Observation) -> int | None:
        return (obs.scene or {}).get("seed")

    def stop_recording(self, discard: bool = False, reason: str = "user_stop") -> dict:
        def _do():
            return self._finish_recording(discard=discard, reason=reason)
        return self.submit(_do)

    def _finish_recording(self, *, discard: bool = False, reason: str = "user_stop") -> dict:
        rec = self.recorder
        if rec is None:
            return {"stopped": False, **self.status()}     # 冪等
        self.recorder = None
        self.rec = {}
        self.mode = "idle"
        ob = self.obs
        if discard:
            meta = rec.abort("利用者が破棄")
        else:
            success = bool(ob and ob.task.get("success"))
            meta = rec.finish("success" if success else "failure", task_result=ob.task if ob else None,
                              expected_frames=None)
        self.bus.publish({"type": "episode", "event": "finished", "meta": _brief(meta)})
        self.bus.publish({"type": "mode", "mode": "idle"})
        return {"stopped": True, "episode": meta, **self.status()}

    # ---------- 推論ワーカーとの排他 ----------
    def release_sim_for_worker(self) -> None:
        def _do():
            if self.recorder is not None:
                raise ApiError(409, "RECORDING", "収録中は推論を開始できません")
            if self.paused_for_inference:
                return
            if self.sim is not None:
                try:
                    self.sim.close()
                except Exception:
                    pass
            self.paused_for_inference = True
            self.mode = "inference"
            self.bus.publish({"type": "mode", "mode": "inference"})
        self.submit(_do)

    def reclaim_sim_after_worker(self) -> None:
        def _do():
            self.paused_for_inference = False
            self.mode = "idle"
            try:
                self._connect()
                if self.task_id:
                    # ワーカーが別タスクを読み込んでいる可能性があるので読み直す (ワールド起動は数秒)
                    self.sim.load_task(self.task_id)
                    self._do_reset(None, True)
            except Exception as e:
                self._enter_error("RECONNECT_FAILED", str(e))
            self.bus.publish({"type": "mode", "mode": self.mode})
        try:
            self.submit(_do, timeout=240)
        except Exception as e:  # pragma: no cover
            log.error("reclaim failed: %s", e)

    # ---------- 状態 ----------
    def _enter_error(self, code: str, message: str) -> None:
        self.mode = "error" if not self.paused_for_inference else self.mode
        self.last_error = {"code": code, "message": message, "time": time.time()}
        self.bus.publish({"type": "error", **self.last_error})
        if self.recorder is not None:
            try:
                meta = self.recorder.abort(f"{code}: {message}")
                self.bus.publish({"type": "episode", "event": "aborted", "meta": _brief(meta)})
            except Exception:
                pass
            self.recorder, self.rec = None, {}

    def status(self) -> dict[str, Any]:
        o = self.obs
        return {
            "session_id": self.bus.session_id, "mode": self.mode, "backend": self.backend or self.backend_pref,
            "connected": bool(self.sim is not None and getattr(self.sim, "connected", False)),
            "task_id": self.task_id, "fps": self.task["fps"] if self.task else None,
            "max_steps": self.task["max_steps"] if self.task else None,
            "joint_names": self.spec.names, "units": self.spec.units, "last_error": self.last_error,
            "recording": ({"episode_id": self.rec.get("episode_id"), "frames": self.recorder.num_frames if self.recorder else 0,
                           "seed": self.rec.get("seed")} if self.recorder else None),
            "task_state": o.task if o else None, "sim_time": o.sim_time if o else None,
            "frame": {"yaw_offset": self.frame.yaw_offset, "scale": self.frame.scale},
            "speed": self.controller.speed if self.controller else 1.0,
            "tick_ms": round(self.last_tick_ms, 1), "overruns": self.overruns,
            "is_mock": self.backend == "mock",
        }

    # ---------- 制御ループ ----------
    def _drain_cmds(self) -> None:
        while True:
            try:
                fn, fut = self._cmdq.get_nowait()
            except queue.Empty:
                return
            try:
                fut.set_result(fn())
            except BaseException as e:   # ApiError も含め呼び出し元へ返す
                fut.set_exception(e)

    def _active(self) -> bool:
        return (self.sim is not None and getattr(self.sim, "connected", False) and self.task_id is not None
                and not self.paused_for_inference and self.mode in ("idle", "recording") and self.obs is not None)

    def _loop(self) -> None:
        next_t = time.monotonic()
        last_state = last_jpg = 0.0
        while not self._stop.is_set():
            self._drain_cmds()
            if not self._active():
                self._maybe_reconnect()
                time.sleep(0.03)
                next_t = time.monotonic()
                continue
            fps = float(self.task["fps"])
            dt = 1.0 / fps
            t0 = time.monotonic()
            try:
                self._tick(dt)
            except SimUnavailable as e:
                self._enter_error("SIM_DISCONNECTED", f"シムとの通信が切断されました: {e}")
                continue
            except SimCommandError as e:
                self.fail_streak += 1
                if self.fail_streak >= 3:
                    self._enter_error("SIM_STEP_FAILED", str(e))
                continue
            except Exception as e:   # 想定外でもループを止めない。収録は壊れるので中断として残す
                log.exception("tick error")
                self._enter_error("INTERNAL", f"{type(e).__name__}: {e}")
                continue
            self.fail_streak = 0
            now = time.monotonic()
            self.last_tick_ms = (now - t0) * 1000
            if now - last_state >= 1.0 / STATE_HZ:
                last_state = now
                self.bus.publish(self._state_event())
            if now - last_jpg >= 1.0 / JPEG_HZ and self.obs is not None:
                last_jpg = now
                self._encode_jpeg()
            next_t += dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                self.overruns += 1 if sleep < -dt else 0
                next_t = time.monotonic()   # 遅れを取り戻そうとしない (取り戻すと指令が連続して詰まる)

    def _tick(self, dt: float) -> None:
        ctl = self.controller
        with self._ilock:
            inp = dict(self._input)
        fresh = inp["source"] is not None and (time.monotonic() - inp["t"]) < INPUT_STALE_S
        if inp["source"] == "vr" and fresh:
            out = ctl.update_vr(inp["pos"], inp["quat"], deadman=inp["deadman"], gripper=inp.get("gripper"))
        elif inp["source"] == "ee" and fresh:
            out = ctl.move_toward(inp["pos"], EE_DRAG_MAX_SPEED * dt * ctl.speed)
        elif inp["source"] == "nudge" and fresh:
            v, w = np.asarray(inp["v"], dtype=float), np.asarray(inp["w"], dtype=float)
            out = ctl.nudge(v * dt * ctl.speed, w * dt * ctl.speed, gripper=inp.get("gripper"))
        else:
            if inp["source"] is not None and not fresh:
                ctl.check_stale()   # 入力途絶 = 通信切断扱い。追従をやめて保持
            out = ctl.hold(inp.get("gripper") if inp["source"] == "nudge" else None)
        q = out.q_target
        prev = self.obs
        new = self.sim.step(q)
        self.obs = new
        self.tick += 1
        self.teleop_status = out.status.value
        if self.recorder is not None:
            self.recorder.add_step(new)
            self._auto_stop_checks(new)

    def _auto_stop_checks(self, obs: Observation) -> None:
        r = self.rec
        if not r or not r.get("auto_stop"):
            return
        if obs.task.get("success"):
            r["success_ticks"] += 1
            if r["success_ticks"] >= 12:    # 成功後に少し静止してから自動終了
                self._finish_recording(reason="auto_success")
                return
        if self.recorder is not None and self.recorder.num_frames >= int(self.task["max_steps"]):
            self._finish_recording(reason="timeout")

    teleop_status = "idle"

    def _state_event(self) -> dict[str, Any]:
        o = self.obs
        ee = self.kin.fk(o.q)[:3, 3] if o is not None else None
        ctl = self.controller
        return {
            "type": "state", "mode": self.mode, "task_id": self.task_id,
            "q": o.q.tolist() if o is not None else None, "q_cmd": ctl.q.tolist() if ctl else None,
            "ee": ee.tolist() if ee is not None else None, "teleop": self.teleop_status,
            "sim_time": o.sim_time if o is not None else None, "task_state": o.task if o is not None else None,
            "objects": o.objects if o is not None else {},
            "recording": ({"episode_id": self.rec.get("episode_id"), "frames": self.recorder.num_frames,
                           "seconds": round(self.recorder.num_frames / float(self.task["fps"]), 1)} if self.recorder else None),
            "tick_ms": round(self.last_tick_ms, 1), "overruns": self.overruns,
            "step_ms": o.step_wall_ms if o is not None else None, "is_mock": self.backend == "mock",
        }

    def _encode_jpeg(self) -> None:
        img = self.obs.image
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.jcv:
                self.jpeg = buf.tobytes()
                self.jcv.notify_all()

    def _maybe_reconnect(self) -> None:
        """切断/エラー後の自動再接続 (2 秒間隔)。ブリッジ側のワールドが生きていれば続行できる。"""
        if self.mode not in ("error", "disconnected") or self.paused_for_inference or self.task_id is None:
            return
        if time.monotonic() - self._last_reconnect < 2.0:
            return
        self._last_reconnect = time.monotonic()
        try:
            self._connect()
            if self.sim_info.get("task_id") != self.task_id:
                self.sim.load_task(self.task_id)
            self._do_reset(None, True)
            self.last_error = None
            self.bus.publish({"type": "mode", "mode": "idle", "reconnected": True})
        except Exception:
            pass


def _brief(meta: dict) -> dict:
    keys = ("episode_id", "task_id", "status", "label", "excluded", "num_frames", "duration_s", "seed", "collection_mode",
            "started_at", "ended_at", "abort_reason", "quality")
    out = {k: meta.get(k) for k in keys}
    q = meta.get("quality") or {}
    out["quality"] = {"ok": q.get("ok"), "issues": q.get("issues", [])[:5], "dropped_frames": q.get("dropped_frames")}
    return out
