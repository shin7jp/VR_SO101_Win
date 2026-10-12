#!/usr/bin/env python3
"""WSL 側 Gazebo シムブリッジ (TCP サーバー)。Windows の FastAPI / ACT ワーカーから使われる。

起動: scripts/wsl/start_bridge.sh  (ROS 2 を source した system python3 で動かす)
プロトコルは src/vrso101/sim/protocol.py。クライアントは常に 1 つ (操作の競合を防ぐ)。
"""
from __future__ import annotations

import argparse
import socket
import sys
import time
import traceback
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import worldgen  # noqa: E402
from gz_ros import GazeboRos, SimError  # noqa: E402
from vrso101.robot.joints import load_robot_spec  # noqa: E402
from vrso101.sim import protocol as P  # noqa: E402
from vrso101.tasks.registry import get_task, sample_scene  # noqa: E402
from vrso101.tasks.success import TaskEvaluator  # noqa: E402


class Bridge:
    def __init__(self, cfg_path: Path):
        self.spec = load_robot_spec()
        self.gz = GazeboRos(cfg_path, self.spec.names, ROOT)
        self.cfg = self.gz.cfg
        self.task: dict | None = None
        self.scene: dict | None = None
        self.evaluator: TaskEvaluator | None = None
        self.step_index = 0
        self.q_cmd = self.spec.home.copy()
        self.dt = 0.0
        self.task_objects: list[str] = []

    # ---------- ハンドラ ----------
    def op_hello(self, a):
        return {"backend": "gazebo", "joint_names": self.spec.names, "units": self.spec.units,
                "task_id": self.task["task_id"] if self.task else None, "fps": self.task["fps"] if self.task else None,
                "render_engine": self.cfg["render_engine"], "world_alive": self.gz.alive()}

    def op_load_task(self, a):
        task = get_task(a["task_id"])
        fps = float(a.get("fps") or task["fps"])
        task = {**task, "fps": fps}
        sub = int(self.cfg["substeps"])
        self.dt = 1.0 / (fps * sub)
        self.gz.start_world(worldgen.make_world_sdf(task, substeps=sub, engine=self.cfg["render_engine"]),
                            f"{task['task_id']}_{int(fps)}", task["camera"]["name"],
                            extra_models=("door",) if task["success"]["type"] in ("open_door", "lever_door") else ())
        self.task = task
        self.evaluator = TaskEvaluator(task)
        self.task_objects = []
        return {"task_id": task["task_id"], "fps": fps, "substeps": sub, "physics_dt": self.dt}

    def _require_task(self):
        if self.task is None or not self.gz.alive():
            raise SimError("NO_WORLD", "Gazebo のワールドが起動していません (load_task を先に呼んでください)")

    @property
    def home(self) -> np.ndarray:
        """タスクごとのホーム姿勢 (ドアは初期姿勢の腕が板に食い込まないよう、パンを逃がす)。"""
        return np.asarray(self.task.get("home", self.spec.home), dtype=np.float64)

    def op_reset(self, a):
        self._require_task()
        seed = int(a.get("seed", 0))
        self.scene = sample_scene(self.task, seed, bool(a.get("randomize", True)))
        gz = self.gz
        sub = self.cfg["substeps"]
        # world の reset (model_only/all) は使わない: 一度でも reset すると以後の 1 反復が約 10 倍遅くなる
        # (実測: 4ms→40ms/iter)。さらに all は sim 時刻を戻し、カメラが旧時刻まで無画像になる。
        # 代わりに「物体を消す → 腕を位置制御でホームへ戻す → 物体を置く」で初期化する。
        gz.require_extra = False
        for name in self.task_objects:
            gz.remove(name)
        self.task_objects = []
        self.q_cmd = self.home.copy()
        gz._clear_obs()
        for i in range(int(a.get("home_max_ticks", 100))):
            gz.publish_cmd(self.q_cmd)
            gz.step(sub, self.dt)
            q = self.spec.reorder(list(gz.joints[1].keys()), list(gz.joints[1].values()))
            if np.max(np.abs(q - self.home)) < 0.02 and i >= 2:
                break
        else:
            raise SimError("HOME_TIMEOUT", "腕がホーム姿勢に戻りません (関節が拘束されている可能性)")
        for name, sdf, pose6 in worldgen.make_task_objects(self.task, self.scene):
            if not gz.spawn(name, sdf, pose6):
                raise SimError("SPAWN_FAILED", f"物体 {name} の生成に失敗しました")
            self.task_objects.append(name)
        gz._clear_obs()
        gz.require_extra = bool(gz.extra_models)
        settle_ticks = int(a.get("settle_ticks", 10))
        for _ in range(settle_ticks):   # 立方体の静止待ち。エピソードの記録には含めない
            gz.publish_cmd(self.q_cmd)
            gz.step(sub, self.dt)
        self.evaluator.reset()
        self.step_index = 0
        self._evaluate()
        return {"scene": self.scene, **self._obs_header()}, self._image_bytes()

    def op_step(self, a):
        self._require_task()
        q = np.asarray(a["q"], dtype=np.float64)
        if q.shape != (self.spec.dof,) or not np.all(np.isfinite(q)):
            raise SimError("BAD_COMMAND", f"q は有限な長さ {self.spec.dof} のベクトルが必要です")
        # 防御的クリップ: クライアントの制限チェックをすり抜けても関節限界を超えさせない
        self.q_cmd = self.spec.clip_to_limits(q)
        self.gz.publish_cmd(self.q_cmd)
        n = int(a.get("n", self.cfg["substeps"]))
        t0 = time.perf_counter()
        self.gz.step(n, self.dt)
        self.step_index += 1
        self._evaluate()
        h = self._obs_header()
        h["step_wall_ms"] = (time.perf_counter() - t0) * 1000
        h["prof"] = self.gz.prof
        return h, self._image_bytes()

    def op_get_obs(self, a):
        self._require_task()
        return self._obs_header(), self._image_bytes()

    def op_task_state(self, a):
        self._require_task()
        return {"task": self._task_header()}

    def op_shutdown(self, a):
        self.gz.stop_world()
        return {}

    # ---------- 観測の組み立て ----------
    def _state(self) -> dict:
        snap = self.gz.snapshot()
        poses = snap["poses"][1] if snap["poses"] else {}
        track = set(self.task_objects) | {"panel", "lever_link"}   # ドアはリンク (panel/lever_link) の姿勢が動く (モデル座標系での相対姿勢)
        objs = {n: {"pos": poses[n][0], "quat": poses[n][1]} for n in track if n in poses}
        joints: dict[str, float] = {}
        for m, (_, pos) in snap["extra"].items():
            joints.update(pos)   # door_hinge / lever
        return {"objects": objs, "joints": joints}

    def _evaluate(self):
        st = self._state()
        kind = self.task["success"]["type"]
        if kind == "pick_place" and "cube" not in st["objects"]:
            return   # 姿勢が未着のステップは評価しない (成功にも失敗にも倒さない)
        if kind in ("open_door", "lever_door") and "door_hinge" not in st["joints"]:
            return
        self.evaluator.update(st)

    def _task_header(self) -> dict:
        ev = self.evaluator.last
        return {"success": self.evaluator.success, "metrics": ev.metrics, "detail": ev.detail}

    def _obs_header(self) -> dict:
        snap = self.gz.snapshot()
        im, jn = snap["image"], snap["joints"]
        if im is None or jn is None:
            raise SimError("NO_OBS", "カメラ画像または関節状態が未受信です")
        names = self.spec.names
        q = self.spec.reorder(list(jn[1].keys()), list(jn[1].values()))
        qd = np.array([jn[2].get(n, 0.0) for n in names])
        return {"sim_time": snap["sim_time"], "step": self.step_index, "q": q.tolist(), "qd": qd.tolist(),
                "q_cmd": self.q_cmd.tolist(), "joint_stamp": jn[0], "image_stamp": im[0],
                "image": {"w": im[2], "h": im[3], "fmt": "rgb8"}, "task": self._task_header(),
                "objects": self._state()["objects"], "ext_joints": self._state()["joints"]}

    def _image_bytes(self) -> bytes:
        im = self.gz.snapshot()["image"]
        if im[4] != "rgb8":
            raise SimError("BAD_IMAGE", f"想定外の画像エンコーディング: {im[4]}")
        return im[1]


def serve(host: str, port: int, cfg_path: Path) -> None:
    import signal
    # SIGTERM でも finally (Gazebo プロセスの後始末) を通す。通さないと gz が孤児として GPU/CPU を握り続ける
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    br = Bridge(cfg_path)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    print(f"[bridge] listening {host}:{port}", flush=True)
    try:
        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[bridge] client {addr}", flush=True)
            try:
                while True:
                    head, _ = P.recv_frame(conn)
                    rid, op = head.get("id"), head.get("op")
                    try:
                        fn = getattr(br, f"op_{op}", None)
                        if fn is None or op not in P.OPS:
                            raise SimError("BAD_OP", f"未知の op: {op}")
                        out = fn(head)
                        payload = b""
                        if isinstance(out, tuple):
                            out, payload = out
                        P.send_frame(conn, {"id": rid, "ok": True, **out}, payload)
                        if op == "shutdown":
                            return
                    except SimError as e:
                        P.send_frame(conn, {"id": rid, "ok": False, "code": e.code, "error": str(e)})
                    except Exception as e:  # 想定外もクライアントへ伝えてサーバーは生かす
                        traceback.print_exc()
                        P.send_frame(conn, {"id": rid, "ok": False, "code": "INTERNAL", "error": f"{type(e).__name__}: {e}"})
            except (ConnectionError, OSError) as e:
                print(f"[bridge] client closed: {e}", flush=True)
            finally:
                conn.close()
    finally:
        br.gz.stop_world()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=P.DEFAULT_PORT)
    ap.add_argument("--config", default=str(ROOT / "config" / "sim" / "gazebo.yaml"))
    a = ap.parse_args()
    serve(a.host, a.port, Path(a.config))
