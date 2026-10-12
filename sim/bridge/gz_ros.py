"""Gazebo Sim(Fortress) + ROS 2(Humble) の薄いラッパー。WSL の system python で動く。

lock-step 設計: Gazebo は停止状態で起動し、ControlWorld(multi_step=N) で制御 1 周期ぶんだけ進める。
カメラ・関節状態・物体姿勢はすべて「そのステップ終了時刻のスタンプ」を持つものだけを採用するので、
観測と指令の時刻ずれを作らない (リアルタイム走行だとネットワーク遅延で観測が前後に混ざる)。
"""
from __future__ import annotations

import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
from ros_gz_interfaces.msg import Entity, EntityFactory, WorldControl, WorldReset
from ros_gz_interfaces.srv import ControlWorld, DeleteEntity, SetEntityPose, SpawnEntity
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage

SKIP = set(os.environ.get("VRSO101_SKIP", "").split(","))
LOG_DIR = Path(os.environ.get("VRSO101_SIM_LOG_DIR", "/tmp/vrso101"))


class SimError(RuntimeError):
    def __init__(self, code: str, msg: str):
        super().__init__(msg)
        self.code = code


def _stamp(t) -> float:
    return t.sec + t.nanosec * 1e-9


class GazeboRos:
    def __init__(self, cfg_path: Path, joint_names: list[str], root: Path):
        self.cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
        self.root = root
        self.joint_names = joint_names
        self.world = self.cfg["world"]
        self.robot = self.cfg["robot_model"]
        self._cv = threading.Condition()
        self._gz: subprocess.Popen | None = None
        self._bridge: subprocess.Popen | None = None
        self._clear()
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("vrso101_sim_bridge")
        self._pubs = {j: self.node.create_publisher(Float64, self._fmt(self.cfg["topics"]["joint_cmd"]["ros"], joint=j), 10)
                      for j in joint_names}
        self._cam_sub = None
        self._extra_subs: list = []
        self.extra_models: tuple[str, ...] = ()
        self.require_extra = False
        self.node.create_subscription(Clock, "/clock", self._on_clock, 50)
        self.node.create_subscription(JointState, self._fmt(self.cfg["topics"]["joint_states"]["ros"]), self._on_joints, 50)
        self.node.create_subscription(TFMessage, self._fmt(self.cfg["topics"]["poses"]["ros"]), self._on_poses, 50)
        self._clients = {
            "control": self.node.create_client(ControlWorld, self._svc("control")),
            "create": self.node.create_client(SpawnEntity, self._svc("create")),
            "remove": self.node.create_client(DeleteEntity, self._svc("remove")),
            "set_pose": self.node.create_client(SetEntityPose, self._svc("set_pose")),
        }
        self._exec = MultiThreadedExecutor(num_threads=4)
        self._exec.add_node(self.node)
        threading.Thread(target=self._exec.spin, daemon=True, name="rclpy-spin").start()
        self.dt = 0.0
        self.prof: dict[str, float] = {}
        self.step_mode = os.environ.get("VRSO101_STEP_MODE", "run_to")
        self.substeps = int(self.cfg["substeps"])

    # ---------- 設定展開 ----------
    def _fmt(self, s: str, **kw) -> str:
        return s.format(world=self.world, robot=self.robot, **kw)

    def _svc(self, key: str) -> str:
        return self._fmt(self.cfg["services"][key]["name"])

    # ---------- 受信キャッシュ ----------
    def _clear(self) -> None:
        with self._cv:
            self.sim_time = 0.0
            self.clock_seen = False
            self.image: tuple[float, bytes, int, int, str] | None = None
            self.joints: tuple[float, dict[str, float], dict[str, float]] | None = None
            self.extra: dict[str, tuple[float, dict[str, float]]] = {}   # 物体側モデル (ドア等) の関節状態
            self.poses: tuple[float, dict[str, tuple[list[float], list[float]]]] | None = None

    def _clear_obs(self) -> None:
        """リセット直後に呼ぶ。古い観測キャッシュを捨てる (sim 時刻は戻さない: 下記 reset の注意参照)。"""
        with self._cv:
            self.image = self.joints = self.poses = None
            self.extra = {}

    def _on_clock(self, m: Clock) -> None:
        with self._cv:
            self.sim_time = _stamp(m.clock)
            self.clock_seen = True
            self._cv.notify_all()

    def _on_image(self, m: Image) -> None:
        with self._cv:
            self.image = (_stamp(m.header.stamp), bytes(m.data), m.width, m.height, m.encoding)
            self._cv.notify_all()

    def _on_joints(self, m: JointState) -> None:
        with self._cv:
            self.joints = (_stamp(m.header.stamp), dict(zip(m.name, m.position)), dict(zip(m.name, m.velocity)))
            self._cv.notify_all()

    def _on_extra(self, model: str, m: JointState) -> None:
        with self._cv:
            self.extra[model] = (_stamp(m.header.stamp), dict(zip(m.name, m.position)))
            self._cv.notify_all()

    def _on_poses(self, m: TFMessage) -> None:
        if not m.transforms:
            return
        d = {}
        for t in m.transforms:
            tr, q = t.transform.translation, t.transform.rotation
            d[t.child_frame_id] = ([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w])
        with self._cv:
            self.poses = (_stamp(m.transforms[0].header.stamp), d)
            self._cv.notify_all()

    # ---------- プロセス管理 ----------
    def start_world(self, world_sdf: str, task_tag: str, camera_name: str, extra_models: tuple[str, ...] = ()) -> None:
        self.stop_world()
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        sdf_path = LOG_DIR / f"{task_tag}.sdf"
        sdf_path.write_text(world_sdf, encoding="utf-8")
        env = dict(os.environ)
        env["IGN_GAZEBO_RESOURCE_PATH"] = str(self.root / "sim" / "models") + ":" + env.get("IGN_GAZEBO_RESOURCE_PATH", "")
        # -r を付けない = 停止状態で起動 (lock-step の起点)
        self._gz = subprocess.Popen(["ign", "gazebo", "-s", "--headless-rendering", "-v", "3", str(sdf_path)],
                                    env=env, stdout=open(LOG_DIR / "gz.log", "wb"), stderr=subprocess.STDOUT,
                                    start_new_session=True)
        args = []
        t = self.cfg["topics"]
        skip = set(os.environ.get("VRSO101_SKIP", "").split(","))   # 診断用: 例 poses,joint_states
        for key in ("clock", "joint_states", "poses"):
            if key in skip:
                continue
            e = t[key]
            args.append(f'{self._fmt(e["ros"])}@{e["ros_type"]}[{e["gz_type"]}')
        for m in extra_models:   # ドア等の関節状態 (成否判定に使う物理量)
            e = t["joint_states"]
            args.append(f'{self._fmt(e["ros"].replace("{robot}", m))}@{e["ros_type"]}[{e["gz_type"]}')
        cam = t["camera"]
        args.append(f'{self._fmt(cam["ros"], camera=camera_name)}@{cam["ros_type"]}[{cam["gz_type"]}')
        for j in self.joint_names:
            e = t["joint_cmd"]
            args.append(f'{self._fmt(e["ros"], joint=j)}@{e["ros_type"]}]{e["gz_type"]}')
        for k, e in self.cfg["services"].items():
            args.append(f'{self._svc(k)}@{e["ros_type"]}')
        self._bridge = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", *args],
                                        env=env, stdout=open(LOG_DIR / "bridge.log", "wb"), stderr=subprocess.STDOUT,
                                        start_new_session=True)
        self._clear()
        if self._cam_sub is not None:
            self.node.destroy_subscription(self._cam_sub)
        self._cam_sub = self.node.create_subscription(Image, self._fmt(cam["ros"], camera=camera_name), self._on_image, 10)
        for sub in self._extra_subs:
            self.node.destroy_subscription(sub)
        self._extra_subs = [self.node.create_subscription(
            JointState, self._fmt(t["joint_states"]["ros"].replace("{robot}", m)),
            (lambda msg, mm=m: self._on_extra(mm, msg)), 50) for m in extra_models]
        self.extra_models = tuple(extra_models)
        self.require_extra = False
        self._wait_ready()

    def _wait_ready(self, timeout: float = 90.0) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self._gz.poll() is not None:
                raise SimError("GZ_EXITED", f"Gazebo が起動直後に終了しました (code={self._gz.returncode}) log={LOG_DIR / 'gz.log'}")
            if all(c.service_is_ready() for c in self._clients.values()):
                return
            time.sleep(0.3)
        missing = [k for k, c in self._clients.items() if not c.service_is_ready()]
        raise SimError("BRIDGE_TIMEOUT", f"ros_gz_bridge のサービスが {timeout:.0f}s 以内に立ち上がりません: {missing}")

    def stop_world(self) -> None:
        for p in (self._bridge, self._gz):
            if p is not None and p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGTERM)
                    p.wait(timeout=4)
                except Exception:
                    try:
                        os.killpg(p.pid, signal.SIGKILL)
                    except Exception:
                        pass
        self._gz = self._bridge = None

    def alive(self) -> bool:
        return self._gz is not None and self._gz.poll() is None

    # ---------- サービス呼び出し ----------
    def _call(self, key: str, req, timeout: float = 5.0):
        cli = self._clients[key]
        done = threading.Event()
        fut = cli.call_async(req)
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            raise SimError("SERVICE_TIMEOUT", f"サービス {key} が {timeout}s 以内に応答しません")
        r = fut.result()
        return r

    def world_control(self, **kw) -> bool:
        req = ControlWorld.Request()
        wc = WorldControl()
        reset = kw.pop("reset", None)
        for k, v in kw.items():
            setattr(wc, k, v)
        if reset:
            wc.reset = WorldReset(**reset)
        req.world_control = wc
        return bool(self._call("control", req).success)

    def spawn(self, name: str, sdf: str, pose6=(0, 0, 0, 0, 0, 0)) -> bool:
        req = SpawnEntity.Request()
        req.entity_factory = EntityFactory(name=name, sdf=sdf, allow_renaming=False)
        x, y, z, r, p, yw = (float(v) for v in pose6)
        f = req.entity_factory.pose
        f.position.x, f.position.y, f.position.z = x, y, z
        cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(yw / 2), math.sin(yw / 2)
        f.orientation.w = cr * cp * cy + sr * sp * sy
        f.orientation.x = sr * cp * cy - cr * sp * sy
        f.orientation.y = cr * sp * cy + sr * cp * sy
        f.orientation.z = cr * cp * sy - sr * sp * cy
        return bool(self._call("create", req).success)

    def remove(self, name: str) -> bool:
        req = DeleteEntity.Request()
        req.entity = Entity(name=name, type=Entity.MODEL)
        return bool(self._call("remove", req).success)

    def set_pose(self, name: str, xyz, quat=(0, 0, 0, 1)) -> bool:
        req = SetEntityPose.Request()
        req.entity = Entity(name=name, type=Entity.MODEL)
        req.pose = Pose()
        req.pose.position.x, req.pose.position.y, req.pose.position.z = map(float, xyz)
        req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w = map(float, quat)
        return bool(self._call("set_pose", req).success)

    # ---------- lock-step ----------
    def publish_cmd(self, q: np.ndarray) -> None:
        for j, v in zip(self.joint_names, q):
            self._pubs[j].publish(Float64(data=float(v)))

    def step(self, n: int, dt: float, *, latch_s: float = 0.01, timeout: float = float(os.environ.get("VRSO101_STEP_TIMEOUT", "30"))) -> float:
        """n ステップ進め、そのステップ終了時刻の観測が揃うまで待つ。戻り値 = 終了 sim 時刻。"""
        time.sleep(latch_s)  # 直前に publish した指令が ign 側のコントローラへ届くのを待つ
        with self._cv:
            t_target = self.sim_time + n * dt
        tc = time.perf_counter()
        if self.step_mode == "run_to":
            # 停止状態の multi_step は 1 反復あたり ~45ms かかる (実測)。実行状態で目標時刻まで走らせ、
            # 到達で自動停止させる方が桁違いに速い (~3ms/反復)。時刻は ns 整数で指定する。
            ns = int(round(t_target * 1e9))
            ok = self.world_control(pause=False, run_to_sim_time=RosTime(sec=ns // 10**9, nanosec=ns % 10**9))
        else:
            ok = self.world_control(multi_step=int(n), pause=True)
        if not ok:
            raise SimError("STEP_REJECTED", "ControlWorld が拒否されました")
        self.prof = {"ctrl_ms": (time.perf_counter() - tc) * 1000}
        tol = dt * 0.5
        deadline = time.monotonic() + timeout

        def ready() -> bool:
            im, jn, ps = self.image, self.joints, self.poses
            # カメラは Sensors システムの都合で物理 1 ステップ遅れて描画される (静かな環境で実測: 常にちょうど 1 step)。
            # GPU/CPU が逼迫 (学習と並走) すると 2〜3 ステップに伸びるので 3.5 step まで許容し、実スタンプは観測に残して
            # 品質検査 (IMAGE_STATE_SKEW) で拾う。
            return (abs(self.sim_time - t_target) < tol and (jn is not None or "joint_states" in SKIP) and (jn is None or abs(jn[0] - t_target) < tol)
                    and im is not None and im[0] - t_target < tol and t_target - im[0] < 3.5 * dt
                    and (not self.require_extra or all(m in self.extra and abs(self.extra[m][0] - t_target) < tol
                                                      for m in self.extra_models)))

        with self._cv:
            while not ready():
                left = deadline - time.monotonic()
                if left <= 0:
                    raise SimError("STEP_TIMEOUT",
                                   f"観測が揃いません (target={t_target:.4f} clock={self.sim_time:.4f} "
                                   f"image={self.image[0] if self.image else None} joints={self.joints[0] if self.joints else None})")
                self._cv.wait(left)
            self.prof["wait_ms"] = (time.perf_counter() - tc) * 1000 - self.prof["ctrl_ms"]
            return t_target

    def snapshot(self) -> dict[str, Any]:
        with self._cv:
            im, jn, ps = self.image, self.joints, self.poses
            return {"sim_time": self.sim_time, "image": im, "joints": jn, "poses": ps, "extra": dict(self.extra)}
