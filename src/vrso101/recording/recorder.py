"""1 エピソードの収録。観測・指令の対応づけ (off-by-one) をここ 1 か所に閉じ込める。

SimClient.step(q) は「q を適用して 1 周期進めた後」の観測を返す。したがって学習データの 1 フレームは
  (image_k, state_k)  = 直前のレスポンス (指令 a_k を出す前の観測)
  action_k            = a_k を適用したレスポンスが報告する q_cmd (関節限界でクリップ後の実指令)
の組。同じレスポンスの観測と指令を組にすると action が 1 周期ずれる (tests/test_recorder.py が検出する)。
最後のレスポンスの観測は対になる action が無いので frame にしない。
"""
from __future__ import annotations

import numpy as np

from ..robot.joints import RobotSpec
from ..sim.client import Observation
from ..video import VideoWriter, write_mp4, read_frames
from .quality import check_episode
from .store import EpisodeStore, atomic_write_json, now_iso

ARRAY_KEYS = ("sim_time", "q", "qd", "q_cmd", "image_stamp", "joint_stamp", "step_wall_ms")


class EpisodeRecorder:
    def __init__(self, store: EpisodeStore, spec: RobotSpec, task_id: str, fps: float, *, meta: dict | None = None):
        self.store, self.spec, self.task_id, self.fps = store, spec, task_id, fps
        self.episode_id, self.dir = store.create(task_id, {
            "fps": fps, "joint_names": spec.names, "units": spec.units, **(meta or {})})
        (self.dir / ".active").touch()   # 異常終了検出用: 正常終了すれば消える
        self._rows: dict[str, list] = {k: [] for k in ARRAY_KEYS}
        self._vw: VideoWriter | None = None
        self._pending: Observation | None = None
        self._finished = False

    # 1. リセット直後など「次の指令を出す前」の観測を渡す
    def prime(self, obs: Observation) -> None:
        self._pending = obs

    # 2. 指令を送って返ってきたレスポンス (applied_obs) を渡す。直前の観測と組にしてフレーム化する
    def add_step(self, applied: Observation) -> None:
        if self._finished:
            raise RuntimeError("終了済みのエピソードに追記できません")
        if self._pending is None:
            raise RuntimeError("prime() で開始観測を渡してください")
        o = self._pending
        h, w = o.image.shape[:2]
        if self._vw is None:
            self._vw = VideoWriter(self.dir / "frames.mkv", self.fps, w, h, codec="ffv1")
            self.store.update_meta(self.task_id, self.episode_id, image_shape=[h, w])
        self._vw.write(o.image)
        self._rows["sim_time"].append(o.sim_time)
        self._rows["q"].append(o.q)
        self._rows["qd"].append(o.qd)
        self._rows["q_cmd"].append(applied.q_cmd)             # ← 適用された指令は applied 側
        self._rows["image_stamp"].append(o.image_stamp)
        self._rows["joint_stamp"].append(o.joint_stamp)
        self._rows["step_wall_ms"].append(applied.step_wall_ms or 0.0)
        self._pending = applied
        if len(self._rows["q"]) % 50 == 0:
            self._flush_arrays()   # 途中で落ちても直近までは残す

    @property
    def num_frames(self) -> int:
        return len(self._rows["q"])

    def _arrays(self) -> dict[str, np.ndarray]:
        return {k: np.asarray(v, dtype=np.float64) for k, v in self._rows.items()}

    def _flush_arrays(self) -> None:
        np.savez(self.dir / "states.npz", **self._arrays())

    def finish(self, status: str, *, task_result: dict | None = None, abort_reason: str | None = None,
               expected_frames: int | None = None) -> dict:
        """status: success | failure | aborted。フレームを確実にフラッシュし、品質を計算して meta に残す。"""
        if self._finished:
            return self.store.load_meta(self.task_id, self.episode_id)
        self._finished = True
        if self._vw is not None:
            self._vw.close()
        self._flush_arrays()
        arrays = self._arrays()
        quality = check_episode(arrays, self.fps, self.spec.lower, self.spec.upper, expected_frames=expected_frames)
        preview_ok = False
        if self.num_frames > 0:
            try:
                write_mp4(self.dir / "preview.mp4", read_frames(self.dir / "frames.mkv"), self.fps)
                preview_ok = True
            except Exception as e:   # プレビュー失敗は収録データ自体を無効にしない
                quality["issues"].append({"code": "PREVIEW_FAILED", "level": "warning", "message": str(e)})
        meta = self.store.update_meta(
            self.task_id, self.episode_id, status=status, ended_at=now_iso(), num_frames=self.num_frames,
            duration_s=self.num_frames / self.fps, quality=quality, preview=preview_ok,
            task_result=task_result or {}, **({"abort_reason": abort_reason} if abort_reason else {}))
        (self.dir / ".active").unlink(missing_ok=True)
        return meta

    def abort(self, reason: str) -> dict:
        return self.finish("aborted", abort_reason=reason)
