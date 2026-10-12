"""推論ループの安全動作 (スタブポリシー + モックシム)。ACT/Gazebo 連携の検証ではない (それは tests/test_e2e_gazebo.py)。"""
import json
import threading

import numpy as np
import pytest

from vrso101.inference.runtime import run_inference_episode
from vrso101.robot.joints import load_robot_spec
from vrso101.sim.mock import MockSim
from vrso101.video import probe_video

spec = load_robot_spec()


class StubRunner:
    chunk_size, n_action_steps, device = 50, 10, "cpu"

    def __init__(self, fn):
        self.fn, self.infer_ms, self.k = fn, [], 0

    def reset(self):
        self.k = 0

    def act(self, obs):
        self.k += 1
        self.infer_ms.append(1.0)
        return self.fn(self.k, obs)


@pytest.fixture
def sim(tmp_path, monkeypatch):
    monkeypatch.setenv("VRSO101_DATA_DIR", str(tmp_path / "data"))
    s = MockSim(spec)
    s.load_task("pick_place")
    return s


def run(sim, fn, **kw):
    return run_inference_episode(sim, spec, StubRunner(fn), "pick_place", "m1", "000100", 10001, max_steps=25, **kw)


def test_timeout_saves_verified_video_and_metadata(sim, tmp_path):
    m = run(sim, lambda k, o: spec.home)
    assert m["outcome"] == "timeout" and m["reason"] == "timeout" and not m["success"]
    assert m["saved"] and m["frames"] == 26   # 初期フレーム + 25 ステップ
    d = tmp_path / "data" / "inference" / "pick_place" / m["episode_id"]
    assert probe_video(d / "inference.mp4")["codec"] == "h264"
    meta = json.loads((d / "metadata.json").read_text(encoding="utf-8"))
    for key in ("policy", "seed", "scene", "started_at", "ended_at", "inference_fps", "sim", "task_result", "video", "evaluation_split"):
        assert key in meta
    assert meta["policy"]["model_id"] == "m1" and meta["policy"]["checkpoint"] == "000100" and meta["evaluation_split"] == "eval"


def test_nan_action_is_never_sent_to_sim(sim):
    sent = []
    orig = sim.step
    sim.step = lambda q: (sent.append(np.array(q)), orig(q))[1]
    m = run(sim, lambda k, o: np.full(6, np.nan) if k > 3 else spec.home)
    assert m["reason"] == "invalid_action" and m["outcome"] == "abnormal"
    assert all(np.all(np.isfinite(q)) for q in sent) and len(sent) == 3
    assert m["saved"]   # 異常停止でも途中までの動画は保存して検証する


def test_velocity_limit_is_applied_to_policy_output(sim):
    qs = []
    orig = sim.step
    sim.step = lambda q: (qs.append(np.array(q)), orig(q))[1]
    run(sim, lambda k, o: spec.upper)         # 常に上限へ飛べという極端な指令
    dt = 1 / 25
    diffs = np.abs(np.diff(np.vstack(qs), axis=0))
    assert np.all(diffs <= spec.vel_limit * dt + 1e-9)
    assert np.all(np.vstack(qs) <= spec.upper + 1e-9)


def test_sim_disconnect_is_a_safe_stop_and_reported(sim):
    state = {"n": 0}
    orig = sim.step

    def flaky(q):
        state["n"] += 1
        if state["n"] == 5:
            sim.fail_next_step = True
        return orig(q)
    sim.step = flaky
    m = run(sim, lambda k, o: spec.home)
    assert m["reason"].startswith("sim_disconnected") and m["outcome"] == "abnormal" and m["saved"]


def test_user_stop(sim):
    stop = threading.Event()
    m = run(sim, lambda k, o: (stop.set() if k == 4 else None) or spec.home, stop=stop)
    assert m["reason"] == "user_stop" and m["outcome"] == "stopped" and m["control_steps"] == 4


def test_policy_exception_is_reported_not_raised(sim):
    def boom(k, o):
        raise RuntimeError("boom")
    m = run(sim, boom)
    assert m["outcome"] == "abnormal" and "boom" in m["reason"]


def test_training_seed_overlap_is_flagged(sim, tmp_path):
    from vrso101.models.registry import META, model_dir
    d = model_dir("pick_place", "m1")
    d.mkdir(parents=True)
    (d / META).write_text(json.dumps({"model_id": "m1", "dataset": {"seeds": [10001, 1, 2], "repo_id": "local/x", "name": "x"}}), encoding="utf-8")
    m = run(sim, lambda k, o: spec.home)
    assert m["train_eval_overlap"] is True and m["dataset"] == "local/x"
