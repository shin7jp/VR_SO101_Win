import numpy as np
import pytest

from vrso101.recording.quality import check_episode
from vrso101.recording.recorder import EpisodeRecorder
from vrso101.recording.store import EpisodeStore
from vrso101.robot.joints import load_robot_spec
from vrso101.sim.mock import MockSim
from vrso101.video import probe_video, read_frames

spec = load_robot_spec()


@pytest.fixture
def store(tmp_path):
    return EpisodeStore(tmp_path / "episodes")


def _record(store, n=20, seed=1):
    sim = MockSim(spec)
    sim.load_task("pick_place")
    obs = sim.reset(seed=seed)
    rec = EpisodeRecorder(store, spec, "pick_place", 25.0, meta={"seed": seed})
    rec.prime(obs)
    cmds, obs_before = [], []
    for k in range(n):
        q_cmd = spec.home + np.array([0.01 * k, 0, 0, 0, 0, 0.02 * k])
        cmds.append(spec.clip_to_limits(q_cmd))
        obs_before.append(obs)
        obs = sim.step(q_cmd)
        rec.add_step(obs)
    return rec, cmds, obs_before


def test_action_is_not_shifted_by_one_tick(store):
    """フレーム k = (step k より前の観測, step k で適用した指令)。1 周期ずれると state と action の因果が壊れる。"""
    rec, cmds, before = _record(store)
    meta = rec.finish("success")
    a = store.load_arrays("pick_place", rec.episode_id)
    assert meta["num_frames"] == 20 and len(a["q"]) == 20
    for k in range(20):
        np.testing.assert_allclose(a["q_cmd"][k], cmds[k], atol=1e-12)
        np.testing.assert_allclose(a["q"][k], before[k].q, atol=1e-12)
    # 第 k+1 フレームの状態は第 k フレームの指令を適用した結果 (= before[k+1])。指令 k が state k+1 に効いている
    np.testing.assert_allclose(a["q"][1], before[1].q, atol=1e-12)
    assert a["sim_time"][0] == before[0].sim_time


def test_video_frames_match_rows_and_are_lossless(store):
    rec, _, before = _record(store, n=12)
    rec.finish("failure")
    d = store.dir("pick_place", rec.episode_id)
    info = probe_video(d / "frames.mkv")
    assert info["frames"] == 12
    frames = list(read_frames(d / "frames.mkv"))
    for k in range(12):
        np.testing.assert_array_equal(frames[k], before[k].image)   # FFV1 は可逆
    assert probe_video(d / "preview.mp4")["frames"] == 12


def test_meta_and_status(store):
    rec, _, _ = _record(store)
    meta = rec.finish("success", task_result={"success": True})
    assert meta["status"] == "success" and meta["quality"]["ok"]
    assert meta["joint_names"] == spec.names and meta["seed"] == 1
    assert not (store.dir("pick_place", rec.episode_id) / ".active").exists()


def test_crashed_recording_is_listed_as_aborted_not_deleted(store):
    rec, _, _ = _record(store, n=5)   # finish() を呼ばない = プロセス落ち相当
    (store.dir("pick_place", rec.episode_id) / ".active").unlink()
    items = store.list("pick_place")
    assert items[0]["status"] == "aborted" and "中断" in items[0]["abort_reason"]


def test_abort_keeps_partial_data(store):
    rec, _, _ = _record(store, n=7)
    meta = rec.abort("通信切断")
    assert meta["status"] == "aborted" and meta["num_frames"] == 7
    assert store.load_arrays("pick_place", rec.episode_id)["q"].shape == (7, 6)


def test_exclude_and_label_roundtrip(store):
    rec, _, _ = _record(store)
    rec.finish("failure")
    store.set_excluded("pick_place", rec.episode_id, True)
    store.set_label("pick_place", rec.episode_id, "success")
    m = store.load_meta("pick_place", rec.episode_id)
    assert m["excluded"] is True and m["label"] == "success" and m["status"] == "failure"
    with pytest.raises(ValueError):
        store.set_label("pick_place", rec.episode_id, "maybe")


def test_id_path_traversal_rejected(store):
    with pytest.raises(ValueError):
        store.dir("pick_place", "../../x")
    with pytest.raises(ValueError):
        store.dir("../evil", "abc")


def test_quality_detects_dropped_nonmonotonic_and_out_of_limits():
    n = 30
    base = dict(sim_time=np.arange(n) / 25.0, q=np.zeros((n, 6)), q_cmd=np.zeros((n, 6)),
                image_stamp=np.arange(n) / 25.0, joint_stamp=np.arange(n) / 25.0, step_wall_ms=np.full(n, 20.0))
    assert check_episode(base, 25.0, spec.lower, spec.upper)["ok"]
    t = base["sim_time"].copy()
    t[10:] += 0.12   # 3 周期ぶん欠落
    r = check_episode({**base, "sim_time": t}, 25.0, spec.lower, spec.upper)
    assert r["dropped_frames"] == 3 and any(i["code"] == "FRAME_DROP" for i in r["issues"])
    t = base["sim_time"].copy()
    t[5] = t[4]
    assert any(i["code"] == "TIMESTAMP_NONMONOTONIC" for i in check_episode({**base, "sim_time": t}, 25.0, spec.lower, spec.upper)["issues"])
    qc = base["q_cmd"].copy()
    qc[3, 0] = 9.0
    assert not check_episode({**base, "q_cmd": qc}, 25.0, spec.lower, spec.upper)["ok"]
    qn = base["q"].copy()
    qn[2, 1] = np.nan
    assert any(i["code"] == "NONFINITE" for i in check_episode({**base, "q": qn}, 25.0, spec.lower, spec.upper)["issues"])
    assert not check_episode({k: v[:0] for k, v in base.items()}, 25.0, spec.lower, spec.upper)["ok"]
