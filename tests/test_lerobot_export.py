import numpy as np
import pytest

pytest.importorskip("lerobot")

from vrso101.dataset.lerobot_export import CAMERA_KEY, export_lerobot, select_episodes, validate_dataset  # noqa: E402
from vrso101.recording.recorder import EpisodeRecorder  # noqa: E402
from vrso101.recording.store import EpisodeStore  # noqa: E402
from vrso101.robot.joints import load_robot_spec  # noqa: E402
from vrso101.sim.mock import MockSim  # noqa: E402

spec = load_robot_spec()


def _make(store, seed, n=15, status="success", mode="scripted"):
    sim = MockSim(spec)
    sim.load_task("pick_place")
    obs = sim.reset(seed=seed)
    rec = EpisodeRecorder(store, spec, "pick_place", 25.0, meta={"seed": seed, "collection_mode": mode})
    rec.prime(obs)
    for k in range(n):
        obs = sim.step(spec.home + np.array([0.02 * k, 0, 0, 0, 0, 0.01 * k]))
        rec.add_step(obs)
    return rec.finish(status)


@pytest.fixture
def store(tmp_path):
    s = EpisodeStore(tmp_path / "episodes")
    for seed in (1, 2, 3):
        _make(s, seed)
    _make(s, 4, status="failure")
    _make(s, 5, status="aborted")
    return s


def test_select_excludes_failed_aborted_and_user_excluded(store):
    eps = store.list("pick_place")
    excl = [e for e in eps if e["seed"] == 3][0]
    store.set_excluded("pick_place", excl["episode_id"], True)
    used, skipped = select_episodes(store, "pick_place")
    assert sorted(m["seed"] for m in used) == [1, 2]
    reasons = {s["seed"]: s["_skip_reason"] for s in skipped}
    assert reasons[3] == "利用者が除外" and reasons[4] == "成功ではない" and "aborted" in reasons[5]


def test_export_roundtrip_and_validate(store, tmp_path):
    root = tmp_path / "ds"
    meta = export_lerobot(store, spec, "pick_place", root, name="t1")
    assert meta["num_episodes"] == 3 and meta["num_frames"] == 45 and meta["num_skipped"] == 2
    assert meta["units"] == "radians" and meta["joint_names"] == spec.names
    res = validate_dataset(root, spec)
    assert res["ok"], res["problems"]
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset("local/pick_place", root=root, video_backend="pyav")
    assert len(ds) == 45 and list(ds.meta.features["action"]["names"]) == spec.names
    # 1 フレーム目の action/state が収録値と一致 (float32 丸めの範囲)
    arr = store.load_arrays("pick_place", used_first(store))
    item = ds[0]
    np.testing.assert_allclose(item["action"].numpy(), arr["q_cmd"][0], atol=1e-6)
    np.testing.assert_allclose(item["observation.state"].numpy(), arr["q"][0], atol=1e-6)
    img = item[CAMERA_KEY].numpy()
    assert img.shape[0] == 3 and 0.0 <= img.min() and img.max() <= 1.0


def used_first(store):
    used, _ = select_episodes(store, "pick_place")
    return used[0]["episode_id"]


def test_export_refuses_overwrite_and_empty(store, tmp_path):
    root = tmp_path / "ds2"
    export_lerobot(store, spec, "pick_place", root, name="t2")
    with pytest.raises(FileExistsError):
        export_lerobot(store, spec, "pick_place", root, name="t2")
    with pytest.raises(ValueError, match="変換できるエピソードがありません"):
        export_lerobot(store, spec, "pick_place", tmp_path / "ds3", name="t3", collection_mode="human")


def test_validate_detects_joint_mismatch(store, tmp_path):
    root = tmp_path / "ds4"
    export_lerobot(store, spec, "pick_place", root, name="t4")
    import dataclasses
    from vrso101.robot.joints import RobotSpec
    raw = {"robot_id": "x", "units": "radians", "urdf": spec.urdf, "base_link": spec.base_link,
           "end_effector_link": spec.end_effector_link, "home": list(spec.home),
           "joints": [dict(name=("renamed" if j.name == "wrist_roll" else j.name), lower=j.lower, upper=j.upper,
                           vel_limit=j.vel_limit) for j in spec.joints]}
    res = validate_dataset(root, RobotSpec(raw))
    assert not res["ok"] and any("関節定義" in p for p in res["problems"])
