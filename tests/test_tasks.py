import pytest

from vrso101.tasks.registry import get_task, load_tasks, reachable, sample_scene
from vrso101.tasks.success import TaskEvaluator


def test_three_tasks_registered():
    assert set(load_tasks()) == {"pick_place", "open_door", "lever_door"}


def test_unknown_task_message():
    with pytest.raises(KeyError, match="未知のタスク"):
        get_task("nope")


def test_scene_sampling_is_seeded_and_reachable():
    cfg = get_task("pick_place")
    assert sample_scene(cfg, 7) == sample_scene(cfg, 7)
    assert sample_scene(cfg, 7) != sample_scene(cfg, 8)
    assert all(reachable(sample_scene(cfg, s)) for s in range(200))
    for t in ("open_door", "lever_door"):
        assert all(reachable(sample_scene(get_task(t), s)) for s in range(100))


def test_scene_without_randomization_is_deterministic_center():
    cfg = get_task("pick_place")
    a, b = sample_scene(cfg, 1, randomize=False), sample_scene(cfg, 99, randomize=False)
    a.pop('seed'); b.pop('seed')
    assert a == b


def _pp_state(x, y, z):
    return {"objects": {"cube": {"pos": [x, y, z]}}, "joints": {}}


def test_pick_place_requires_lift_before_place():
    cfg = get_task("pick_place")
    ev = TaskEvaluator(cfg)
    tx, ty = cfg["scene"]["target"]["center"]
    for _ in range(40):                       # 最初から目標内に置かれているだけ → 成功にしない
        ev.update(_pp_state(tx, ty, 0.015))
    assert not ev.success
    ev.reset()
    ev.update(_pp_state(0.2, 0.0, 0.015))
    ev.update(_pp_state(0.2, 0.0, 0.08))      # 持ち上げ
    for _ in range(20):
        ev.update(_pp_state(tx + 0.01, ty, 0.015))
    assert ev.success


def test_pick_place_outside_zone_fails():
    cfg = get_task("pick_place")
    ev = TaskEvaluator(cfg)
    ev.update(_pp_state(0.2, 0.0, 0.08))
    for _ in range(40):
        ev.update(_pp_state(0.2, -0.1, 0.015))
    assert not ev.success


def test_pick_place_success_is_latched():
    cfg = get_task("pick_place")
    ev = TaskEvaluator(cfg)
    tx, ty = cfg["scene"]["target"]["center"]
    ev.update(_pp_state(0.2, 0.0, 0.08))
    for _ in range(20):
        ev.update(_pp_state(tx, ty, 0.015))
    assert ev.success
    ev.update(_pp_state(0.3, 0.3, 0.015))
    assert ev.success


def test_open_door_threshold():
    ev = TaskEvaluator(get_task("open_door"))
    for _ in range(30):
        ev.update({"objects": {}, "joints": {"door_hinge": 0.3}})
    assert not ev.success
    for _ in range(15):
        ev.update({"objects": {}, "joints": {"door_hinge": 0.7}})
    assert ev.success


def test_lever_door_requires_lever_first():
    cfg = get_task("lever_door")
    ev = TaskEvaluator(cfg)
    for _ in range(30):   # レバーを回さずドアだけ開いた (ラッチ迂回などの異常) → 成功にしない
        ev.update({"objects": {}, "joints": {"lever": 0.0, "door_hinge": 0.8}})
    assert not ev.success
    ev.reset()
    ev.update({"objects": {}, "joints": {"lever": 0.6, "door_hinge": 0.0}})
    assert ev.last.metrics["latch_released"] == 1.0
    for _ in range(15):
        ev.update({"objects": {}, "joints": {"lever": 0.0, "door_hinge": 0.8}})
    assert ev.success
