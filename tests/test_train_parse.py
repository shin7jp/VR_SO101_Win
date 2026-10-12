from vrso101.training.trainer import TrainConfig, parse_train_line

LOG = ("INFO 2026-10-10 21:49:42 ot_train.py:641 step:20 smpl:80 ep:0 epch:0.12 loss:11.349 grdn:327.358 lr:1.0e-05 "
       "updt_s:0.069 data_s:0.089 smp/s:25 mem_gb:1.35 l1_loss:0.744 kld_loss:1.060")
BIG = LOG.replace("step:20", "step:6K").replace("smpl:80", "smpl:50K")


def job():
    return {"progress": {"step": 0, "total": 15000}, "metrics": []}


def test_parse_loss_line():
    j = job()
    parse_train_line(LOG, j)
    m = j["metrics"][0]
    assert m["step"] == 20 and m["loss"] == 11.349 and m["l1_loss"] == 0.744 and m["kld_loss"] == 1.06
    assert j["progress"]["step"] == 20


def test_big_step_uses_tqdm_progress_for_exact_step():
    j = job()
    parse_train_line("Training:  42%|████▏     | 6293/15000 [10:01<16:17,  8.91step/s]", j)
    parse_train_line(BIG, j)
    assert j["metrics"][0]["step"] == 6293 and j["progress"]["step"] == 6293


def test_garbage_lines_are_ignored():
    j = job()
    for line in ("", "Traceback", "step:abc loss:x", "[libx264 @ 0x1] kb/s:1452"):
        parse_train_line(line, j)
    assert j["metrics"] == []


def test_train_config_validation():
    import pytest
    with pytest.raises(Exception):
        TrainConfig(task_id="pick_place", dataset="d", device="tpu")
    with pytest.raises(Exception):
        TrainConfig(task_id="pick_place", dataset="d", name="bad name!")
    with pytest.raises(Exception):
        TrainConfig(task_id="pick_place", dataset="d", steps=0)
    assert TrainConfig(task_id="pick_place", dataset="d", device="cuda:1").device == "cuda:1"
