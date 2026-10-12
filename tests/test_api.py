"""API の結合テスト (mock バックエンド)。注意: これは Gazebo 連携のテストではない。"""
import time

import pytest
from fastapi.testclient import TestClient

from vrso101.api.app import create_app


@pytest.fixture
def client(tmp_path, monkeypatch):
    monkeypatch.setenv("VRSO101_DATA_DIR", str(tmp_path / "data"))
    app = create_app(backend="mock")
    with TestClient(app) as c:
        yield c


def wait(cond, timeout=15.0, step=0.05):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(step)
    return False


def frames(client):
    r = client.get("/api/sim/status").json()["recording"]
    return r["frames"] if r else 0


def test_system_and_tasks(client):
    s = client.get("/api/system").json()
    assert s["backend"] == "mock" and set(s["tasks"]) >= {"pick_place", "open_door", "lever_door"}
    tasks = {t["task_id"]: t for t in client.get("/api/tasks").json()}
    assert tasks["pick_place"]["implemented"] is True
    r = client.get("/api/robot").json()
    assert r["joint_names"][0] == "shoulder_pan" and r["units"] == "radians"


def test_unknown_task_and_invalid_input(client):
    r = client.post("/api/sim/task", json={"task_id": "nope"})
    assert r.status_code == 404 and r.json()["error"]["code"] == "UNKNOWN_TASK"
    assert client.post("/api/sim/reset", json={"seed": -1}).status_code == 422
    assert client.post("/api/datasets/export", json={"task_id": "pick_place", "name": "bad name!"}).status_code == 422
    assert client.get("/api/episodes/pick_place/..%2f..%2fx").status_code in (400, 404, 422)


def test_reset_before_task_is_clear_error(client):
    r = client.post("/api/sim/reset", json={})
    assert r.status_code == 409 and r.json()["error"]["code"] == "NO_TASK"


def test_record_episode_lifecycle(client):
    assert client.post("/api/sim/connect").status_code == 200
    s = client.post("/api/sim/task", json={"task_id": "pick_place"}).json()
    assert s["task_id"] == "pick_place" and s["mode"] == "idle"
    assert wait(lambda: client.get("/api/sim/frame.jpg").status_code == 200)
    r = client.post("/api/episodes/start", json={"seed": 7, "auto_stop": False}).json()
    assert r["mode"] == "recording"
    again = client.post("/api/episodes/start", json={}).json()    # 連打しても 1 本
    assert again.get("already_recording") is True
    assert wait(lambda: frames(client) >= 12)
    # 収録中はリセット/タスク切替を拒否
    assert client.post("/api/sim/reset", json={}).status_code == 409
    assert client.post("/api/sim/task", json={"task_id": "pick_place"}).status_code == 409
    out = client.post("/api/episodes/stop", json={}).json()
    assert out["stopped"] and out["episode"]["status"] in ("success", "failure")
    assert out["episode"]["num_frames"] >= 12 and out["episode"]["seed"] == 7
    assert client.post("/api/episodes/stop", json={}).json()["stopped"] is False   # 冪等
    items = client.get("/api/episodes", params={"task_id": "pick_place"}).json()
    assert items["stats"]["total"] == 1
    eid = items["items"][0]["episode_id"]
    assert client.get(f"/api/episodes/pick_place/{eid}/video").status_code == 200
    d = client.get(f"/api/episodes/pick_place/{eid}").json()
    assert d["trajectory"]["q"] and len(d["trajectory"]["q"][0]) == 6
    # ラベル・除外・削除 (失敗エピソードを自動削除せず利用者が決める)
    m = client.patch(f"/api/episodes/pick_place/{eid}", json={"label": "failure", "excluded": True}).json()
    assert m["label"] == "failure" and m["excluded"] is True
    assert client.patch(f"/api/episodes/pick_place/{eid}", json={"label": "maybe"}).status_code == 422
    assert client.delete(f"/api/episodes/pick_place/{eid}").status_code == 200
    assert client.delete(f"/api/episodes/pick_place/{eid}").status_code == 200   # 冪等


def test_discard_recording(client):
    client.post("/api/sim/task", json={"task_id": "pick_place"})
    client.post("/api/episodes/start", json={"auto_stop": False})
    assert wait(lambda: frames(client) >= 3)
    out = client.post("/api/episodes/stop", json={"discard": True}).json()
    assert out["episode"]["status"] == "aborted"
    assert client.get("/api/episodes").json()["items"][0]["status"] == "aborted"   # 消さずに残す


def test_teleop_via_websocket_moves_robot_and_stale_input_holds(client):
    client.post("/api/sim/task", json={"task_id": "pick_place"})
    with client.websocket_connect("/ws") as ws:
        hello = ws.receive_json()
        assert hello["type"] == "hello" and hello["session_id"]
        q0 = None
        for _ in range(30):
            ev = ws.receive_json()
            if ev["type"] == "state":
                q0 = ev["q"]
                break
        assert q0 is not None
        for _ in range(40):   # 前方へ EE を動かす (PC/タッチ入力)
            ws.send_json({"type": "nudge", "v": [0.05, 0, 0], "w": [0, 0, 0]})
            time.sleep(0.04)
        time.sleep(0.3)
        moved = None
        for _ in range(30):
            ev = ws.receive_json()
            if ev["type"] == "state":
                moved = ev["q"]
        assert max(abs(a - b) for a, b in zip(q0, moved)) > 0.02
        # 入力が途絶えたら保持 (追加の移動なし)
        time.sleep(0.8)
        qs = [e["q"] for e in [ws.receive_json() for _ in range(15)] if e["type"] == "state"]
        assert max(abs(a - b) for a, b in zip(qs[0], qs[-1])) < 0.02


def test_ws_events_have_seq_and_resync(client):
    client.post("/api/sim/task", json={"task_id": "pick_place"})
    with client.websocket_connect("/ws") as ws:
        seq0 = ws.receive_json()["seq"]
    client.post("/api/episodes/start", json={"auto_stop": False})
    assert wait(lambda: frames(client) >= 3)
    client.post("/api/episodes/stop", json={})
    with client.websocket_connect(f"/ws?since={seq0}") as ws:
        h = ws.receive_json()
        assert h["resync_required"] is False
        kinds = []
        for _ in range(12):
            ev = ws.receive_json()
            kinds.append(ev["type"])
            if ev["type"] == "state":
                break
        assert "episode" in kinds or "mode" in kinds   # 切断中のイベントが再送される


def test_export_job_runs_cli_and_dataset_listed(client):
    client.post("/api/sim/task", json={"task_id": "pick_place"})
    for seed in (1, 2):
        client.post("/api/episodes/start", json={"seed": seed, "auto_stop": False})
        assert wait(lambda: frames(client) >= 15)
        client.post("/api/episodes/stop", json={})
    for e in client.get("/api/episodes").json()["items"]:
        client.patch(f"/api/episodes/pick_place/{e['episode_id']}", json={"label": "success"})
    j = client.post("/api/datasets/export", json={"task_id": "pick_place", "name": "t1", "only_success": True}).json()
    assert j["kind"] == "export"
    assert wait(lambda: client.get(f"/api/jobs/{j['id']}").json()["state"] in ("succeeded", "failed"), timeout=120)
    jj = client.get(f"/api/jobs/{j['id']}").json()
    assert jj["state"] == "succeeded", jj["log_tail"]
    ds = client.get("/api/datasets").json()
    assert ds and ds[0]["name"] == "t1" and ds[0]["num_episodes"] == 2
    assert client.get("/api/datasets/pick_place/t1/validate").json()["ok"]
    # 同名の再エクスポートは拒否
    assert client.post("/api/datasets/export", json={"task_id": "pick_place", "name": "t1"}).status_code == 409


def test_training_precheck_errors_are_actionable(client):
    r = client.post("/api/training/start", json={"task_id": "pick_place", "dataset": "missing", "steps": 10})
    assert r.status_code == 400 and "見つかりません" in r.json()["error"]["message"]
    r = client.post("/api/training/start", json={"task_id": "pick_place", "dataset": "x", "steps": 10, "device": "tpu"})
    assert r.status_code == 422


def test_inference_start_unknown_model(client):
    r = client.post("/api/inference/start", json={"task_id": "pick_place", "model_id": "nope"})
    assert r.status_code in (404, 400)


def test_diagnostics_reports_bridge_down(client):
    d = client.get("/api/system/diagnostics").json()
    names = {i["name"]: i for i in d["items"]}
    assert "Gazebo ブリッジ (WSL)" in names and "PyTorch" in names


def test_token_guard(tmp_path, monkeypatch):
    monkeypatch.setenv("VRSO101_DATA_DIR", str(tmp_path / "data"))
    import vrso101.api.app as appmod
    monkeypatch.setattr(appmod, "TOKEN", "secret")
    with TestClient(create_app(backend="mock")) as c:
        assert c.get("/api/system").status_code == 200
        assert c.post("/api/sim/reset", json={}).status_code == 401
        assert c.post("/api/sim/reset", json={}, headers={"X-VRSO101-Token": "secret"}).status_code == 409
