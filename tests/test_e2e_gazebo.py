"""実 Gazebo + 実 ACT の E2E (CI のモックテストとは別物)。

シナリオ (仕様 17 章): Gazebo の SO-101 を起動 → Pick and Place 初期化 → テレオペで短いエピソードを収録 →
LeRobot データセット生成・読込検証 → ACT スモーク学習 → チェックポイントをロード → Gazebo 上で推論 →
推論動画とメタデータ保存 → 保存された動画を読み込み正常性を確認。

実行条件: WSL のブリッジが起動していること (別クライアントが使用中でないこと) と CUDA。満たさなければ skip。
  $env:VRSO101_E2E = 1; $env:VRSO101_BRIDGE_PORT = 9101   # 本番ブリッジが収録などで使用中なら開発用 9101 を使う
  .\\.venv\\Scripts\\python.exe -m pytest tests/test_e2e_gazebo.py -v -s
"""
import os
import socket
import time

import pytest

pytestmark = [pytest.mark.gazebo, pytest.mark.gpu]


def _bridge_up() -> bool:
    port = int(os.environ.get("VRSO101_BRIDGE_PORT", 9100))
    try:
        socket.create_connection(("localhost", port), timeout=1).close()
        return True
    except OSError:
        return False


def _cuda() -> bool:
    try:
        import torch
        return torch.cuda.is_available()
    except Exception:
        return False


if os.environ.get("VRSO101_E2E") != "1" or not _bridge_up() or not _cuda():
    pytest.skip("VRSO101_E2E=1 + WSL ブリッジ + CUDA が必要 (実 Gazebo/ACT の E2E)", allow_module_level=True)

from fastapi.testclient import TestClient  # noqa: E402

from vrso101.api.app import create_app  # noqa: E402
from vrso101.video import probe_video  # noqa: E402


def wait(cond, timeout=120.0, step=0.1):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = cond()
        if v:
            return v
        time.sleep(step)
    return None


def test_full_pipeline_on_real_gazebo(tmp_path, monkeypatch):
    monkeypatch.setenv("VRSO101_DATA_DIR", str(tmp_path / "data"))
    with TestClient(create_app(backend="gazebo")) as c:
        # 1-2. Gazebo 起動済みのシムへ接続し、Pick and Place を初期化
        assert c.post("/api/sim/connect").json()["connected"]
        s = c.post("/api/sim/task", json={"task_id": "pick_place"}).json()
        assert s["task_id"] == "pick_place" and not s["is_mock"]
        assert wait(lambda: c.get("/api/sim/frame.jpg").status_code == 200)
        # 3. テレオペ (WebSocket の EE 速度入力) で短いエピソードを 2 本収録
        with c.websocket_connect("/ws") as ws:
            ws.receive_json()
            for seed in (11, 12):
                assert c.post("/api/episodes/start", json={"seed": seed, "auto_stop": False}).json()["mode"] == "recording"
                for _ in range(30):
                    ws.send_json({"type": "nudge", "v": [0.04, 0.0, -0.02], "w": [0, 0, 0], "gripper": 0.5})
                    time.sleep(0.05)
                out = c.post("/api/episodes/stop", json={}).json()
                assert out["stopped"] and out["episode"]["num_frames"] >= 20
                # 収録 → 人手評価 (パイプライン検証用に成功ラベルを付ける)
                c.patch(f"/api/episodes/pick_place/{out['episode']['episode_id']}", json={"label": "success"})
        # 4. LeRobot データセット生成 → 読込検証
        j = c.post("/api/datasets/export", json={"task_id": "pick_place", "name": "e2e"}).json()
        assert wait(lambda: c.get(f"/api/jobs/{j['id']}").json()["state"] in ("succeeded", "failed"), 180)
        assert c.get(f"/api/jobs/{j['id']}").json()["state"] == "succeeded", c.get(f"/api/jobs/{j['id']}").json()["log_tail"]
        assert c.get("/api/datasets/pick_place/e2e/validate").json()["ok"]
        # 5-6. ACT スモーク学習 → チェックポイント保存
        t = c.post("/api/training/start", json={"task_id": "pick_place", "dataset": "e2e", "steps": 20, "batch_size": 4,
                                                  "save_freq": 20, "num_workers": 0, "device": "cuda", "name": "e2e"}).json()
        assert wait(lambda: c.get(f"/api/jobs/{t['id']}").json()["state"] in ("succeeded", "failed"), 600)
        tj = c.get(f"/api/jobs/{t['id']}").json()
        assert tj["state"] == "succeeded", tj["log_tail"]
        models = c.get("/api/models", params={"task_id": "pick_place"}).json()
        assert models and models[0]["status"] == "trained" and models[0]["checkpoints"][-1]["ok"]
        mid = models[0]["model_id"]
        # 7-8. Gazebo 上で推論 → 動画 + メタデータ保存
        r = c.post("/api/inference/start", json={"task_id": "pick_place", "model_id": mid, "checkpoints": ["last"],
                                                   "seeds": [10000], "max_steps": 25, "n_action_steps": 5}).json()
        assert wait(lambda: c.get(f"/api/jobs/{r['id']}").json()["state"] in ("succeeded", "failed", "stopped"), 600)
        ij = c.get(f"/api/jobs/{r['id']}").json()
        assert ij["state"] == "succeeded", ij["log_tail"]
        ep = ij["result"]["episodes"][0]
        assert ep["saved"] and ep["frames"] >= 20
        # 9. 保存された動画を読み込み、正常であることを確認 (UI が『保存済み』を出す条件と同じ)
        meta = c.get(f"/api/inference/videos/pick_place/{ep['episode_id']}").json()
        assert meta["saved"] and meta["policy"]["model_id"] == mid
        assert meta["evaluation_split"] == "eval" and meta["video"]["codec"] == "h264"
        info = probe_video(tmp_path / "data" / "inference" / "pick_place" / ep["episode_id"] / "inference.mp4")
        assert info["frames"] == meta["frames"] and info["width"] == 640
        assert c.get(f"/api/inference/videos/pick_place/{ep['episode_id']}/video").status_code == 200
        # 推論後はシムの所有権が API へ戻り、再びテレオペできる
        assert wait(lambda: c.get("/api/sim/status").json()["mode"] == "idle", 180)
