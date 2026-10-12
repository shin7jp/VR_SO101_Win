"""推論経路と学習経路の食い違い検査 (診断)。

同じエピソードの同じフレームを (A) LeRobotDataset (学習時のデコード経路) と (B) 生エピソード (frames.mkv + states.npz → ACTRunner.act、
推論時の経路) で ACT に通し、出力アクションを比べる。差が大きければ画像の色順序/前処理/正規化の食い違いを疑う。
  .\\.venv\\Scripts\\python.exe scripts/check_inference_path.py --task pick_place --dataset scripted50 --model <model_id> --ckpt 015000
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: E402
from lerobot.policies.factory import make_pre_post_processors  # noqa: E402

from vrso101.inference.runtime import IMAGE_KEY, ACTRunner  # noqa: E402
from vrso101.models.registry import model_dir, resolve_checkpoint  # noqa: E402
from vrso101.paths import data_dir  # noqa: E402
from vrso101.recording.store import EpisodeStore  # noqa: E402
from vrso101.robot.joints import load_robot_spec  # noqa: E402
from vrso101.sim.client import Observation  # noqa: E402
from vrso101.video import read_frames  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--task", default="pick_place")
ap.add_argument("--dataset", required=True)
ap.add_argument("--model", required=True)
ap.add_argument("--ckpt", default="last")
ap.add_argument("--frames", type=int, nargs="+", default=[0, 60, 120, 200])
a = ap.parse_args()

spec = load_robot_spec()
root = data_dir() / "datasets" / a.task / a.dataset
vm = json.loads((root / "vrso101_meta.json").read_text(encoding="utf-8"))
ep = vm["episodes"][0]
ds = LeRobotDataset(f"local/{a.task}_{a.dataset}", root=root, video_backend="pyav")
ckpt = resolve_checkpoint(model_dir(a.task, a.model), a.ckpt)
runner = ACTRunner(ckpt, spec, device="cuda", n_action_steps=10)
pre, post = make_pre_post_processors(runner.policy.config, pretrained_path=str(ckpt),
                                     preprocessor_overrides={"device_processor": {"device": "cuda"}})
store = EpisodeStore()
arr = store.load_arrays(a.task, ep["episode_id"])
raw = list(read_frames(store.dir(a.task, ep["episode_id"]) / "frames.mkv"))
print("episode", ep["episode_id"], "frames", len(raw))
for k in a.frames:
    item = ds[k]
    with torch.inference_mode():
        runner.reset()
        batch = pre({IMAGE_KEY: item[IMAGE_KEY].float(), "observation.state": item["observation.state"].float()})
        act_ds = post(runner.policy.select_action(batch)).float().cpu().numpy().reshape(-1)
    obs = Observation(sim_time=0, step=0, q=arr["q"][k], qd=np.zeros(6), q_cmd=arr["q_cmd"][k], image=raw[k],
                      joint_stamp=0, image_stamp=0)
    runner.reset()
    act_rt = runner.act(obs)
    img_diff = float((item[IMAGE_KEY].numpy() * 255 - raw[k].transpose(2, 0, 1)).__abs__().mean())
    print(f"frame {k}: |A(dataset)-B(runtime)|max={np.abs(act_ds - act_rt).max():.4f} rad  "
          f"|B - recorded action|max={np.abs(act_rt - arr['q_cmd'][k]).max():.4f} rad  mean|img diff|={img_diff:.2f}/255")
