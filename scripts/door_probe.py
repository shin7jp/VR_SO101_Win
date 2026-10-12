"""ドアシーンの確認: 読み込み → リセット → 数ステップ → 画像保存 + 物理状態 (ドア角/レバー角) を表示。

  .venv\\Scripts\\python.exe scripts/door_probe.py --task lever_door --port 9101 --save data/door.png
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vrso101.robot.joints import load_robot_spec  # noqa: E402
from vrso101.sim.client import SimClient  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--task", default="open_door")
ap.add_argument("--port", type=int, default=9101)
ap.add_argument("--save", default="")
ap.add_argument("--ticks", type=int, default=20)
a = ap.parse_args()
spec = load_robot_spec()
c = SimClient(port=a.port)
print(c.connect())
print(c.load_task(a.task))
o = c.reset(seed=1, randomize=False)
print("scene", o.scene)
print("task", o.task)
print("objects", {k: [round(x, 3) for x in v["pos"]] for k, v in o.objects.items()})
for _ in range(a.ticks):
    o = c.step(o.q_cmd)
print("after", a.ticks, "ticks:", o.task)
print("objects", {k: [round(x, 3) for x in v["pos"]] for k, v in o.objects.items()})
if a.save:
    cv2.imwrite(a.save, cv2.cvtColor(o.image, cv2.COLOR_RGB2BGR))
