"""スクリプトドアデモをトレース: 各ステップの所要時間・EE 位置・ドア/レバー角を出力 (遅くなる/固まる瞬間の特定用)。

  VRSO101_BRIDGE_PORT=9101 .venv\\Scripts\\python.exe scripts/door_trace.py --task open_door --seed 20000
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vrso101.robot.joints import load_robot_spec  # noqa: E402
from vrso101.robot.kinematics import Kinematics  # noqa: E402
from vrso101.sim.client import SimClient  # noqa: E402
from vrso101.tasks.registry import get_task  # noqa: E402
from vrso101.runner import scripted_actions  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--task", default="open_door")
ap.add_argument("--seed", type=int, default=20000)
ap.add_argument("--every", type=int, default=10)
ap.add_argument("--max", type=int, default=400)
ap.add_argument("--frames", default="")
a = ap.parse_args()
spec = load_robot_spec()
kin = Kinematics(spec)
c = SimClient()
c.connect()
c.load_task(a.task)
o = c.reset(seed=a.seed, randomize=False)
print("scene", o.scene)
fps = get_task(a.task)["fps"]
n = 0
for q in scripted_actions(spec, kin, a.task, o, fps):
    try:
        o = c.step(q)
    except Exception as e:
        print("STEP FAIL at", n, e)
        break
    n += 1
    if n % a.every == 0 or (o.step_wall_ms or 0) > 150:
        ee = kin.fk(o.q)[:3, 3]
        m = o.task.get("metrics", {})
        print(f"{n:4d} wall={o.step_wall_ms:6.1f}ms ee={np.round(ee, 3)} gripper={o.q[-1]:.2f} door={o.ext_joints.get('door_hinge', 0):+.3f} lever={o.ext_joints.get('lever', 0):+.3f} succ={o.task.get('success')}")
    if a.frames and (n % 20 == 0 and n >= 100):
        import cv2
        Path(a.frames).mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(Path(a.frames) / f"{a.task}_{n:04d}.png"), cv2.cvtColor(o.image, cv2.COLOR_RGB2BGR))
    if n >= a.max:
        break
print("final", o.task)
