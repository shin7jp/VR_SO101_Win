"""Gazebo ブリッジの疎通・同期・再現性を実測する (WSL で start_bridge.sh 起動済みであること)。

  .venv\\Scripts\\python.exe scripts/smoke_sim.py [--task pick_place] [--ticks 60]
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vrso101.robot.joints import load_robot_spec  # noqa: E402
from vrso101.sim.client import SimClient  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="pick_place")
    ap.add_argument("--ticks", type=int, default=60)
    ap.add_argument("--save", default="")
    a = ap.parse_args()
    spec = load_robot_spec()
    c = SimClient()
    print("hello:", c.connect())
    t0 = time.time()
    print("load_task:", c.load_task(a.task), f"({time.time() - t0:.1f}s)")
    o = c.reset(seed=3)
    print("reset: scene=", o.scene, "\n  q=", np.round(o.q, 3), "img", o.image.shape, "sim_time", o.sim_time)

    def rollout():
        o = c.reset(seed=3)
        traj, walls = [], []
        q = spec.home.copy()
        for i in range(a.ticks):
            q = spec.home + np.array([0.4 * np.sin(i / 10), 0.2 * np.sin(i / 7), 0, 0, 0, 0.5 + 0.5 * np.sin(i / 9)])
            o = c.step(q)
            traj.append(o.q.copy())
            walls.append(o.step_wall_ms)
            assert 0 <= o.joint_stamp - o.image_stamp < 0.0051, f"画像と関節のスタンプ差が大きい: {o.joint_stamp - o.image_stamp}"
        return np.array(traj), walls, o

    tr1, w1, o1 = rollout()
    tr2, w2, o2 = rollout()
    print(f"tick wall ms: median={statistics.median(w1):.1f} p95={sorted(w1)[int(len(w1) * .95)]:.1f}"
          f"  -> {1000 / statistics.median(w1):.1f} ticks/s")
    print("再現性 (同一指令列 2 回の最大関節差 [rad]):", float(np.abs(tr1 - tr2).max()))
    print("task:", o2.task)
    print("prof(last step):", o2.prof)
    if a.save:
        import cv2
        cv2.imwrite(a.save, cv2.cvtColor(o2.image, cv2.COLOR_RGB2BGR))
        print("saved", a.save)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
