"""スクリプトデモを Gazebo で実行して把持・運搬・配置が物理的に成立するかを確認する。

  .venv\\Scripts\\python.exe scripts/run_scripted.py --seeds 0 1 2 --frames data/scripted_frames
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vrso101.robot.joints import load_robot_spec  # noqa: E402
from vrso101.robot.kinematics import Kinematics  # noqa: E402
from vrso101.sim.client import SimClient  # noqa: E402
from vrso101.teleop.controller import TeleopController  # noqa: E402
from vrso101.teleop.scripted import ScriptedPickPlace  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[0])
    ap.add_argument("--frames", default="", help="フェーズ境界のフレームを保存するディレクトリ")
    a = ap.parse_args()
    spec = load_robot_spec()
    kin = Kinematics(spec)
    c = SimClient()
    c.connect()
    c.load_task("pick_place")
    ok = 0
    for seed in a.seeds:
        o = c.reset(seed=seed)
        cu, tg = o.scene["cube"], o.scene["target"]
        plan = ScriptedPickPlace((cu["x"], cu["y"], cu["z"]), cu["yaw"], (tg["x"], tg["y"]))
        ctl = TeleopController(spec, kin, rate_hz=25, smoothing=0.0)
        ctl.ik_rot_weight = 1.0
        ctl.workspace.lower[2] = -0.01
        ctl.reset(o.q)
        last, tick = None, 0
        for name, out in plan.run(ctl):
            o = c.step(out.q_target)
            tick += 1
            if a.frames and name != last:
                Path(a.frames).mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(Path(a.frames) / f"s{seed}_{tick:03d}_{name}.png"), cv2.cvtColor(o.image, cv2.COLOR_RGB2BGR))
            last = name
            if o.task.get("success"):
                pass
        for _ in range(15):   # 最終静止
            o = c.step(ctl.q)
        t = o.task
        ok += bool(t["success"])
        print(f"seed={seed} success={t['success']} detail={t['detail']} metrics={ {k: round(v, 4) for k, v in t['metrics'].items()} }")
        if a.frames:
            cv2.imwrite(str(Path(a.frames) / f"s{seed}_final.png"), cv2.cvtColor(o.image, cv2.COLOR_RGB2BGR))
    print(f"成功 {ok}/{len(a.seeds)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
