#!/usr/bin/env python3
"""lock-step 1 周期の所要時間を計る診断 (WSL で ROS を source して実行)。

  VRSO101_STEP_MODE=run_to|multi_step  VRSO101_SKIP=poses,joint_states  python3 bench_step.py
"""
import os
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

import worldgen  # noqa: E402
from gz_ros import GazeboRos  # noqa: E402
from vrso101.robot.joints import load_robot_spec  # noqa: E402
from vrso101.tasks.registry import get_task  # noqa: E402

spec = load_robot_spec()
gz = GazeboRos(ROOT / "config" / "sim" / "gazebo.yaml", spec.names, ROOT)
task = get_task("pick_place")
sub = gz.substeps
dt = 1.0 / (task["fps"] * sub)
gz.start_world(worldgen.make_world_sdf(task, substeps=sub, engine="ogre"), "bench", task["camera"]["name"])
print("mode", gz.step_mode, "skip", os.environ.get("VRSO101_SKIP"))
gz.world_control(pause=True)
gz.publish_cmd(spec.home)
from vrso101.tasks.registry import sample_scene  # noqa: E402
if os.environ.get("RESET"):
    gz.world_control(pause=True, reset={"model_only": True})
if os.environ.get("SPAWN"):
    for name, sdf, pose6 in worldgen.make_task_objects(task, sample_scene(task, 1)):
        print("spawn", name, gz.spawn(name, sdf, pose6))
    gz._clear_obs()
walls = []
for i in range(int(os.environ.get("TICKS", "30"))):
    t0 = time.perf_counter()
    q = spec.home.copy()
    if os.environ.get("MOVE"):
        q = spec.home + np.array([0.4 * np.sin(i / 10), 0.2 * np.sin(i / 7), 0, 0, 0, 0.5 + 0.5 * np.sin(i / 9)])
    gz.publish_cmd(q)
    gz.step(sub, dt)
    walls.append((time.perf_counter() - t0) * 1000)
print(f"tick ms median={statistics.median(walls):.1f} min={min(walls):.1f} max={max(walls):.1f} last_prof={gz.prof}")
gz.stop_world()
