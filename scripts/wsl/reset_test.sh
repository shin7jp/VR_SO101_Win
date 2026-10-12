#!/usr/bin/env bash
# lock-step の前提を ign CLI で実測: (1) 停止状態から multi_step が効くか (2) 実行時 spawn (3) reset が spawn 物体をどう扱うか
set +u; source /opt/ros/humble/setup.bash; set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export IGN_GAZEBO_RESOURCE_PATH="$ROOT/sim/models"
W=/tmp/so101_test.sdf
python3 - <<PY
import sys, yaml
sys.path.insert(0, "$ROOT/sim/bridge")
import worldgen
t = yaml.safe_load(open("$ROOT/config/tasks/pick_place.yaml", encoding="utf-8"))
open("$W", "w").write(worldgen.make_world_sdf(t))
sc = {"cube": {"x": 0.2, "y": 0.0, "z": 0.015, "yaw": 0.3}}
open("/tmp/cube.sdf", "w").write(worldgen.make_pick_place_objects(t, sc)[1][1])
PY
ign gazebo -s --headless-rendering -v 2 "$W" > /tmp/reset_gz.log 2>&1 &   # -r なし = 停止状態で起動
GZ=$!
trap 'kill $GZ 2>/dev/null; sleep 1 2>/dev/null; true' EXIT
sleep 12
svc() { ign service -s /world/so101_world/$1 --reqtype $2 --reptype ignition.msgs.Boolean --timeout 3000 --req "$3"; }
simt() { timeout 3 ign topic -e -t /world/so101_world/stats -n 1 2>&1 | awk '/sim_time/{f=1} f&&/sec:/{printf "%s.",$2} f&&/nsec:/{print $2; exit}'; }
echo "t0 (paused):"; simt; sleep 1; echo "1s later (should be same):"; simt
echo "multi_step 10:"; svc control ignition.msgs.WorldControl 'multi_step: 10'; sleep 1; simt
echo "spawn cube:"; svc create ignition.msgs.EntityFactory "sdf_filename: \"/tmp/cube.sdf\""; sleep 0.5
svc control ignition.msgs.WorldControl 'multi_step: 60'; sleep 1
echo "cube pose after 60 steps:"; timeout 3 ign topic -e -t /world/so101_world/dynamic_pose/info -n 1 2>&1 | grep -B2 -A8 'name: "cube"' | head -24
echo "reset all:"; svc control ignition.msgs.WorldControl 'reset: {all: true}'; sleep 1; simt
svc control ignition.msgs.WorldControl 'multi_step: 5'; sleep 1
echo "entities after reset (cube present?):"; timeout 3 ign topic -e -t /world/so101_world/dynamic_pose/info -n 1 2>&1 | grep 'name:' | head -20
echo "log:"; grep -iE "error|fail" /tmp/reset_gz.log | head
