#!/usr/bin/env bash
# 立方体の有無・衝突の有無で実時間係数 (RTF) がどう変わるかを比べる診断スクリプト。
set +u; source /opt/ros/humble/setup.bash; set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export IGN_GAZEBO_RESOURCE_PATH="$ROOT/sim/models"
python3 - <<PY
import sys, yaml
sys.path.insert(0, "$ROOT/sim/bridge")
import worldgen
t = yaml.safe_load(open("$ROOT/config/tasks/pick_place.yaml", encoding="utf-8"))
open("/tmp/w.sdf", "w").write(worldgen.make_world_sdf(t))
sc = {"cube": {"x": 0.2, "y": 0.0, "z": 0.015, "yaw": 0.3}}
sdf = worldgen.make_pick_place_objects(t, sc)[1][1]
open("/tmp/cubeA.sdf", "w").write(sdf)
open("/tmp/cubeB.sdf", "w").write(sdf.replace("<collision", "<collision_off").replace("</collision>", "</collision_off>"))
PY
for V in none cubeA cubeB; do
  ign gazebo -s -r --headless-rendering -v 1 /tmp/w.sdf > /tmp/rtf.log 2>&1 &
  GZ=$!; sleep 10
  if [ $V != none ]; then
    ign service -s /world/so101_world/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 3000 --req "sdf_filename: \"/tmp/$V.sdf\"" > /dev/null
  fi
  sleep 4
  echo -n "$V: "; timeout 4 ign topic -e -t /world/so101_world/stats -n 1 2>&1 | grep real_time_factor
  kill $GZ; sleep 2; sleep 1
done
