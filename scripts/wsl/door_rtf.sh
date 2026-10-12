#!/usr/bin/env bash
# ドア SDF を単体で Gazebo に載せて RTF と警告を確認する診断 (ブリッジ無し)。使い方: bash door_rtf.sh open_door|lever_door
set +u; source /opt/ros/humble/setup.bash; set -u
export IGN_PARTITION=doortest
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
TASK="${1:-open_door}"
export IGN_GAZEBO_RESOURCE_PATH="$ROOT/sim/models"
python3 - <<PY
import sys, yaml
sys.path.insert(0, "$ROOT/sim/bridge"); sys.path.insert(0, "$ROOT/src")
import worldgen
t = yaml.safe_load(open("$ROOT/config/tasks/$TASK.yaml", encoding="utf-8"))
open("/tmp/dw.sdf", "w").write(worldgen.make_world_sdf(t))
objs = worldgen.make_door_objects(t, {"door": {"hinge_x": 0.26, "hinge_y": -0.10}})
for n, sdf, pose in objs:
    open(f"/tmp/dw_{n}.sdf", "w").write(sdf)
    print(n, pose)
PY
ign gazebo -s -r --headless-rendering -v 3 /tmp/dw.sdf > /tmp/dw.log 2>&1 &
GZ=$!; sleep 12
for n in ${MODELS:-door_frame door}; do
  ign service -s /world/so101_world/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 5000 --req "sdf_filename: \"/tmp/dw_$n.sdf\" pose: {position: {x: 0.26, y: -0.10}}"
done
sleep 5
echo -n "RTF: "; timeout 4 ign topic -e -t /world/so101_world/stats -n 1 2>&1 | grep real_time_factor
echo "--- door joint state"; timeout 4 ign topic -e -t /world/so101_world/model/door/joint_state -n 1 2>&1 | grep -E "name|position:" | head -8
echo "--- warnings"; grep -iE "error|warn|fail|invalid" /tmp/dw.log | head -12
kill $GZ 2>/dev/null; sleep 2
