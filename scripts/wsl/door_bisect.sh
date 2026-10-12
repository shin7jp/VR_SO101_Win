#!/usr/bin/env bash
# ドア/レバーが Gazebo を極端に遅くする原因の切り分け。バリアントごとに RTF を測る。使い方: TASK=lever_door bash door_bisect.sh
# ロボットの初期姿勢はタスクの home (パン -0.9) に揃える: 既定の home だと腕が板に食い込み、それ自体が遅さの原因になる。
set +u; source /opt/ros/humble/setup.bash; set -u
export IGN_PARTITION=doortest2
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
TASK="${TASK:-lever_door}"
rm -rf /tmp/models_door && cp -r "$ROOT/sim/models" /tmp/models_door
python3 - <<PY
import re
p = "/tmp/models_door/so101/model.sdf"
s = open(p, encoding="utf-8").read()
s = re.sub(r"(<joint_name>shoulder_pan</joint_name>.*?<initial_position>)[^<]*", r"\g<1>-0.9", s, flags=re.S)
open(p, "w", encoding="utf-8").write(s)
PY
export IGN_GAZEBO_RESOURCE_PATH=/tmp/models_door
python3 - <<PY
import sys, yaml, re
sys.path.insert(0, "$ROOT/sim/bridge"); sys.path.insert(0, "$ROOT/src")
import worldgen
t = yaml.safe_load(open("$ROOT/config/tasks/$TASK.yaml", encoding="utf-8"))
open("/tmp/dw.sdf", "w").write(worldgen.make_world_sdf(t))
objs = dict((n, s) for n, s, p in worldgen.make_door_objects(t, {"door": {"hinge_x": 0.26, "hinge_y": -0.10}}))
d, f = objs["door"], objs["door_frame"]
def nocol(s, names):
    return re.sub(r'<collision name="(%s)_c">.*?</collision>' % names, "", s, flags=re.S)
V = {
  "full": (d, f),
  "no_keeper_plate": (d, nocol(f, "keeper_plate")),
  "no_keeper_post": (d, nocol(f, "keeper_post")),
  "no_frame_collision": (d, nocol(f, "hinge_post|keeper_plate|keeper_post")),
  "no_arm_collision": (nocol(d, "arm"), f),
  "no_lever_plugin": (re.sub(r"<plugin filename=\"ignition-gazebo-joint-position.*?</plugin>", "", d, flags=re.S), f),
  "no_panel_collision": (nocol(d, "panel"), f),
}
for k, (a, b) in V.items():
    open(f"/tmp/dv_{k}_door.sdf", "w").write(a)
    open(f"/tmp/dv_{k}_frame.sdf", "w").write(b)
print(list(V))
PY
for V in full no_keeper_plate no_keeper_post no_frame_collision no_arm_collision no_lever_plugin no_panel_collision; do
  ign gazebo -s -r --headless-rendering -v 1 /tmp/dw.sdf > /tmp/dw.log 2>&1 &
  GZ=$!; sleep 10
  for part in frame door; do
    ign service -s /world/so101_world/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 5000 --req "sdf_filename: \"/tmp/dv_${V}_${part}.sdf\" pose: {position: {x: 0.26, y: -0.10}}" > /dev/null
  done
  sleep 4
  echo -n "$V: "; timeout 5 ign topic -e -t /world/so101_world/stats -n 1 2>&1 | grep real_time_factor || echo "(no stats: hung)"
  kill $GZ 2>/dev/null; sleep 2
done
