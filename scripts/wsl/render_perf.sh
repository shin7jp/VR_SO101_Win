#!/usr/bin/env bash
# カメラ購読中の「実描画フレーム数/秒」と RTF を、ワールド構成ごとに比べる診断。
set +u; source /opt/ros/humble/setup.bash; set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export IGN_GAZEBO_RESOURCE_PATH="$ROOT/sim/models"
python3 - <<PY
import sys, yaml
sys.path.insert(0, "$ROOT/sim/bridge")
import worldgen
t = yaml.safe_load(open("$ROOT/config/tasks/pick_place.yaml", encoding="utf-8"))
w = worldgen.make_world_sdf(t)
open("/tmp/w.sdf", "w").write(w)
open("/tmp/w_noshadow.sdf", "w").write(w.replace("<cast_shadows>true", "<cast_shadows>false"))
t["camera"]["width"], t["camera"]["height"] = 320, 240
open("/tmp/w_small.sdf", "w").write(worldgen.make_world_sdf(t))
# ロボット無し (ground+camera だけ): メッシュが重いのか切り分け
import re
open("/tmp/w_norobot.sdf", "w").write(re.sub(r"<include>.*?</include>", "", w, flags=re.S))
PY
run() {  # $1=label $2=world
  ign gazebo -s -r --headless-rendering -v 1 "$2" > /tmp/rp.log 2>&1 &
  local GZ=$!; sleep 12
  local N; N=$(timeout 12 ign topic -e -t /camera/front -d 6 2>/dev/null | grep -c '^width:')
  echo -n "$1: frames_in_6s=$N  "; timeout 4 ign topic -e -t /world/so101_world/stats -n 1 2>&1 | grep real_time_factor
  kill $GZ 2>/dev/null; sleep 2; sleep 1
}
run default /tmp/w.sdf
run no_shadow /tmp/w_noshadow.sdf
run small_320x240 /tmp/w_small.sdf
run no_robot /tmp/w_norobot.sdf
