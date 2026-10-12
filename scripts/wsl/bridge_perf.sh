#!/usr/bin/env bash
# 起動方法 (-r の有無・-v・一時停止からの再開) で RTF がどう変わるかを切り分ける。
set +u; source /opt/ros/humble/setup.bash; set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export IGN_GAZEBO_RESOURCE_PATH="$ROOT/sim/models"
W=/tmp/w.sdf
python3 - <<PY
import sys, yaml
sys.path.insert(0, "$ROOT/sim/bridge")
import worldgen
t = yaml.safe_load(open("$ROOT/config/tasks/pick_place.yaml", encoding="utf-8"))
open("/tmp/w.sdf", "w").write(worldgen.make_world_sdf(t))
PY
WORLD=so101_world
ctl() { ign service -s /world/$WORLD/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req "$1" > /dev/null; }
run() {  # $1=label $2=gz flags  $3=unpause(1/0)
  ign gazebo -s $2 --headless-rendering $W > /tmp/bp.log 2>&1 &
  local GZ=$!
  sleep 12
  [ "$3" = 1 ] && ctl 'pause: false'
  sleep 5
  echo -n "$1: "; timeout 4 ign topic -e -t /world/$WORLD/stats -n 1 2>&1 | grep -E "real_time_factor|paused" | tr '\n' ' '; echo
  kill $GZ 2>/dev/null; sleep 2; sleep 1
}
run "-r -v1" "-r -v 1" 0
run "-r -v3" "-r -v 3" 0
run "paused-start -v3 + unpause" "-v 3" 1
run "paused-start -v1 + unpause" "-v 1" 1
