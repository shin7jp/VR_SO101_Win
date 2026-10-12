#!/usr/bin/env bash
# 画像を ROS(DDS) で受けると Gazebo の RTF が落ちるか、DDS 設定でどう変わるかを実測する。
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
cat > /tmp/sub.py <<'PY'
import rclpy, time, sys
from sensor_msgs.msg import Image
rclpy.init(); n = rclpy.create_node("sub"); c = [0]
n.create_subscription(Image, "/camera/front", lambda m: c.__setitem__(0, c[0] + 1), 10)
t0 = time.time()
while time.time() - t0 < float(sys.argv[1]): rclpy.spin_once(n, timeout_sec=0.1)
print("ros_frames", c[0], "in", sys.argv[1], "s")
PY
WORLD=so101_world
CAM="/camera/front@sensor_msgs/msg/Image[ignition.msgs.Image"
run() {  # $1=label, 以降=環境変数
  local label="$1"; shift
  ign gazebo -s -r --headless-rendering -v 1 $W > /tmp/dp.log 2>&1 &
  local GZ=$!
  env "$@" ros2 run ros_gz_bridge parameter_bridge "$CAM" > /tmp/dp_br.log 2>&1 &
  local BR=$!
  sleep 12
  env "$@" python3 /tmp/sub.py 8 &
  local SUB=$!
  sleep 5
  echo -n "$label: "; timeout 4 ign topic -e -t /world/$WORLD/stats -n 1 2>&1 | grep real_time_factor | tr '\n' ' '
  wait $SUB
  kill $BR 2>/dev/null; kill $GZ 2>/dev/null; sleep 2; sleep 1
}
run default X=1
run localhost_only ROS_LOCALHOST_ONLY=1
