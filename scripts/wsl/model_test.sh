#!/usr/bin/env bash
# SO-101 モデルが Gazebo に載り、関節指令が効き、関節状態が出るかの確認 (ROS なし・ign CLI のみ)
set +u; source /opt/ros/humble/setup.bash; set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export IGN_GAZEBO_RESOURCE_PATH="$ROOT/sim/models"
export PYTHONPATH="$ROOT/src:$ROOT"
W=/tmp/so101_test.sdf
python3 - <<PY
import sys, yaml
sys.path.insert(0, "$ROOT/sim/bridge")
import worldgen
t = yaml.safe_load(open("$ROOT/config/tasks/pick_place.yaml", encoding="utf-8"))
open("$W", "w").write(worldgen.make_world_sdf(t))
PY
ign gazebo -s -r --headless-rendering -v 3 "$W" > /tmp/model_gz.log 2>&1 &
GZ=$!
trap 'kill $GZ 2>/dev/null; sleep 1 2>/dev/null; true' EXIT
sleep 15
echo "== topics"; ign topic -l | grep -E "so101|camera|joint" | head -30
pos() { timeout 5 ign topic -e -t /world/so101_world/model/so101/joint_state -n 1 2>&1 | awk '/^joint \{/{inj=1} inj&&/^  name:/{n=$2} inj&&/^    position:/{printf "%s=%.3f ", n, $2} END{print ""}'; }
echo "== 初期姿勢 (home=[0,-1,1,0.6,0,0] を期待)"; pos
echo "== cmd: shoulder_pan=0.5, gripper=1.0, elbow_flex=0.3 を指令して 3s 後"
ign topic -t /so101/cmd/shoulder_pan -m ignition.msgs.Double -p 'data: 0.5'
ign topic -t /so101/cmd/gripper -m ignition.msgs.Double -p 'data: 1.0'
ign topic -t /so101/cmd/elbow_flex -m ignition.msgs.Double -p 'data: 0.3'
sleep 3; pos
echo "== 実時間係数"; timeout 3 ign topic -e -t /stats -n 1 2>&1 | grep real_time_factor
echo "== log errors"; grep -iE "error|warn|fail" /tmp/model_gz.log | head -20
