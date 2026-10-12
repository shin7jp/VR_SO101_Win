#!/usr/bin/env bash
# 既に動いている Gazebo(ブリッジ経由で起動済み) を非停止にして RTF を測る。引数: ラベル
set +u; source /opt/ros/humble/setup.bash; set -u
ign service -s /world/so101_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: false' > /dev/null
sleep 4
echo -n "$1: "; timeout 4 ign topic -e -t /world/so101_world/stats -n 1 2>&1 | grep real_time_factor
ign service -s /world/so101_world/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: true' > /dev/null
echo "subscribers of camera:"; ign topic -i -t /camera/front 2>&1 | head -8
