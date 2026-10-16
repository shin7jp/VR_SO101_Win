#!/usr/bin/env bash
# WSL 側 Gazebo シムブリッジを前面で起動する (Ctrl-C で Gazebo ごと停止)。
set +u; source /opt/ros/humble/setup.bash; set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
source "$(dirname "$0")/gl_env.sh"
export PYTHONUNBUFFERED=1 PYTHONUTF8=1
exec python3 "$ROOT/sim/bridge/server.py" "$@"
