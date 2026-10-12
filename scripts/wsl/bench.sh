#!/usr/bin/env bash
# lock-step 周期の所要時間をモード別に計る。環境変数を素通しする。
set +u; source /opt/ros/humble/setup.bash; set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
export PYTHONUNBUFFERED=1 PYTHONUTF8=1
exec python3 "$ROOT/sim/bridge/bench_step.py"
