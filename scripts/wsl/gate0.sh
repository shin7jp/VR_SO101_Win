#!/usr/bin/env bash
# Gate 0: WSL2 でヘッドレスの Gazebo カメラが描画できるか / ステップ実行が効くかを実測する。
# 使い方: bash scripts/wsl/gate0.sh [ogre2|ogre]   (描画エンジンを切り替えて試す)
set +u; source /opt/ros/humble/setup.bash; set -u
ENGINE="${1:-ogre}"
source "$(dirname "$0")/gl_env.sh"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WORLD="$(mktemp --suffix=.sdf)"
sed "s|<render_engine>ogre2</render_engine>|<render_engine>${ENGINE}</render_engine>|" "$ROOT/sim/worlds/gate0.sdf" > "$WORLD"
LOG=/tmp/gate0_gz.log
echo "== engine=$ENGINE world=$WORLD"
ign gazebo -s -r --headless-rendering -v 3 "$WORLD" > "$LOG" 2>&1 &
GZ=$!
trap 'kill $GZ 2>/dev/null; sleep 1 2>/dev/null; true' EXIT
# 固定 sleep だと遅い環境 (初回の D3D12 初期化など) で 0 フレームになるので、最初のフレームが来るまで待つ (最大 90 秒)
S=$(date +%s)
until timeout 5 ign topic -e -t /gate0/camera -n 1 2>&1 | grep -q '^width:'; do
  if [ $(( $(date +%s) - S )) -gt 90 ]; then echo "[NG] 90 秒待ってもカメラのフレームが来ません"; break; fi
done
echo "first frame after $(( $(date +%s) - S ))s"
echo "== topics"; ign topic -l | head -20
echo "== camera frames in 5s (走行中の実測。Fortress の ign topic に -hz は無いので数える)"
N=$(timeout 8 ign topic -e -t /gate0/camera -d 5 2>/dev/null | grep -c '^width:' || true)
echo "frames=$N  -> $(python3 -c "print(round($N/5,1))") Hz"
echo "== 1 frame"
timeout 5 ign topic -e -t /gate0/camera -n 1 2>&1 | grep -E '^(width|height|pixel_format_type)' || true
echo "== pixel content"
python3 "$(dirname "$0")/gz_frame_stats.py" /gate0/camera || true
echo "== sim clock / stats"
timeout 4 ign topic -e -t /stats -n 1 2>&1 | head -12 || true
echo "== gz log tail"; tail -15 "$LOG"
