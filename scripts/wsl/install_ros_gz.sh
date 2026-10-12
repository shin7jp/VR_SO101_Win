#!/usr/bin/env bash
# WSL2 Ubuntu 22.04 に ROS 2 Humble + Gazebo Fortress (ros_gz) を入れる。
#
# なぜこの組み合わせか: Ubuntu 22.04 の公式 ROS 2 は Humble で、`ros-humble-ros-gz` が
# 既定で組むのは Gazebo Fortress (ign-gazebo6)。Harmonic は Humble では非既定で追加
# リポジトリが要るため、まず既定ペアで通す。冪等: 何度流しても壊れない。
# sudo パスワードは端末で本人が入力する (チャットや sudoers 変更に頼らない)。
set -euo pipefail

if [[ "$(lsb_release -rs)" != "22.04" ]]; then
  echo "[NG] Ubuntu 22.04 専用です (検出: $(lsb_release -rs))" >&2; exit 1
fi

echo "[1/4] ネットワーク到達性チェック"
for u in https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         http://packages.ros.org/ros2/ubuntu/dists/jammy/InRelease; do
  curl -fsS -o /dev/null -m 20 "$u" || { echo "[NG] 到達不可: $u" >&2; exit 1; }
done

echo "[2/4] ROS 2 apt リポジトリ登録"
sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe
sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "[3/4] パッケージ導入 (パスワード入力は 1 回で済むよう一括)"
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros-base ros-humble-ros-gz ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-cv-bridge ros-humble-rosbridge-server \
  python3-colcon-common-extensions python3-rosdep python3-venv python3-pip \
  ffmpeg mesa-utils libgl1-mesa-dri xvfb

echo "[4/4] 確認"
set +u; source /opt/ros/humble/setup.bash; set -u
ros2 --version 2>/dev/null || true
command -v ign && ign gazebo --versions || true
echo "[OK] 完了。次は scripts/wsl/gate0.sh でヘッドレスカメラ描画を検証してください。"
