#!/usr/bin/env bash
# 開発用の 2 本目のブリッジ: ign / ROS の通信を別パーティション・別ドメインに隔離し、別ポートで起動する。
# 本番ブリッジ (9100) が収録などでシムを使用中でも、別のシーンを並行して試せる。
export IGN_PARTITION=dev1 ROS_DOMAIN_ID=77 VRSO101_SIM_LOG_DIR=/tmp/vrso101_dev
exec bash "$(dirname "$0")/start_bridge.sh" --port "${1:-9101}"
