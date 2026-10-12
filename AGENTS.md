# AGENTS.md — VR_SO101_ACT_Win_Web

SO-101 を VR / PC / タッチで遠隔操作 → 収録 → LeRobot データセット → ACT 学習 → Gazebo 推論 →
動画保存までを Web から回すプラットフォーム。共通ルールは `E:\Codex\AGENTS.md`。

## 前提 (実際に確認した環境)
- Windows 11 + WSL2 Ubuntu 22.04。**ROS 2 Humble + Gazebo Fortress は WSL 側**、
  FastAPI・Web・LeRobot/PyTorch は **Windows ネイティブ** (`.venv`, Python 3.12, LeRobot 0.6.1)。
- Gazebo/ROS 2 は WSL の system python、LeRobot は Windows venv。依存を混ぜない。
  両者は `src/vrso101/sim/protocol.py` の WebSocket (WSL 側がサーバー) でのみ繋ぐ。
  WSL→Windows の localhost は通らないので、サーバーは常に WSL 側に置く。
- sudo が要る作業 (apt) は私は実行できない。`scripts/wsl/*.sh` を用意してユーザーが端末で実行する。

## 設計の芯
- 「正本は Gazebo」。観測画像は Gazebo カメラのみ (学習も推論も同じ経路)。Three.js は操作・可視化用。
- シムは lock-step (停止状態から N ステップ進める) で駆動し、観測と指令の時刻ずれを作らない。
- 関節名・順序・単位・上下限は `config/robot/so101.yaml` が唯一の定義。URDF と一致をテストで検証。
- 単位は **ラジアン** (グリッパー含む)。LeRobot の degrees/range_m100_100 とは別物。
- モック (`mock` バックエンド) の合格は Gazebo 連携の合格ではない。検証範囲は README に正直に書く。
- `vendor/SO-ARM100` は上流のまま (Apache-2.0)。改変物は `sim/` に置く。

## コマンド
```powershell
.\.venv\Scripts\python.exe -m pytest tests            # 単体 (Gazebo 不要)
.\.venv\Scripts\python.exe -m vrso101.cli --help      # CLI (UI は薄い皮)
```
