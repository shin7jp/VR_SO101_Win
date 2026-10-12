# ARCHITECTURE.md

## 全体像

```
┌──────────────────────────────────────────────┐        Windows (ネイティブ, Python 3.12 .venv)
│  React + TypeScript + Three.js (Vite, HTTPS)  │
│  11 画面 / WebXR / PC・タッチ操作              │
└───────────────┬──────────────────────────────┘
                │ REST + WebSocket (同一オリジン: seq + session_id, 再接続で再同期)
┌───────────────▼──────────────────────────────┐
│ FastAPI (src/vrso101/api)                     │
│  SimSession ── 制御ループ 1 本 (25 Hz, lock-step)│
│  JobManager ── 学習/変換/推論ワーカー (子プロセス)│
│  EpisodeStore / モデルレジストリ / 評価集計     │
└───────┬──────────────────────┬───────────────┘
        │ TCP (protocol.py)    │ 固定 argv の子プロセス (ブラウザからコマンドは渡せない)
        │ localhost:9100       ├─ lerobot-train (vrso101.training.launcher 経由)
        │                      ├─ vrso101.cli export (LeRobot 変換)
        │                      └─ vrso101.inference.worker (ACT + PyTorch, シム専有)
┌───────▼──────────────────────┐   ←  WSL2 Ubuntu 22.04 (system python 3.10 + ROS 2 Humble)
│ sim/bridge/server.py          │
│  rclpy + ros_gz_bridge        │
│  Gazebo Sim Fortress          │  カメラ/関節状態/物体姿勢/ControlWorld/Spawn
│  (ogre ヘッドレス描画)        │
└──────────────────────────────┘
```

**Gazebo が物理の正本**。Three.js 側は表示専用で、WebSocket で届く関節計測値 (`q`) を描くだけ (独自に状態を進めない)。
学習用・推論用の画像はどちらも同じ Gazebo カメラ → ブリッジ → `Observation.image` の経路で、前処理は LeRobot の processor に委ねる。

## プロセス分離の理由

| プロセス | 環境 | 理由 |
|---|---|---|
| Gazebo ブリッジ | WSL system python (ROS) | ROS 2/Gazebo の依存と LeRobot/PyTorch を混ぜない。WSL→Windows の localhost は通らないので **WSL 側がサーバー** |
| FastAPI + UI | Windows venv | ユーザーが使うブラウザ/LAN からの到達性。HTTPS 配信 |
| 学習・推論 | Windows の子プロセス | GPU を掴む処理を API から切り離し、`taskkill /T /F` で確実に解放。ブラウザを閉じても止まらない |

`protocol.py`・`tasks/*`・`robot/joints.py` は **標準ライブラリ + numpy + yaml のみ** で書き、両側から import される
(WSL の Python 3.10 / numpy 1.21 で動くこと)。

## lock-step 同期 (観測と指令の時刻ずれを作らない)

Gazebo は**停止状態で起動**し、制御 1 周期 (10 物理ステップ = 40 ms @ 25 fps) ごとに
`ControlWorld(run_to_sim_time)` で目標時刻まで進めて自動停止させる。画像・関節状態・物体姿勢は
「そのステップ終了時刻のスタンプ」を持つものだけを採用する。リアルタイム走行だとネットワーク遅延で観測が前後に混ざる。
- 物理 dt = 1/(fps×10)。fps と dt が整数 ns になる値だけ許可 (25/20/40/50…)。
- カメラ画像は物理 1 ステップ遅れ (一定)。許容窓に入れ、`image_stamp` を記録して品質検査する。
- 実測: ~25〜40 ms/周期 (RTF ≈ 1.0〜1.7)。リアルタイムのテレオペ (25 Hz) に十分で、推論は一定以上の速度で回る。

### off-by-one を 1 か所に閉じ込める (`recording/recorder.py`)

`step(q)` は「q を適用した後」の観測を返す。学習フレーム k = **(指令 k より前の観測, 指令 k で適用された q_cmd)**。
同じレスポンスの観測と指令を組にすると action が 1 周期ずれる。`tests/test_recorder.py` が検出する。

## テレオペ (`teleop/`)

```
VR コントローラ (WebXR y上/-z前) ─┐
PC キーボード/マウスドラッグ     ─┼─▶ TeleopController ─▶ IK(DLS, URDF 直読み) ─▶ 速度/関節制限 ─▶ 平滑化 ─▶ 関節指令
iPhone/iPad 方向パッド           ─┘     (クラッチ方式)       ワークスペース制限
```

- 座標変換 `transform.py`: WebXR(右手系・y 上・-z 前) → ロボット(x 前・y 左・z 上)。yaw 補正・スケール・原点オフセットを設定可能。
- **クラッチ方式**: グリップ (デッドマン) を握った瞬間のコントローラ/EE 姿勢を基準にし、以降は相対移動だけを反映。
  握った瞬間に飛ばない。離せば即座に追従解除 (現在姿勢を保持)。
- IK は 5 自由度なので位置優先 (姿勢は重みを下げる)。解なしでも現在より近づく最良近似を採用し、NaN は出さない。
- 入力が 0.4 秒途絶える / 全 WebSocket 切断 → 追従停止 (通信切断時の停止)。
- VR / PC / タッチは同じ `SimSession` の「最新入力」に入り、毎周期 1 回だけ IK→制限→平滑化→step を通る。安全処理を共有する。

## データモデル (すべてローカルファイル。`VRSO101_DATA_DIR`)

```
data/
├── episodes/<task>/<episode_id>/      生データ (正本)
│     meta.json  states.npz  frames.mkv(FFV1 可逆)  preview.mp4(H.264)
├── datasets/<task>/<name>/            LeRobot v3 データセット + vrso101_meta.json
│     meta/ data/ videos/ ...          (lerobot 0.6.1 の LeRobotDataset.create で生成。独自形式なし)
├── models/<task>/<model_id>/          vrso101_model.json + lerobot/checkpoints/<step>/pretrained_model/ (+ last ジャンクション)
├── inference/<task>/<episode_id>/     inference.mp4 + metadata.json
├── jobs/<job_id>/                     job.json + job.log (再起動後も復元)
└── certs/                             自己署名証明書
```

- 生データは**可逆** (FFV1) で保存し、LeRobot 変換時の H.264 でだけ劣化させる (二重劣化を避ける)。
- 特徴量: `observation.images.front` (video, 480×640×3) / `observation.state` (float32×6) / `action` (float32×6)。
  **単位はラジアン** (グリッパー含む)。LeRobot 標準 SO101 の degrees / 0–100 とは別物なので `vrso101_meta.json` に単位・関節順・
  上下限・カメラ設定・件数・成功/失敗内訳・生成バージョンを残す。
- ストレージは `EpisodeStore` / モデルレジストリ関数経由 (将来 DB/クラウドに差し替え可能な境界)。

## ジョブ (`jobs/manager.py`)

- 固定 argv だけを起動 (ブラウザから任意シェルは不可)。`key` が同じ実行中ジョブがあれば新規起動せず既存を返す (連打・二重起動の防止)。
- 状態は `job.json` に永続化。バックエンド再起動後、生きているプロセスはログ追跡を再開し、居なければ「中断」にする。
- 進捗は子プロセスのログをパース (学習: tqdm 行から正確なステップ、`lerobot` の loss 行から損失曲線)。WebSocket に `job` イベントを流す。
- 学習は `lerobot-train` (公式) をそのまま使う。外部送信なし: `push_to_hub=false`, W&B 無効, `HF_HUB_OFFLINE=1`。

## 推論 (`inference/`)

学習済みチェックポイントを `ACTPolicy.from_pretrained` + `make_pre_post_processors(pretrained_path=…)` で読み込む
(**正規化を自前で再実装しない**)。1 周期: 観測 → 前処理 → (アクションキューが空なら) チャンク推論 → 後処理 → 速度/関節制限 → step。
`n_action_steps` ごとに再推論するので、チャンクを丸ごと盲目的に実行しない。
読み込み失敗 (破損・形状不一致・CUDA 無し) や観測が取れないときは**動作を開始せず**、原因を返す。
NaN 指令・1 ステップが 5 秒超 (制御遅延)・通信切断は安全停止。動画は推論中の Gazebo カメラフレームを H.264 で記録し、
**保存後に実際にデコードして検証**できて初めて `saved=true` にする。

## タスクとシーン

| タスク | シーン (Gazebo, 実行時に spawn) | 成功判定 (物理状態のみ) |
|---|---|---|
| Pick and Place | 立方体 (動的, 摩擦 1.5) + 目標ゾーン (見た目のみ) | 一度 4 cm 以上持ち上げ → XY が目標内 & 机に静止 を 15 ステップ連続 |
| Open Door | ヒンジ付きドア板 (回転 ±1.4 rad, ダンピング) + 頭付きペグ | |ヒンジ角| ≥ 0.6 rad を 10 ステップ連続 (押しても引いても可) |
| Rotating Lever Door | 同ドア + 回転レバー (摩擦保持) + U 字受け金 (ラッチ) | レバー ≥ 0.5 rad を経てから |ヒンジ角| ≥ 0.5 rad (順序要求) |

ラッチは物理で実装: レバー先端が受け金の U 字スロットに掛かっていて、レバーを回すと先端がスロットの下へ抜ける。
レバーを回さずにドアだけ開くことはできない (押しても引いても当たる)。初期配置はシードで再現可能で、
到達可能域に収まる範囲に限定 (`tests/test_tasks.py`, `tests/test_scripted_door.py` が全シードの到達性を検証)。

## 評価の分離

収録データ品質 (欠落フレーム・時刻逆行・指令の関節限界超過・画像/状態の時刻差) / ACT 学習結果 (loss・チェックポイント) /
Gazebo 推論結果 (成功率+Wilson 区間・タイムアウト率・異常停止率・完了時間・FPS・把持/配置精度/ドア角/レバー) を別々に集計。
学習に使ったシードは 0–9999、評価用は 10000 以上と分け、重複して評価したら警告する。
