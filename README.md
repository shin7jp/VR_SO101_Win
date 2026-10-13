# VR × SO-101 × ACT — 学習・Gazebo 推論 Web アプリ

SO-101 ロボットアームを **VR / PC / タッチ**で遠隔操作し、Gazebo のカメラ画像・関節状態・指令を同期収録 →
**LeRobot 互換データセット**へ変換 → **ACT (LeRobot 公式実装)** で学習 → 学習済みポリシーを **Gazebo 上で実行**し、
推論の様子を **MP4 + メタデータ**として保存・比較評価する、実際に動くアプリケーション。

- 3 タスク: **Pick and Place / Open Door / Rotating Lever Door** (物理シーン + 物理状態による成功判定)
- 11 画面: Dashboard / Simulation / VR Teleoperation / Data Collection / Dataset Explorer / ACT Training /
  Model Registry / Gazebo Inference / Inference Video Gallery / Evaluation / Settings
- ROS 2 Humble + Gazebo Fortress は WSL2、LeRobot/PyTorch/FastAPI/Web は Windows ネイティブ (依存を分離)

詳細: [SETUP.md](SETUP.md) (導入) / [ARCHITECTURE.md](ARCHITECTURE.md) (設計) / [TROUBLESHOOTING.md](TROUBLESHOOTING.md) (トラブル・実測した落とし穴)

## 起動

```powershell
.\scripts\start_all.ps1        # WSL の Gazebo ブリッジ + Web (https://localhost:8443)
.\scripts\stop_all.ps1
```

初回は `SETUP.md` の導入 (WSL に ROS 2 + Gazebo、Windows に venv) が必要。**Settings 画面の環境診断**が、足りないものを復旧手順つきで示す。
Gazebo 無しで UI だけ試す: `.\scripts\serve.ps1 -Backend mock` (疑似シム。UI に常時警告が出て、**検証結果にはならない**)。

## 使い方 (ワークフロー)

1. **Simulation / Data Collection**: タスクを選んで「読み込み」。WebXR 対応環境では VR Teleoperation から「VR 開始」。
   - VR: 右コントローラ。**手先がコントローラーの動きに常時追従** (没入した瞬間のポーズを基準にするので飛ばない)、**グリップ=一時停止** (握っている間は止まり、離した位置で基準を取り直す)、トリガー=グリッパー、
     A/X=向きの再キャリブレーション。座標変換 (単位・軸・yaw・スケール・原点オフセット) は設定可能。
   - PC: W/S 前後・A/D 左右・R/F 上下・矢印/Q/E で手首・Z/X/Space でグリッパー。「ドラッグ: EE」でマウスドラッグ操作。
   - iPhone/iPad: 画面の方向パッド (押している間だけ動く)・グリッパースライダー・ピンチで視点。
2. **収録**: 「● 収録開始」→ 操作 → 成功で自動停止 (または「■ 停止して保存」)。成否は**シミュレータの物理状態**で自動判定され、人手の評価ラベルと別に持つ。
   失敗エピソードは**自動削除しない** (Dataset Explorer で確認して「除外」「削除」)。
3. **Dataset Explorer**: 動画・関節軌跡 (観測 q と指令 action)・品質検査 (欠落フレーム・時刻逆行・限界超過) → 「LeRobot 形式へ変換」。
4. **ACT Training**: データセット・ステップ数・バッチ・デバイス・保存間隔を指定して開始。損失曲線とログがライブ更新。ブラウザを閉じても続行。
5. **Model Registry** → **Gazebo Inference**: チェックポイントと**評価用シード (10000 以上)** を選んで実行。ライブ映像を見ながら即時停止できる。
6. **Inference Video Gallery / Evaluation**: 動画再生・ダウンロード・メタデータ・2 本比較、チェックポイント間の成功率 (Wilson 区間つき)・完了時間・FPS 比較。

CLI も同じ機能を持つ (Web UI は薄い皮): `python -m vrso101.cli {collect-scripted, episodes, export, train, infer}`。

## 保存先

```
data/episodes/<task>/<episode_id>/         生エピソード (meta.json, states.npz, frames.mkv[可逆], preview.mp4)
data/datasets/<task>/<name>/               LeRobot v3 データセット + vrso101_meta.json
data/models/<task>/<model_id>/             vrso101_model.json + lerobot/checkpoints/<step>/pretrained_model/
data/inference/<task>/<episode_id>/        inference.mp4 + metadata.json
data/jobs/<job_id>/                        job.json + job.log
```

再起動後もこれらから一覧が復元される。

## API (主なもの)

REST (`/api/...`) と WebSocket (`/ws?since=<seq>`, すべてのイベントに `session_id` と `seq`; 切断→再接続でイベントを再送、欠落時は `resync_required`)。
任意のシェルコマンドを実行する API は無く、ジョブは固定 argv のみ。

| 機能 | エンドポイント |
|---|---|
| システム/診断/デバイス/タスク/ロボット | `GET /api/system`, `/api/system/diagnostics`, `/api/system/devices`, `/api/tasks`, `/api/robot` |
| シム | `POST /api/sim/connect`, `/api/sim/task`, `/api/sim/reset`; `GET /api/sim/status`, `/api/sim/camera.mjpg` |
| テレオペ | WS メッセージ `vr` / `nudge` / `release` / `speed`; `POST /api/teleop/calibrate` |
| エピソード | `POST /api/episodes/start|stop`; `GET/PATCH/DELETE /api/episodes/{task}/{id}` (+ `/video`) |
| データセット | `GET /api/datasets`; `POST /api/datasets/export`; `GET /api/datasets/{task}/{name}/validate` |
| 学習 | `POST /api/training/start`; `POST /api/jobs/{id}/stop`; `GET /api/jobs`, `/api/jobs/{id}` |
| モデル | `GET /api/models`, `GET/DELETE /api/models/{task}/{id}` |
| 推論 | `POST /api/inference/start|stop`; `GET /api/inference/videos` (+ `/{task}/{id}`, `/video?download=true`) |
| 評価/設定 | `GET /api/evaluation?task_id=`, `/api/settings/export` |

`VRSO101_TOKEN` を設定すると変更系 API と WebSocket にトークンが必要 (LAN 公開時)。長時間処理は 202 相当のジョブ ID で状態取得する。
再起動・連打に対して停止/リセット/収録開始は冪等 (同じ key の実行中ジョブは新規起動せず既存を返す)。

## テスト

```powershell
.\.venv\Scripts\python.exe -m pytest tests     # 単体 + モック結合。Gazebo/GPU 不要
$env:VRSO101_E2E=1; .\.venv\Scripts\python.exe -m pytest tests\test_e2e_gazebo.py -v -s   # 実 Gazebo + 実 ACT の E2E
```

**モックテストの合格は、実 Gazebo・ACT 連携の合格ではない** (UI にも常時警告)。実機相当の検証結果は下の表。

## 動作確認結果 (この開発環境で実際に確認できた範囲)

環境: Windows 11 + WSL2 (ROS 2 Humble / Gazebo Fortress 6.18, ogre ヘッドレス) / RTX 4070 Ti SUPER / LeRobot 0.6.1 / PyTorch 2.11+cu128。
**以下は実 Gazebo・実 ACT での実測値** (モックではない)。

| 項目 | 結果 |
|---|---|
| Gazebo ヘッドレスカメラ (Gate 0) | 640×480 を取得できた (描画は `ogre` のみ。`ogre2` は WSL2 で落ちる) |
| lock-step 同期 | 25 fps、1 周期 ~25〜40 ms (テレオペ中)。画像は物理 1 step 遅れ (一定)。カメラ/関節/物体姿勢のスタンプ整合を毎周期検査 |
| Pick and Place の物理把持 (スクリプトデモ) | **49/50 成功** (把持アシストなし。摩擦把持で持ち上げ ≥4cm → 目標ゾーンへ設置、配置誤差 ≲ 5mm)。1 本は失敗として残してある |
| Open Door (スクリプトデモ = 押して開ける) | **8/8 成功** (ヒンジ角 ≈0.63 rad、物理判定) |
| Rotating Lever Door (スクリプトデモ) | **8/8 成功** (レバー ≈0.86 rad で押し下げ → ラッチ解除 → ドア ≈0.63 rad) |
| LeRobot 変換・読込 | 49 エピソード / 11,270 フレーム → `LeRobotDataset` で読込・画像デコード検証 OK |
| ACT 学習 | (a) 49 本 / 15,000 step / batch 8 / CUDA、約 23 分、loss 0.079。(b) 145 本 (33,350 フレーム) / 40,000 step、最終 loss ≈0.05。チェックポイント 10k/20k/30k/40k 保存 |
| 学習経路 vs 推論経路の整合 | 同一フレームを両経路で ACT に通した出力差 ≤ 0.012 rad (H.264 劣化の範囲)。推論前処理に食い違いなし |
| **ACT の Gazebo 推論 (評価シード 10000–10009, 各 500 step 上限)** | 49 本・15k ckpt: **1/10** (Wilson 95% 区間 2–40%)、同 10k: 0/10。**145 本・40k ckpt: 5/30** (シード 10000–10029、17%、区間 7–34%。最初の 10 本は 4/10、追加 20 本は 1/20)、同 20k (10 本): **0/10** (うち 1 本は control_delay で異常停止)。成功時の完了時間 8–19 秒 (sim) |
| **Open Door の ACT (評価シード 10000–10029, 各 750 step 上限)** | スクリプトデモ 108 本 (20,520 フレーム)・30k step で学習 (最終 loss 0.028)。30k ckpt: **25/30 成功** (83%、Wilson 95% 区間 66–93%)。成功時の完了時間 5.6–13 秒 (sim)、失敗 5 本はすべてタイムアウト。10k/20k ckpt は未評価 |
| **Rotating Lever Door の ACT (評価シード 10000–10029, 各 1000 step 上限)** | スクリプトデモ 108 本 (33,480 フレーム)・30k step で学習 (最終 loss 0.033)。30k ckpt: **10/30 成功** (33%、Wilson 95% 区間 19–51%)。失敗 20 本はすべてタイムアウトで、**レバーは全試行で下げ切れてラッチは外れる (100%) が、ドアが ≈0.05 rad しか開かない** (= 失敗はレバーの後、ドアを押して開ける段階)。10k/20k ckpt は未評価 |
| (診断) 学習シード 3, 7, 20 での 15k ckpt | 2/3 成功 (評価には含めず削除済み) |
| 推論 FPS (制御ループ) | ~6.5 Hz (1 周期の中央値 ~105 ms、方策推論自体は ~7 ms) |
| 推論動画 | 500 フレームの H.264 MP4 + metadata.json を全エピソードで保存し、保存後にデコードして検証 (保存失敗時は「保存済み」と出さない) |
| 自動テスト | 95 件 + 1 skip (実 Gazebo E2E は `VRSO101_E2E=1` で実行) が通過。**実 Gazebo + 実 ACT の E2E (`tests/test_e2e_gazebo.py`: 収録 → 変換 → 検証 → 学習 → ロード → 推論 → MP4/JSON → 動画デコード検証) も通過 (105 秒)** |
| HTTPS / WSS | `https://localhost:8443` と `wss://localhost:8443/ws` (hello イベント) を確認 |
| UI | 11 画面を実装。ビルド・型検査 OK。実 Gazebo のカメラ映像・3D 表示 (ロボット URDF・立方体・ドア/レバー/受け金) を確認。375px 幅のレイアウトを確認 |

**ACT の成功率について**: 145 本・40k step の方策は、未知の評価シード 30 本で **5/30 (17%、区間 7–34%)** だった。
最初の 10 本では 4/10 だったが、追加の 20 本で 1/20 となり、最初の値は楽観的な偶然だった。49 本・15k step の 1/10 (n=10) と
比べて改善したとは言えない (同条件の n=30 評価はしていない)。20k ckpt は 0/10 で、チェックポイント間のばらつきも大きい。
推論パイプラインの不具合ではなく (上の整合検査)、スクリプトデモ 145 本 (単一の動作様式) では汎化が弱い、というのが現状の実測。
次の手: 人間のデモを含める、デモの多様性 (把持位置・経路のばらつき) を増やす、`n_action_steps` を調整する。
この値は「パイプライン全体が実 Gazebo で通り、成否・動画・指標が記録される」ことの証拠であって、性能の主張ではない。


## 既知の制約 (正直なまとめ)

**仕様からの意図的な逸脱・簡略化 (バグではなく設計判断。正直に明記する)**

- **Open Door のスクリプトデモは「パネルを押して開ける」**。ハンドル (頭付きペグ) を顎で挟んで蝶番の円弧に沿って引く方式は、
  DART で接触が破綻して不安定だった (TROUBLESHOOTING.md)。ヒンジは両方向に回り、成功判定は |角度| なので押しても引いても成功。
  ペグは人間が引くために付けてあるが、**引いて開けるところは最後まで検証できていない**。
- **Lever Door のレバーは摩擦保持 (ばね戻りなし)**: 押し下げると下がったまま留まる。ラッチは物理 (U 字の受け金にレバー先端が掛かる) で、
  レバーを回さずにドアは開かない (押しても引いても当たる) ことを成功判定の順序要求と合わせて確認している。
- **ACT は人間のデモではなく、スクリプト生成デモ 49 本で学習**した (`collection_mode=scripted` として区別して保存・変換できる)。
  人間のテレオペ収録の UI/API は動くが、人間のデータでの学習は未実施。
- **ドアタスクの ACT 学習・推論は未実施**。収録パイプラインのみ検証 (スクリプトデモ Open Door 8/8・Lever Door 8/8 成功、LeRobot 変換経路は Pick and Place と共通)。
- SO-101 のモデルは公式 URDF から生成 (Apache-2.0)。**衝突形状は凸包/凸分解の近似** (元メッシュは遅すぎる)、ドア部品の円柱衝突は直方体で代替。
- 制御は位置指令 + Gazebo の `JointPositionController` (PID)。実機の STS3215 サーボ特性や LeRobot の degrees / 0–100 正規化とは別物 (単位はラジアン)。
- fps は 25 (物理 dt=4ms×10)。30 fps は整数 ns にならずカメラ同期が不規則になるため不採用。

**この環境で検証できていないもの**

- **実 VR ヘッドセット上の WebXR** (コード・座標変換・追従と一時停止・単体テストはあるが、ヘッドセット無しで未検証)。HTTPS/WSS 経路自体は `curl -k` と wss 接続で確認済み。
- **実機の iPhone / iPad** (375px 幅の画面エミュレーションでレイアウトのみ確認。タッチ操作は未検証)。
- Docker (環境に無い)、Vite 開発サーバーの自己署名 SSL。
- 実機の SO-101。

**既知の挙動・限界**

- カメラ画像は物理 1 ステップ (4 ms) 遅れ。学習/推論とも同じ経路なので分布は一致するが、GPU/CPU が逼迫すると 2〜3 ステップに伸びる (許容窓 3.5 step、実スタンプは記録して品質検査)。
- 同一指令列を 2 回流したときの関節差は最大 ~0.003〜0.009 rad (ビット再現はしない)。
- 推論中の 1 周期は **中央値 ~105 ms** (スクリプトデモ収録時は ~30 ms)。ポリシーの出力が大きく動くと接触が増えるためと推測 (未解明)。推論 FPS は ~5。
- 推論中はシムを推論ワーカーが専有する (テレオペ・収録不可)。ブリッジは同時 1 クライアント。
- 外部送信は既定でなし (W&B・Hub 無効)。ただし ResNet18 の ImageNet 事前学習重みは初回に公開サーバーから取得する。

