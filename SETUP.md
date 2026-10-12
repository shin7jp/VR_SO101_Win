# SETUP.md — 導入手順とバージョン

## 検証した環境 (すべて実際にこの構成で動かした値)

| 項目 | バージョン / 値 |
|---|---|
| OS | Windows 11 Pro 10.0.26300 + WSL2 (Ubuntu 22.04.3 LTS, kernel 5.15.133.1, WSLg 1.0.59, systemd 有効) |
| GPU | NVIDIA GeForce RTX 4070 Ti SUPER 16GB (Windows ドライバ 610.88) |
| ROS 2 | **Humble** (`ros-humble-ros-base` / `ros-humble-ros-gz` 0.244.26) — WSL |
| Gazebo | **Fortress** (`ign gazebo` 6.18.0) — WSL。Humble の `ros_gz` 既定の組 |
| WSL の Python | 3.10.12 (system) + numpy 1.21 + PyYAML (ROS 同梱) — ブリッジはこれだけで動く |
| Windows の Python | **3.12.12** (`uv venv --python 3.12`) |
| PyTorch | 2.11.0+cu128 / torchvision 0.26.0 |
| LeRobot | **0.6.1** (`lerobot[training]==0.6.1`) — 公式 `lerobot-train` を固定バージョンで使用 |
| PyAV | 15.1.0 (動画の書き出し・デコード。torchcodec は Windows で読めないので `video_backend=pyav`) |
| Node.js | 22.19.0 / npm 10 |
| フロント | React 18.3.1, three 0.168.0, urdf-loader 0.12.3, zustand 4.5.5, Vite 5.4.8, TypeScript 5.6.2 (すべて固定) |
| ロボットモデル | TheRobotStudio/SO-ARM100 の SO-101 URDF (Apache-2.0)。`vendor/SO-ARM100` は上流のまま |

ROS 2 と Gazebo は WSL、LeRobot/PyTorch/FastAPI/Web は Windows ネイティブ。依存を混ぜない。
両者は TCP (WSL 側がサーバー、`localhost:9100`) だけで繋がる。

## 1. WSL2 に ROS 2 Humble + Gazebo Fortress を入れる

`sudo` が要るので **自分の端末で** 実行する (パスワードを他人/AI に渡さない)。冪等なので何度流しても壊れない。

```powershell
wsl -d Ubuntu -- bash /mnt/e/claude/VR_SO101_ACT_Win_Web/scripts/wsl/install_ros_gz.sh
```

導入後の確認 (Gate 0: ヘッドレスでカメラが描画できるか):

```powershell
wsl -d Ubuntu -- bash /mnt/e/claude/VR_SO101_ACT_Win_Web/scripts/wsl/gate0.sh ogre
```

`frames=…  -> … Hz` と `width: 640 height: 480` が出れば OK。**描画エンジンは必ず `ogre`**
(`ogre2` は WSL2 の D3D12 GL で落ちる。TROUBLESHOOTING.md)。

## 2. Windows 側 Python 環境

```powershell
cd E:\claude\VR_SO101_ACT_Win_Web
uv venv --python 3.12 .venv
uv pip install --python .venv\Scripts\python.exe "torch==2.11.0" "torchvision==0.26.0" --index-url https://download.pytorch.org/whl/cu128
uv pip install --python .venv\Scripts\python.exe -e ".[test]"
uv pip install --python .venv\Scripts\python.exe "lerobot[training]==0.6.1" trimesh scipy coacd
```

(`trimesh` / `scipy` / `coacd` は SO-101 モデルの衝突メッシュ生成 `sim/tools/build_model.py` 用。)

SO-101 の Gazebo モデルを生成 (vendor の URDF → SDF + 凸包衝突メッシュ。生成物はコミット不要):

```powershell
git clone https://github.com/TheRobotStudio/SO-ARM100.git vendor/SO-ARM100     # 未取得なら
.\.venv\Scripts\python.exe sim\tools\build_model.py
```

## 3. フロントエンド

```powershell
cd frontend
npm install
npm run build        # frontend/dist (FastAPI が配信)
```

## 4. 起動

```powershell
.\scripts\start_all.ps1            # WSL ブリッジ起動 + Web アプリ (HTTPS :8443)
```

- ブラウザ: `https://localhost:8443`。他端末 (Quest / iPad / iPhone) は `scripts\make_cert.py` が表示する LAN URL。
  **自己署名証明書**なので初回に警告が出る (詳細 → 続行)。Windows ファイアウォールで TCP 8443 の受信を許可すること。
- WebXR は HTTPS 必須。HTTPS ページから `ws://` は開けないので、API・UI・WebSocket を同一ポートで配信している。
- 停止: `.\scripts\stop_all.ps1` (GPU を掴む孫プロセスごと `taskkill /T /F`)。
- Gazebo 無しで UI だけ試す: `.\scripts\serve.ps1 -Backend mock` (疑似シム。**検証結果にはならない**。UI に警告が出る)。
- 開発 (ホットリロード): ターミナル 1 `.\scripts\serve.ps1 -NoTls -Port 8000`、ターミナル 2 `cd frontend; npm run dev`
  (`https://localhost:5173`。`/api` と `/ws` は Vite が 8000 へ中継)。

### ブリッジの単体操作

```powershell
.\scripts\bridge.ps1 start | stop | restart | status | log
```

### 環境変数 (`.env.example` 参照)

| 変数 | 既定 | 意味 |
|---|---|---|
| `VRSO101_BACKEND` | `gazebo` | `gazebo` / `mock` |
| `VRSO101_DATA_DIR` | `<repo>/data` | データ保存先 (datasets / models / inference / episodes) |
| `VRSO101_BRIDGE_PORT` | `9100` | WSL ブリッジのポート |
| `VRSO101_TOKEN` | (なし) | 設定すると変更系 API と WebSocket にトークンが必要 (LAN 公開時) |
| `VRSO101_STEP_TIMEOUT` | `30` | ブリッジ内の 1 ステップ観測待ちの上限 [秒] |

## 5. テスト

```powershell
.\.venv\Scripts\python.exe -m pytest tests                      # 単体 + モック結合 (Gazebo/GPU 不要)
# 実 Gazebo + 実 ACT の E2E (ブリッジ起動済み・CUDA 必須。別クライアントが使っていないこと)
$env:VRSO101_E2E = 1; .\.venv\Scripts\python.exe -m pytest tests\test_e2e_gazebo.py -v -s
cd frontend; npx tsc --noEmit                                    # フロントの型検査
```

モックテストの合格は Gazebo 連携の合格ではない (`tests/test_e2e_gazebo.py` が実機相当の検証)。

## 6. Docker (適用できる部分のみ・**未検証**)

`docker/` に Gazebo ブリッジ (Ubuntu 22.04 + Humble + Fortress) と Web バックエンドの Dockerfile / compose がある。
この開発環境には Docker が無く、ビルド・実行は確認していない。WSLg / GPU を渡す設定は環境依存。
