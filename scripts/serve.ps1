# Web アプリを HTTPS で起動する (WebXR 用)。API と UI を同一オリジン・同一ポートで配信するので、
# https ページから ws:// を開けない (mixed content) 問題が起きない。
#   .\scripts\serve.ps1                 # 既定: gazebo バックエンド, https://0.0.0.0:8443
#   .\scripts\serve.ps1 -Backend mock   # Gazebo 無し (疑似シム。検証結果にはならない)
#   .\scripts\serve.ps1 -NoTls          # HTTP (localhost だけなら WebXR 以外は十分)
param([string]$Backend = "gazebo", [int]$Port = 8443, [switch]$NoTls, [switch]$Build, [string]$BindHost = "0.0.0.0")
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
$py = "$root\.venv\Scripts\python.exe"
if (-not (Test-Path $py)) { Write-Error ".venv がありません。SETUP.md の手順で作成してください"; exit 1 }
if ($Build -or -not (Test-Path "$root\frontend\dist\index.html")) {
  Write-Host "フロントエンドをビルドします..."
  Push-Location "$root\frontend"; npm install --no-audit --no-fund; npm run build; Pop-Location
}
# 子プロセスの文字コード (E:\claude 共通ルール)
$env:PYTHONUTF8 = 1; $env:PYTHONIOENCODING = 'utf-8'; $env:PYTHONUNBUFFERED = 1
$env:VRSO101_BACKEND = $Backend
$ssl = @()
if (-not $NoTls) {
  & $py scripts\make_cert.py
  $ssl = @("--ssl-keyfile", "$root\data\certs\key.pem", "--ssl-certfile", "$root\data\certs\cert.pem")
}
$scheme = if ($NoTls) { "http" } else { "https" }
Write-Host "起動: ${scheme}://localhost:$Port  (backend=$Backend)"
Write-Host "他端末 (iPad / Quest) からは上の LAN URL。初回は自己署名証明書の警告を許可してください。Windows ファイアウォールで TCP $Port の受信許可が必要です。"
& $py -m uvicorn vrso101.api.app:get_app --factory --host $BindHost --port $Port @ssl
