# WSL の Gazebo シムブリッジを Windows 側からバックグラウンド起動/停止/状態確認する。
#   .\scripts\bridge.ps1 start | stop | restart | status | log
param([Parameter(Position=0)][ValidateSet('start','stop','restart','status','log')][string]$Cmd = 'status')
$root = Split-Path -Parent $PSScriptRoot
$logDir = Join-Path $root 'data\logs'
New-Item -ItemType Directory -Force $logDir | Out-Null
$log = Join-Path $logDir 'bridge.log'
$wslRoot = '/mnt/' + $root.Substring(0,1).ToLower() + $root.Substring(2).Replace([string][char]92,'/')

function Test-Bridge { $r = wsl -d Ubuntu -- bash -c "pgrep -f '[s]im/bridge/server.py' | head -1"; return [bool]$r }

switch ($Cmd) {
  'start' {
    if (Test-Bridge) { Write-Host 'bridge は既に起動中です'; break }
    # 子プロセス出力の文字化けを避ける (CLAUDE.md 共通ルール)
    Start-Process -WindowStyle Hidden -FilePath wsl -ArgumentList "-d Ubuntu -- bash -c `"PYTHONUTF8=1 bash $wslRoot/scripts/wsl/start_bridge.sh > $wslRoot/data/logs/bridge.log 2>&1`""
    for ($i=0; $i -lt 30; $i++) { Start-Sleep 1; if ((Get-Content $log -ErrorAction SilentlyContinue) -match 'listening') { Write-Host 'bridge 起動 OK'; break } }
  }
  'stop' {
    wsl -d Ubuntu -- bash -c "pkill -TERM -f '[s]im/bridge/server.py'; sleep 3; pkill -KILL -f '[i]gn-gazebo|[i]gn gazebo|[p]arameter_bridge' ; true"
    Write-Host 'bridge 停止'
  }
  'restart' { & $PSCommandPath stop; & $PSCommandPath start }
  'status' { if (Test-Bridge) { Write-Host 'running' } else { Write-Host 'stopped' } }
  'log' { Get-Content $log -Tail 40 }
}
