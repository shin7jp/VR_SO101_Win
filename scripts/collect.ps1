# スクリプトデモ収録をバックグラウンドで回すヘルパー (子プロセスの文字コード設定込み)。
#   .\scripts\collect.ps1 -Task open_door -N 8 -SeedStart 0 [-Port 9101] [-Log data\logs\x.log]
param([string]$Task = "pick_place", [int]$N = 5, [int]$SeedStart = 0, [int]$Port = 9100, [string]$Log = "")
$root = Split-Path -Parent $PSScriptRoot
$env:PYTHONUTF8 = 1; $env:PYTHONIOENCODING = 'utf-8'; $env:PYTHONUNBUFFERED = 1
$env:VRSO101_BRIDGE_PORT = $Port
if (-not $Log) { $Log = Join-Path $root "data\logs\collect_$Task.log" }
& "$root\.venv\Scripts\python.exe" -m vrso101.cli collect-scripted --task $Task --n $N --seed-start $SeedStart *> $Log
