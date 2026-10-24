# WSL の Gazebo ブリッジ + Web アプリ (HTTPS) をまとめて起動する。停止は scripts\stop_all.ps1。
param([string]$Backend = "gazebo", [int]$Port = 8443, [switch]$NoTls)
$root = Split-Path -Parent $PSScriptRoot
if ($Backend -eq "gazebo") { & "$root\scripts\bridge.ps1" start }
# 配列で渡すと "-Backend" が引数名ではなく位置引数の文字列になり、"gazebo" が -Port (整数) に入ってしまう。
# 名前付きで渡すにはハッシュテーブルでスプラットする。
$serveArgs = @{ Backend = $Backend; Port = $Port }
if ($NoTls) { $serveArgs.NoTls = $true }
& "$root\scripts\serve.ps1" @serveArgs
