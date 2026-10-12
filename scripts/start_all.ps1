# WSL の Gazebo ブリッジ + Web アプリ (HTTPS) をまとめて起動する。停止は scripts\stop_all.ps1。
param([string]$Backend = "gazebo", [int]$Port = 8443, [switch]$NoTls)
$root = Split-Path -Parent $PSScriptRoot
if ($Backend -eq "gazebo") { & "$root\scripts\bridge.ps1" start }
$args2 = @("-Backend", $Backend, "-Port", $Port)
if ($NoTls) { $args2 += "-NoTls" }
& "$root\scripts\serve.ps1" @args2
