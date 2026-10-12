# Web アプリ (uvicorn) とブリッジ・実行中の学習/推論ワーカーを止める。GPU を掴む孫プロセスごと taskkill /T /F。
$root = Split-Path -Parent $PSScriptRoot
Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -match 'vrso101\.(api\.app|training\.launcher|inference\.worker)' -and $_.CommandLine -match [regex]::Escape($root) -or
    $_.CommandLine -match 'uvicorn vrso101\.api\.app' } | ForEach-Object {
  Write-Host "停止: PID $($_.ProcessId)"
  taskkill /PID $_.ProcessId /T /F | Out-Null
}
& "$root\scripts\bridge.ps1" stop
