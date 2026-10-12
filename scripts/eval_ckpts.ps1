# 複数チェックポイントを同じ評価シードで Gazebo 実行する (CLI 経由。結果は data/inference と Evaluation 画面に出る)。
param([string]$Model, [string[]]$Checkpoints = @("last"), [int]$SeedStart = 10000, [int]$N = 10, [int]$Nas = 10, [string]$Task = "pick_place")
$root = Split-Path -Parent $PSScriptRoot
$env:PYTHONUTF8 = 1; $env:PYTHONIOENCODING = 'utf-8'; $env:PYTHONUNBUFFERED = 1; $env:HF_HUB_OFFLINE = 1
$seeds = $SeedStart..($SeedStart + $N - 1)
foreach ($c in $Checkpoints) {
  & "$root\.venv\Scripts\python.exe" -m vrso101.cli infer --task $Task --model $Model --checkpoint $c --seeds $seeds --n-action-steps $Nas
}
