"""プロジェクト内パスの一元管理。データ保存先は環境変数で差し替え可能 (ストレージ差し替えの入口)。"""
from __future__ import annotations

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = ROOT / "config"
VENDOR_DIR = ROOT / "vendor"


def data_dir() -> Path:
    # テストは VRSO101_DATA_DIR を tmp に向けて実データを汚さない
    return Path(os.environ.get("VRSO101_DATA_DIR", ROOT / "data")).resolve()


def resolve_project_path(p: str | Path) -> Path:
    p = Path(p)
    return p if p.is_absolute() else ROOT / p
