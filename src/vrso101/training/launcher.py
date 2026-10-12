"""lerobot-train の Windows 用ランチャー。`python -m vrso101.training.launcher <lerobot-train の引数>`

なぜ必要か: lerobot は各チェックポイント保存後に `checkpoints/last` へシンボリックリンクを張るが、
Windows で開発者モード/管理者権限が無いと os.symlink が WinError 1314 で失敗し、学習が最後に異常終了する
(チェックポイント自体は保存済みなのに終了コードが非 0 になる)。権限不要のディレクトリジャンクションで
同じ役割を果たす関数に差し替える。lerobot (vendor でも site-packages でもある上流) 本体は改変しない。
"""
from __future__ import annotations

import os
import sys
from pathlib import Path


def _update_last_checkpoint(checkpoint_dir: Path) -> Path:
    link = checkpoint_dir.parent / "last"
    if link.is_symlink():
        link.unlink()
    elif link.exists():
        try:
            os.rmdir(link)          # ジャンクションは rmdir で消える (中身は消えない)
        except OSError:
            import shutil
            shutil.rmtree(link)
    if sys.platform == "win32":
        import _winapi
        _winapi.CreateJunction(str(checkpoint_dir.resolve()), str(link))
    else:
        link.symlink_to(checkpoint_dir.relative_to(checkpoint_dir.parent))
    return link


def main() -> None:
    import lerobot.common.train_utils as tu
    import lerobot.scripts.lerobot_train as lt
    tu.update_last_checkpoint = _update_last_checkpoint
    lt.update_last_checkpoint = _update_last_checkpoint
    lt.main()


if __name__ == "__main__":
    main()
