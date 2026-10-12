r"""Web 3D 表示用にロボットの STL を軽量化する (頂点・三角形を減らす。外郭は変えない)。

元の STL (vendor/SO-ARM100, 合計 ~30 万三角形) はブラウザの描画 (特に iPad / iPhone / Quest) には細かすぎる。
二次誤差メトリクスの間引き (fast-simplification) で三角形を減らしつつ、元の表面との最大距離 (ハウスドルフ距離) が
許容値 TOL_M 以下に収まる範囲で、メッシュごとに最大限まで減らす。外郭が変わるほど減らさないことを数値で保証する。

  .venv\Scripts\python.exe sim\tools\decimate_visuals.py

出力: sim/models/so101_web/assets/*.stl (生成物。vendor は触らない)。API はここにあればこちらを配信する。
Gazebo のカメラ描画用メッシュ (sim/models/so101) は学習画像の元なので、ここでは変えない。
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "vendor" / "SO-ARM100" / "Simulation" / "SO101" / "assets"
OUT = ROOT / "sim" / "models" / "so101_web" / "assets"
# 外郭の許容ずれ。0.3mm だと細部 (ネジ穴・丸み) のせいで三角形が 12% しか減らない (実測)。画面上のロボットは
# 1px ≈ 1mm なので 0.5mm はサブピクセル。環境変数 VRSO101_DECIMATE_TOL_MM で変えられる。
TOL_M = float(os.environ.get("VRSO101_DECIMATE_TOL_MM", "0.5")) / 1000
SAMPLES = 5000        # 探索中の粗いサンプル数 (最後に SAMPLES_FINAL で測り直して報告する)
SAMPLES_FINAL = 30000


def _simplify(m: trimesh.Trimesh, faces: int) -> trimesh.Trimesh:
    import fast_simplification as fs
    v, f = fs.simplify(m.vertices.astype(np.float32), m.faces.astype(np.int32), target_count=faces, agg=7)
    return trimesh.Trimesh(v, f, process=False)


def hausdorff(a: trimesh.Trimesh, b: trimesh.Trimesh, n: int = SAMPLES) -> float:
    """a と b の双方向の最大距離 (表面サンプル → 相手の表面への最短距離)。"""
    from trimesh.proximity import closest_point
    worst = 0.0
    for x, y in ((a, b), (b, a)):
        pts, _ = trimesh.sample.sample_surface(x, n, seed=0)
        _, d, _ = closest_point(y, pts)
        worst = max(worst, float(d.max()))
    return worst


def decimate(m: trimesh.Trimesh) -> tuple[trimesh.Trimesh, float]:
    """許容値を満たす最小の三角形数を二分探索し、精密なサンプルで検証して、超えていたら三角形を戻す。
    探索中の粗いサンプルは最悪点を見逃す (実測: 粗い測定 0.5mm 以下でも精密に測ると 1.6mm のずれ)。"""
    lo, hi = 200, len(m.faces)              # lo: 満たさない側 / hi: 満たす側 (元そのもの)
    best = m
    for _ in range(6):
        mid = (lo + hi) // 2
        c = _simplify(m, mid)
        if hausdorff(m, c) <= TOL_M:
            hi, best = mid, c
        else:
            lo = mid
    if best is m:
        return m, 0.0
    err = hausdorff(m, best, SAMPLES_FINAL)
    while err > TOL_M and len(best.faces) < len(m.faces):
        faces = int(len(best.faces) * 1.15) + 1
        best = m if faces >= len(m.faces) else _simplify(m, faces)
        err = 0.0 if best is m else hausdorff(m, best, SAMPLES_FINAL)
    return best, err


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    tot0 = tot1 = 0
    for p in sorted(SRC.glob("*.stl")):
        m = trimesh.load(p)
        m.merge_vertices()
        d, err = decimate(m)
        d.export(OUT / p.name)
        tot0 += len(m.faces); tot1 += len(d.faces)
        print(f"{p.name:44s} {len(m.faces):6d} -> {len(d.faces):6d} 面 ({100*len(d.faces)/len(m.faces):4.1f}%)  最大ずれ {err*1000:.3f} mm", flush=True)
    print(f"合計 {tot0} -> {tot1} 三角形 ({100*tot1/tot0:.1f}%)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
