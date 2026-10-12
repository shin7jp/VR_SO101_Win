"""評価の集計。収録データ品質・学習結果・Gazebo 推論結果を分離して扱う。

推論結果は data/inference/<task>/<episode_id>/metadata.json から読み出す (再起動後も復元できる)。
単一シードの成功を一般的な成功とみなさないよう、シード数と Wilson 区間も返す。
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from ..paths import data_dir


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def load_inference_runs(task_id: str | None = None) -> list[dict[str, Any]]:
    root = data_dir() / "inference"
    out = []
    if not root.exists():
        return out
    for tdir in sorted(p for p in root.iterdir() if p.is_dir()):
        if task_id and tdir.name != task_id:
            continue
        for edir in sorted((p for p in tdir.iterdir() if p.is_dir()), reverse=True):
            f = edir / "metadata.json"
            if f.exists():
                try:
                    out.append(json.loads(f.read_text(encoding="utf-8")))
                except json.JSONDecodeError:
                    pass
    return out


def summarize(runs: list[dict[str, Any]]) -> dict[str, Any]:
    n = len(runs)
    if n == 0:
        return {"n": 0}
    succ = [r for r in runs if r.get("success")]
    seeds = sorted({r.get("seed") for r in runs if r.get("seed") is not None})
    lo, hi = wilson(len(succ), n)
    times = [r["sim_seconds"] for r in succ if r.get("sim_seconds") is not None]
    outcomes: dict[str, int] = {}
    for r in runs:
        outcomes[r.get("outcome", "?")] = outcomes.get(r.get("outcome", "?"), 0) + 1
    metrics = [r.get("task_result", {}).get("metrics", {}) for r in runs if r.get("task_result")]

    def avg(key, only=None):
        vals = [m[key] for m, r in zip(metrics, runs) if key in m and (only is None or only(r))]
        return float(np.mean(vals)) if vals else None
    return {
        "n": n, "n_success": len(succ), "success_rate": len(succ) / n, "success_ci95": [lo, hi],
        "timeout_rate": outcomes.get("timeout", 0) / n, "abnormal_rate": outcomes.get("abnormal", 0) / n,
        "outcomes": outcomes, "mean_time_to_complete_s": float(np.mean(times)) if times else None,
        "mean_inference_fps": float(np.mean([r["inference_fps"] for r in runs if r.get("inference_fps")]) or 0) or None,
        "unique_seeds": len(seeds), "eval_split_runs": sum(1 for r in runs if r.get("evaluation_split") == "eval"),
        "overlap_runs": sum(1 for r in runs if r.get("train_eval_overlap")),
        # タスク別指標 (存在するものだけ)
        "grasp_success_rate": avg("grasped_lifted"), "mean_place_error_m": avg("place_error", lambda r: r.get("success")),
        "mean_door_angle_rad": avg("max_door_angle"), "lever_actuated_rate": avg("latch_released"),
        "saved_rate": sum(1 for r in runs if r.get("saved")) / n,
    }


def compare(task_id: str) -> dict[str, Any]:
    """チェックポイント (モデル × step) ごとの比較表。同じシード集合で比較できているかも返す。"""
    runs = load_inference_runs(task_id)
    groups: dict[tuple, list] = {}
    for r in runs:
        p = r.get("policy", {})
        groups.setdefault((p.get("model_id"), p.get("checkpoint"), p.get("n_action_steps")), []).append(r)
    rows = []
    for (mid, ck, nas), rs in groups.items():
        rows.append({"model_id": mid, "checkpoint": ck, "n_action_steps": nas, "seeds": sorted({r["seed"] for r in rs}),
                     **summarize(rs)})
    all_seed_sets = {tuple(r["seeds"]) for r in rows}
    return {"task_id": task_id, "rows": sorted(rows, key=lambda x: -x["success_rate"]) if rows else [],
            "same_seed_set": len(all_seed_sets) <= 1, "total_runs": len(runs)}
