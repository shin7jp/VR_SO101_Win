"""収録エピソードの品質検査。不正データ・不完全エピソードを「消さずに」指摘する (除外は利用者が決める)。"""
from __future__ import annotations

from typing import Any

import numpy as np


def check_episode(arrays: dict[str, np.ndarray], fps: float, lower: np.ndarray, upper: np.ndarray,
                  *, expected_frames: int | None = None) -> dict[str, Any]:
    """arrays: sim_time, q, q_cmd, image_stamp, joint_stamp, step_wall_ms (すべて長さ N)。"""
    issues: list[dict[str, str]] = []
    n = int(len(arrays["sim_time"]))
    dt = 1.0 / fps

    def add(code: str, msg: str, level: str = "error"):
        issues.append({"code": code, "level": level, "message": msg})

    if n == 0:
        add("EMPTY", "フレームが 1 枚もありません")
        return {"ok": False, "num_frames": 0, "issues": issues}
    t = arrays["sim_time"]
    d = np.diff(t)
    if np.any(d <= 0):
        add("TIMESTAMP_NONMONOTONIC", f"シム時刻が逆行/重複しています ({int(np.sum(d <= 0))} 箇所)")
    gaps = d[d > 0] / dt
    dropped = int(np.sum(np.round(gaps[gaps > 1.5]) - 1)) if gaps.size else 0
    if dropped:
        add("FRAME_DROP", f"フレーム欠落 {dropped} 枚 (周期が 1.5 倍を超える間隔)")
    if gaps.size and np.any(np.abs(gaps - 1.0) > 0.05) and not dropped:
        add("PERIOD_JITTER", "フレーム周期が一定ではありません", "warning")
    for k in ("q", "q_cmd"):
        if not np.all(np.isfinite(arrays[k])):
            add("NONFINITE", f"{k} に NaN/inf が含まれます")
    if np.any(arrays["q_cmd"] < lower - 1e-6) or np.any(arrays["q_cmd"] > upper + 1e-6):
        add("COMMAND_OUT_OF_LIMITS", "関節指令が上下限を超えています")
    lag = arrays["joint_stamp"] - arrays["image_stamp"]
    if np.any(lag < -1e-6) or np.any(lag > 2.5 * dt / 10 + 1e-6 + 0.01):
        add("IMAGE_STATE_SKEW", f"画像と関節状態の時刻差が大きい (最大 {float(np.max(np.abs(lag))) * 1000:.1f} ms)")
    if expected_frames is not None and n < expected_frames:
        add("INCOMPLETE", f"期待 {expected_frames} フレームに対し {n} フレーム (途中終了)", "warning")
    if n < 10:
        add("TOO_SHORT", f"フレーム数が少なすぎます ({n})", "warning")
    wall = arrays.get("step_wall_ms")
    stats = {}
    if wall is not None and len(wall):
        w = np.asarray(wall, dtype=float)
        stats = {"step_ms_median": float(np.median(w)), "step_ms_p95": float(np.percentile(w, 95)),
                 "step_ms_max": float(np.max(w)), "overrun_frames": int(np.sum(w > dt * 1000))}
    return {"ok": not any(i["level"] == "error" for i in issues), "num_frames": n, "dropped_frames": dropped,
            "issues": issues, "latency": stats}
