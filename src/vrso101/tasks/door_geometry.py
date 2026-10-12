"""ドア周りの寸法の正本 (標準ライブラリのみ。WSL のブリッジと Windows のスクリプトデモが共有)。

ドア座標系: 原点=ヒンジ軸と作業台面の交点、+y=自由端方向、-x=ロボット側の面。
"""
from __future__ import annotations

from typing import Any

Z0 = 0.004   # ドア下端と作業台の隙間 (擦らない)


def door_geometry(task: dict[str, Any]) -> dict[str, float]:
    """ドア周りの寸法 (ドア座標系: 原点=ヒンジ軸と作業台面の交点、+y=自由端方向、-x=ロボット側の面)。
    スクリプトデモ (teleop/scripted_door.py) も同じ値を使うので、ここを正本にする。"""
    d = task["scene"]["door"]
    w, h, t = d["width"], d["height"], d["thickness"]
    g = {"w": w, "h": h, "t": t, "z0": Z0}
    if "handle" in task["scene"]:
        hd = task["scene"]["handle"]
        g.update(handle_y=w - hd["edge_offset"], handle_x=-t / 2 - hd["grasp_offset"], handle_z=Z0 + hd["height"],
                 peg_r=hd["peg_radius"], peg_len=hd["peg_length"], head_r=hd["head_radius"], head_t=hd["head_thickness"])
    if "lever" in task["scene"]:
        lv = task["scene"]["lever"]
        g.update(spindle_y=w - lv["edge_offset"], spindle_z=Z0 + lv["axis_height"], lever_len=lv["length"],
                 lever_r=lv["radius"], lever_x=-t / 2 - lv.get("standoff", 0.014), lever_spring=lv.get("spring", 0.0),
                 lever_friction=lv.get("friction", 0.002 if lv.get("spring", 0.0) > 0 else 0.02))
    return g
