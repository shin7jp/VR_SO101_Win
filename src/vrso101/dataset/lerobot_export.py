"""生エピソード → LeRobot 公式データセット (LeRobotDataset v3, lerobot 0.6.1) への変換と検証。

独自形式は作らず、LeRobotDataset.create / add_frame / save_episode / finalize を使う。
特徴量: observation.images.front (video, H×W×3) / observation.state (float32, 6) / action (float32, 6)
単位は関節ラジアン (グリッパー含む)。LeRobot 標準 SO101 の degrees / 0-100 とは別物なので、
`vrso101_meta.json` に単位・関節順を明記して、データセット単体から復元できるようにする。
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np

from .. import __version__
from ..recording.store import EpisodeStore, atomic_write_json, now_iso
from ..robot.joints import RobotSpec
from ..tasks.registry import get_task
from ..video import read_frames

EXPORT_FORMAT_VERSION = "vrso101-export/1"
CAMERA_KEY = "observation.images.front"


def build_features(spec: RobotSpec, height: int, width: int) -> dict[str, dict[str, Any]]:
    return {
        CAMERA_KEY: {"dtype": "video", "shape": (height, width, 3), "names": ["height", "width", "channels"]},
        "observation.state": {"dtype": "float32", "shape": (spec.dof,), "names": list(spec.names)},
        "action": {"dtype": "float32", "shape": (spec.dof,), "names": list(spec.names)},
    }


def select_episodes(store: EpisodeStore, task_id: str, *, only_success: bool = True, collection_mode: str | None = None,
                    episode_ids: list[str] | None = None) -> tuple[list[dict], list[dict]]:
    """(採用, 除外) を返す。除外理由を各 meta に `_skip_reason` として付ける (UI で理由を見せるため)。"""
    used, skipped = [], []
    for m in store.list(task_id):
        if episode_ids is not None and m["episode_id"] not in episode_ids:
            continue
        reason = None
        if m.get("excluded"):
            reason = "利用者が除外"
        elif m.get("status") in ("recording", "aborted", "corrupt"):
            reason = f"status={m.get('status')}"
        elif not m.get("quality", {}).get("ok", False):
            reason = "品質検査 NG"
        elif only_success and (m.get("label") or m.get("status")) != "success":
            reason = "成功ではない"
        elif collection_mode and m.get("collection_mode") != collection_mode:
            reason = f"collection_mode={m.get('collection_mode')}"
        (skipped if reason else used).append({**m, "_skip_reason": reason} if reason else m)
    return used, skipped


def export_lerobot(store: EpisodeStore, spec: RobotSpec, task_id: str, out_root: Path, *, name: str,
                   only_success: bool = True, collection_mode: str | None = None,
                   episode_ids: list[str] | None = None, vcodec: str = "h264", crf: int = 18,
                   progress=None) -> dict[str, Any]:
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    task = get_task(task_id)
    used, skipped = select_episodes(store, task_id, only_success=only_success, collection_mode=collection_mode,
                                    episode_ids=episode_ids)
    if not used:
        raise ValueError(f"変換できるエピソードがありません (除外 {len(skipped)} 件: "
                         f"{sorted({s['_skip_reason'] for s in skipped})})")
    out_root = Path(out_root)
    if out_root.exists():
        raise FileExistsError(f"出力先が既に存在します: {out_root} (別名を指定してください)")
    fps = int(task["fps"])
    shapes = {tuple(m.get("image_shape", ())) for m in used}
    if len(shapes) != 1 or () in shapes:
        raise ValueError(f"エピソード間で画像サイズが揃っていません/不明です: {shapes}")
    (height, width), = shapes
    cam = {**task["camera"], "height": height, "width": width}   # 実際に収録された解像度が正 (設定ではなく)
    feats = build_features(spec, height, width)
    ds = LeRobotDataset.create(
        repo_id=f"local/{task_id}_{name}", fps=fps, features=feats, root=out_root, robot_type=spec.robot_id,
        use_videos=True, video_backend="pyav",
        # H.264 + 短い GOP: 既定の AV1 は iOS Safari 等で再生できず、デコードも遅い。学習への劣化を抑えるため crf は低め
        rgb_encoder=RGBEncoderConfig(vcodec=vcodec, crf=crf, g=2, pix_fmt="yuv420p"))
    t0 = time.time()
    try:
        for i, m in enumerate(used):
            eid = m["episode_id"]
            arr = store.load_arrays(task_id, eid)
            frames = read_frames(store.dir(task_id, eid) / "frames.mkv")
            n = len(arr["q"])
            count = 0
            for k, img in enumerate(frames):
                if k >= n:
                    break
                ds.add_frame({CAMERA_KEY: img, "observation.state": arr["q"][k].astype(np.float32),
                              "action": arr["q_cmd"][k].astype(np.float32), "task": task["instruction"]})
                count += 1
            if count != n:
                ds.clear_episode_buffer()
                raise ValueError(f"{eid}: 動画フレーム数 {count} と状態行数 {n} が一致しません")
            ds.save_episode()
            if progress:
                progress(i + 1, len(used), eid)
    finally:
        ds.finalize()
    meta = {
        "format": EXPORT_FORMAT_VERSION, "generator_version": __version__, "created_at": now_iso(),
        "lerobot_version": _lerobot_version(), "task_id": task_id, "task_name": task["name"],
        "instruction": task["instruction"], "robot_id": spec.robot_id, "joint_names": spec.names,
        "units": spec.units, "joint_limits": {"lower": spec.lower.tolist(), "upper": spec.upper.tolist()},
        "fps": fps, "camera": cam, "features": {k: {"dtype": v["dtype"], "shape": list(v["shape"]), "names": v["names"]}
                                                  for k, v in feats.items()},
        "video": {"vcodec": vcodec, "crf": crf, "pix_fmt": "yuv420p", "gop": 2},
        "num_episodes": len(used), "num_frames": int(sum(m["num_frames"] for m in used)),
        "num_skipped": len(skipped), "skip_reasons": _count([s["_skip_reason"] for s in skipped]),
        "outcome_breakdown": _count([(m.get("label") or m.get("status")) for m in used]),
        "collection_modes": _count([m.get("collection_mode", "unknown") for m in used]),
        "episodes": [{"episode_id": m["episode_id"], "seed": m.get("seed"), "num_frames": m["num_frames"],
                      "status": m["status"], "label": m.get("label"), "collection_mode": m.get("collection_mode"),
                      "scene": m.get("scene")} for m in used],
        "seeds": sorted(m["seed"] for m in used if m.get("seed") is not None),
        "export_seconds": round(time.time() - t0, 1),
    }
    atomic_write_json(out_root / "vrso101_meta.json", meta)
    return meta


def _count(items) -> dict[str, int]:
    out: dict[str, int] = {}
    for it in items:
        out[str(it)] = out.get(str(it), 0) + 1
    return out


def _lerobot_version() -> str:
    try:
        import lerobot
        return lerobot.__version__
    except Exception:   # pragma: no cover
        return "unknown"


def validate_dataset(root: Path, spec: RobotSpec | None = None, *, check_decode: bool = True) -> dict[str, Any]:
    """LeRobotDataset として実際に読み込み、特徴量・次元・フレーム数・画像デコードを検証する。"""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(root)
    problems: list[str] = []
    vm_path = root / "vrso101_meta.json"
    if not vm_path.exists():
        return {"ok": False, "problems": ["vrso101_meta.json がありません (このアプリで作ったデータセットではない)"]}
    vm = json.loads(vm_path.read_text(encoding="utf-8"))
    try:
        ds = LeRobotDataset(f"local/{vm['task_id']}", root=root, video_backend="pyav")
    except Exception as e:
        return {"ok": False, "problems": [f"LeRobotDataset として読めません: {type(e).__name__}: {e}"]}
    feats = ds.meta.features
    for key in ("observation.state", "action", CAMERA_KEY):
        if key not in feats:
            problems.append(f"特徴量 {key} がありません")
    if spec is not None:
        for key in ("observation.state", "action"):
            if key in feats and (list(feats[key]["names"]) != spec.names or tuple(feats[key]["shape"]) != (spec.dof,)):
                problems.append(f"{key} の関節定義が現在のロボット設定と一致しません")
    if ds.meta.total_episodes != vm["num_episodes"]:
        problems.append(f"エピソード数不一致: {ds.meta.total_episodes} != {vm['num_episodes']}")
    if ds.meta.total_frames != vm["num_frames"]:
        problems.append(f"フレーム数不一致: {ds.meta.total_frames} != {vm['num_frames']}")
    if int(ds.fps) != int(vm["fps"]):
        problems.append(f"fps 不一致: {ds.fps} != {vm['fps']}")
    decoded = None
    if check_decode and not problems and len(ds) > 0:
        try:
            item = ds[len(ds) // 2]
            img = item[CAMERA_KEY]
            decoded = {"shape": list(img.shape), "min": float(img.min()), "max": float(img.max())}
            h, w = vm["camera"]["height"], vm["camera"]["width"]
            if tuple(img.shape) != (3, h, w):
                problems.append(f"デコード画像の形状 {tuple(img.shape)} != (3,{h},{w})")
            if item["observation.state"].shape[0] != len(vm["joint_names"]):
                problems.append("state の次元が関節数と一致しません")
        except Exception as e:
            problems.append(f"動画のデコードに失敗: {type(e).__name__}: {e}")
    return {"ok": not problems, "problems": problems, "num_episodes": ds.meta.total_episodes,
            "num_frames": ds.meta.total_frames, "fps": ds.fps, "decoded_sample": decoded, "meta": vm}
