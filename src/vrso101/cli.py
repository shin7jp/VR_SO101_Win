"""CLI (Web UI は薄い皮にするため、機能は必ずここに先に足す)。

  python -m vrso101.cli collect-scripted --task pick_place --n 10 --seed-start 0
  python -m vrso101.cli episodes --task pick_place
  python -m vrso101.cli export --task pick_place --name v1
  python -m vrso101.cli train --task pick_place --dataset <name> --steps 2000
  python -m vrso101.cli infer --task pick_place --model <model_id> --seeds 10000 10001
"""
from __future__ import annotations

import argparse
import json
import sys

from .paths import data_dir


def _spec_kin():
    from .robot.joints import load_robot_spec
    from .robot.kinematics import Kinematics
    spec = load_robot_spec()
    return spec, Kinematics(spec)


def cmd_collect_scripted(a) -> int:
    from .recording.store import EpisodeStore
    from .runner import make_sim, run_episode, scripted_actions
    from .tasks.registry import get_task

    spec, kin = _spec_kin()
    store = EpisodeStore()
    sim = make_sim(a.backend, spec, kin)
    info = sim.connect()
    sim.load_task(a.task)
    fps = float(get_task(a.task)["fps"])
    ok = 0
    for i in range(a.n):
        seed = a.seed_start + i
        r = run_episode(sim, spec, store, a.task, seed, lambda o: scripted_actions(spec, kin, a.task, o, fps),
                        meta={"collection_mode": "scripted", "backend": info.get("backend", a.backend)})
        ok += r["success"]
        print(f"[{i + 1}/{a.n}] seed={seed} {r['status']} steps={r['steps']} ep={r.get('episode_id')} "
              f"place_err={r['task']['metrics'].get('place_error')}", flush=True)
    print(f"成功 {ok}/{a.n}")
    return 0


def cmd_episodes(a) -> int:
    from .recording.store import EpisodeStore
    for m in EpisodeStore().list(a.task):
        print(f"{m['task_id']:<12} {m['episode_id']} seed={m.get('seed')} {m.get('status'):<8} label={m.get('label')} "
              f"excl={m.get('excluded')} frames={m.get('num_frames')} mode={m.get('collection_mode')}")
    return 0


def cmd_export(a) -> int:
    from .dataset.lerobot_export import export_lerobot, validate_dataset
    from .recording.store import EpisodeStore

    spec, _ = _spec_kin()
    root = data_dir() / "datasets" / a.task / a.name
    meta = export_lerobot(EpisodeStore(), spec, a.task, root, name=a.name, only_success=not a.include_failures,
                          collection_mode=a.mode, progress=lambda i, n, e: print(f"  変換 {i}/{n} {e}", flush=True))
    print(json.dumps({k: meta[k] for k in ("num_episodes", "num_frames", "num_skipped", "outcome_breakdown")}, ensure_ascii=False))
    res = validate_dataset(root, spec)
    print("検証:", "OK" if res["ok"] else f"NG {res['problems']}")
    return 0 if res["ok"] else 1


def cmd_train(a) -> int:
    from .training.trainer import run_training_foreground
    return run_training_foreground(a.task, a.dataset, steps=a.steps, batch_size=a.batch_size, device=a.device,
                                   save_freq=a.save_freq, chunk_size=a.chunk_size, name=a.name, num_workers=a.num_workers)


def cmd_infer(a) -> int:
    from .inference.runtime import run_inference_cli
    return run_inference_cli(a)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="vrso101")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("collect-scripted", help="スクリプトデモを収録 (検証用。collection_mode=scripted)")
    s.add_argument("--task", default="pick_place")
    s.add_argument("--n", type=int, default=5)
    s.add_argument("--seed-start", type=int, default=0)
    s.add_argument("--backend", default="gazebo", choices=["gazebo", "mock"])
    s.set_defaults(fn=cmd_collect_scripted)
    s = sub.add_parser("episodes")
    s.add_argument("--task")
    s.set_defaults(fn=cmd_episodes)
    s = sub.add_parser("export", help="LeRobot データセットへ変換")
    s.add_argument("--task", default="pick_place")
    s.add_argument("--name", required=True)
    s.add_argument("--mode", default=None, help="collection_mode で絞る (scripted | human)")
    s.add_argument("--include-failures", action="store_true")
    s.set_defaults(fn=cmd_export)
    s = sub.add_parser("train", help="ACT 学習 (lerobot-train を起動)")
    s.add_argument("--task", default="pick_place")
    s.add_argument("--dataset", required=True)
    s.add_argument("--steps", type=int, default=2000)
    s.add_argument("--batch-size", type=int, default=8)
    s.add_argument("--device", default="cuda")
    s.add_argument("--save-freq", type=int, default=1000)
    s.add_argument("--chunk-size", type=int, default=50)
    s.add_argument("--num-workers", type=int, default=2)
    s.add_argument("--name", default=None)
    s.set_defaults(fn=cmd_train)
    s = sub.add_parser("infer", help="学習済みポリシーを Gazebo で実行し動画を保存")
    s.add_argument("--task", default="pick_place")
    s.add_argument("--model", required=True)
    s.add_argument("--checkpoint", default="last")
    s.add_argument("--seeds", type=int, nargs="+", default=[10000])
    s.add_argument("--backend", default="gazebo", choices=["gazebo", "mock"])
    s.add_argument("--n-action-steps", type=int, default=10, help="再推論までに実行するアクション数")
    s.add_argument("--max-steps", type=int, default=None)
    s.add_argument("--device", default="cuda")
    s.set_defaults(fn=cmd_infer)
    a = p.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
