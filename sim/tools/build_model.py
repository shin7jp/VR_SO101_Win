"""vendor の SO-101 URDF から Gazebo(Fortress) 用 SDF モデルを生成する。

なぜ URDF 直読みにしないか: sdformat の URDF 変換だと JointPositionController 等のプラグインや
摩擦・effort を後付けできず、`package://` / 相対メッシュパスの解決も環境依存になる。
生成物 (sim/models/so101/) は再生成可能なので vendor は上流のまま触らない。

  python sim/tools/build_model.py            # Windows 側 venv で実行 (numpy 必要)
"""
from __future__ import annotations

import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import os

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from vrso101.robot.joints import load_robot_spec  # noqa: E402
from vrso101.robot.kinematics import rpy_to_matrix  # noqa: E402

SRC_DIR = ROOT / "vendor" / "SO-ARM100" / "Simulation" / "SO101"
OUT_DIR = ROOT / "sim" / "models" / "so101"

# 関節ごとの駆動パラメータ。velocity 型 PID(JointPositionController) の P ゲインは経験的に調整する値。
CTRL = {
    "shoulder_pan": dict(p=25.0, d=0.0, effort=8.0),
    "shoulder_lift": dict(p=25.0, d=0.0, effort=8.0),
    "elbow_flex": dict(p=25.0, d=0.0, effort=8.0),
    "wrist_flex": dict(p=25.0, d=0.0, effort=5.0),
    "wrist_roll": dict(p=25.0, d=0.0, effort=3.0),
    "gripper": dict(p=20.0, d=0.0, effort=1.5),   # 立方体を潰さないよう小さめ
}
MIN_I = 1e-8   # 仮想リンク(gripper_frame_link)の慣性ゼロは gz が警告し数値的にも危うい
# 把持面になるメッシュだけ凸分解で凹み(指の間の隙間)を残す。他は凸包1個で十分 (自己衝突は無効)。
CONCAVE_MESHES = {"moving_jaw_so101_v1.stl", "wrist_roll_follower_so101_v1.stl"}
# 把持面の凸分解の最大個数。多いほど接触点が増え、掴んで持ち上げた間の物理が重くなる (実測: 運搬中 ~335ms/step)。
JAW_HULLS = int(os.environ.get("VRSO101_JAW_HULLS", "8"))
HULL_VERTS = int(os.environ.get("VRSO101_HULL_VERTS", "96"))   # 衝突用凸包 1 個あたりの最大頂点数
JAW_LINKS = {"gripper_link", "moving_jaw_so101_v1_link"}   # 把持する面。摩擦を高くする


def _vec(s: str | None, n: int = 3) -> list[float]:
    return [float(v) for v in (s or " ".join(["0"] * n)).split()]


def _T(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def _pose_str(T: np.ndarray) -> str:
    R = T[:3, :3]
    # SDF の pose は x y z roll pitch yaw (ZYX 外因)
    pitch = np.arcsin(-np.clip(R[2, 0], -1, 1))
    if abs(np.cos(pitch)) > 1e-6:
        roll, yaw = np.arctan2(R[2, 1], R[2, 2]), np.arctan2(R[1, 0], R[0, 0])
    else:
        roll, yaw = np.arctan2(-R[1, 2], R[1, 1]), 0.0
    x, y, z = T[:3, 3]
    return f"{x:.8g} {y:.8g} {z:.8g} {roll:.8g} {pitch:.8g} {yaw:.8g}"


def _thin_hull(h, max_verts: int = HULL_VERTS):
    """凸包の頂点を最遠点サンプリングで間引いて凸包を取り直す。元 STL が細かいので凸包でも頂点が千個超になり、
    掴んで持ち上げている間の接触計算が重い (実測: 運搬中 ~335ms/step、掴む前は ~28ms)。形は ~1mm 以内で保たれる。"""
    import trimesh
    v = np.asarray(h.vertices)
    if len(v) <= max_verts:
        return h
    idx = [int(np.argmax(np.linalg.norm(v - v.mean(axis=0), axis=1)))]
    d = np.linalg.norm(v - v[idx[0]], axis=1)
    while len(idx) < max_verts:
        i = int(np.argmax(d))
        idx.append(i)
        d = np.minimum(d, np.linalg.norm(v - v[i], axis=1))
    return trimesh.Trimesh(v[idx]).convex_hull


def collision_pieces(fn: str, out_dir: Path) -> list[str]:
    """元 STL (数万三角形) をそのまま衝突に使うと、腕が動くたびに BVH 更新で 1 反復 ~50ms かかる (実測)。
    凸包/凸分解で数十〜数百面に落とした衝突用メッシュを作り、そのファイル名一覧を返す。"""
    import trimesh
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(fn).stem
    m = trimesh.load(SRC_DIR / "assets" / fn)
    if fn in CONCAVE_MESHES:
        import coacd
        coacd.set_log_level("error")
        parts = coacd.run_coacd(coacd.Mesh(m.vertices, m.faces), threshold=0.03, max_convex_hull=JAW_HULLS, seed=0)
        hulls = [trimesh.Trimesh(v, f).convex_hull for v, f in parts]
    else:
        hulls = [m.convex_hull]
    hulls = [_thin_hull(h) for h in hulls]
    names = []
    for k, h in enumerate(hulls):
        name = f"{stem}_{k}.stl"
        h.export(out_dir / name)
        names.append(name)
    return names


def build(model_name: str = "so101", urdf_name: str = "so101_new_calib.urdf") -> Path:
    spec = load_robot_spec()
    root = ET.parse(SRC_DIR / urdf_name).getroot()
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    links = {l.get("name"): l for l in root.findall("link")}

    # リンクの world(=base_link) 姿勢を親から順に積む
    abs_T: dict[str, np.ndarray] = {"base_link": np.eye(4)}

    def link_pose(name: str) -> np.ndarray:
        if name in abs_T:
            return abs_T[name]
        j = joints[name]
        o = j.find("origin")
        T = link_pose(j.find("parent").get("link")) @ _T(_vec(o.get("xyz")), _vec(o.get("rpy")))
        abs_T[name] = T
        return T

    mesh_dir = OUT_DIR / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    used_meshes: set[str] = set()

    out = [f'<?xml version="1.0"?>', '<sdf version="1.8">', f'<model name="{model_name}">']
    for name, link in links.items():
        out.append(f'<link name="{name}"><pose>{_pose_str(link_pose(name))}</pose>')
        inertial = link.find("inertial")
        if inertial is not None:
            o, m, i = inertial.find("origin"), inertial.find("mass"), inertial.find("inertia")
            out.append(f'<inertial><pose>{_pose_str(_T(_vec(o.get("xyz")), _vec(o.get("rpy"))))}</pose>'
                       f'<mass>{max(float(m.get("value")), 1e-6)}</mass><inertia>'
                       + "".join(f"<{k}>{max(float(i.get(k)), MIN_I):.8g}</{k}>" if k in ("ixx", "iyy", "izz") else f"<{k}>{float(i.get(k)):.8g}</{k}>"
                         for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"))
                       + "</inertia></inertial>")
        for kind in ("visual", "collision"):
            for n, el in enumerate(link.findall(kind)):
                mesh = el.find("geometry/mesh")
                if mesh is None:
                    continue
                fn = Path(mesh.get("filename")).name
                used_meshes.add(fn)
                o = el.find("origin")
                pose = _pose_str(_T(_vec(o.get("xyz")), _vec(o.get("rpy")))) if o is not None else "0 0 0 0 0 0"
                if kind == "visual":
                    geo = f'<geometry><mesh><uri>model://{model_name}/meshes/{fn}</uri></mesh></geometry>'
                    out.append(f'<visual name="v{n}"><pose>{pose}</pose>{geo}<material><ambient>0.9 0.9 0.9 1</ambient>'
                               f'<diffuse>0.95 0.55 0.1 1</diffuse></material></visual>')
                else:
                    mu = 2.0 if name in JAW_LINKS else 1.0
                    for k, piece in enumerate(collision_pieces(fn, mesh_dir / "collision")):
                        geo = f'<geometry><mesh><uri>model://{model_name}/meshes/collision/{piece}</uri></mesh></geometry>'
                        out.append(f'<collision name="c{n}_{k}"><pose>{pose}</pose>{geo}<surface><friction><ode>'
                                   f'<mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>')
        out.append("</link>")
    for name, j in joints.items():
        jt = j.get("type")
        parent = j.find("parent").get("link")
        if jt == "fixed":
            out.append(f'<joint name="{j.get("name")}" type="fixed"><parent>{parent}</parent><child>{name}</child></joint>')
            continue
        lim = j.find("limit")
        jn = j.get("name")
        eff = CTRL.get(jn, {}).get("effort", float(lim.get("effort")))
        out.append(
            f'<joint name="{jn}" type="revolute"><parent>{parent}</parent><child>{name}</child>'
            f'<axis><xyz>{" ".join(str(v) for v in _vec(j.find("axis").get("xyz")))}</xyz>'
            f'<limit><lower>{lim.get("lower")}</lower><upper>{lim.get("upper")}</upper>'
            f'<effort>{eff}</effort><velocity>{lim.get("velocity")}</velocity></limit>'
            f'<dynamics><damping>0.05</damping><friction>0.02</friction></dynamics></axis></joint>')
    # base_link を world に固定 (台に据え付け)
    out.append('<joint name="world_fix" type="fixed"><parent>world</parent><child>base_link</child></joint>')
    # 制御系: 関節ごとの位置コントローラ (init はホーム姿勢) と関節状態パブリッシャ
    for jn, home in zip(spec.names, spec.home):
        c = CTRL[jn]
        out.append(
            f'<plugin filename="ignition-gazebo-joint-position-controller-system" '
            f'name="ignition::gazebo::systems::JointPositionController">'
            f'<joint_name>{jn}</joint_name><topic>/{model_name}/cmd/{jn}</topic>'
            f'<p_gain>{c["p"]}</p_gain><i_gain>0.0</i_gain><d_gain>{c["d"]}</d_gain>'
            f'<initial_position>{home}</initial_position></plugin>')
    out.append('<plugin filename="ignition-gazebo-joint-state-publisher-system" '
               'name="ignition::gazebo::systems::JointStatePublisher">'
               + "".join(f"<joint_name>{n}</joint_name>" for n in spec.names) + "</plugin>")
    out += ["</model>", "</sdf>"]

    (OUT_DIR).mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "model.sdf").write_text("\n".join(out), encoding="utf-8")
    (OUT_DIR / "model.config").write_text(
        f'<?xml version="1.0"?><model><name>{model_name}</name><version>1.0</version>'
        f'<sdf version="1.8">model.sdf</sdf><description>SO-101 (generated from TheRobotStudio/SO-ARM100, Apache-2.0)'
        f'</description></model>', encoding="utf-8")
    for fn in used_meshes:
        shutil.copy2(SRC_DIR / "assets" / fn, mesh_dir / fn)
    return OUT_DIR / "model.sdf"


if __name__ == "__main__":
    print(build())
