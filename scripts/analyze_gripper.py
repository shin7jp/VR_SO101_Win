"""グリッパーの開閉方向を EE(gripper_frame_link) 座標で調べる (スクリプト把持デモの設計用診断)。"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from vrso101.robot.joints import load_robot_spec  # noqa: E402
from vrso101.robot.kinematics import Kinematics, axis_angle, rpy_to_matrix  # noqa: E402

spec = load_robot_spec()
kin = Kinematics(spec)
root = ET.parse(ROOT / spec.urdf).getroot()


def origin_T(el):
    o = el.find("origin")
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*[float(v) for v in o.get("rpy").split()])
    T[:3, 3] = [float(v) for v in o.get("xyz").split()]
    return T


joints = {j.get("name"): j for j in root.findall("joint")}
links = {l.get("name"): l for l in root.findall("link")}
T_g_ee = origin_T(joints["gripper_frame_joint"])           # gripper_link -> ee
T_g_jaw0 = origin_T(joints["gripper"])                     # gripper_link -> jaw(q=0)


def mesh_pts_in_ee(link_name, T_link_to_gripper):
    pts = []
    for vis in links[link_name].findall("visual"):
        fn = Path(vis.find("geometry/mesh").get("filename")).name
        m = trimesh.load(ROOT / "vendor/SO-ARM100/Simulation/SO101/assets" / fn)
        T = np.linalg.inv(T_g_ee) @ T_link_to_gripper @ origin_T(vis)
        pts.append(trimesh.transform_points(m.vertices, T))
    return np.vstack(pts)


np.set_printoptions(precision=3, suppress=True)
# 固定顎 (gripper_link に付くメッシュのうち wrist_roll_follower) と動顎
fixed = mesh_pts_in_ee("gripper_link", np.eye(4))
print("EE 原点 = 指先中心。固定顎メッシュ点群の範囲(EE座標):", fixed.min(0), fixed.max(0))
for q in (0.0, 0.5, 1.0, 1.7):
    R = np.eye(4)
    R[:3, :3] = axis_angle(np.array([0, 0, 1.0]), q)
    jaw = mesh_pts_in_ee("moving_jaw_so101_v1_link", T_g_jaw0 @ R)
    # 指先側 (EE 原点付近, z が EE で -0.02..0.05) の点の重心
    sel = jaw[(jaw[:, 2] > -0.03) & (jaw[:, 2] < 0.06)]
    print(f"q_grip={q:.1f}: 動顎 範囲 min={jaw.min(0)} max={jaw.max(0)}  指先近傍重心={sel.mean(0) if len(sel) else None}")
print("ホーム姿勢の EE 軸 (ワールド):\n", kin.fk(spec.home)[:3, :3])

# --- 指先付近 (z_ee ∈ [-0.04, 0.0], |y_ee| < 0.01) の固定顎/動顎の x 範囲 → 把持中心のオフセット ---
def band(P):
    s = P[(P[:, 2] > -0.04) & (P[:, 2] < 0.0) & (np.abs(P[:, 1]) < 0.01)]
    return (s[:, 0].min(), s[:, 0].max()) if len(s) else None


print("固定顎 指先帯 x 範囲:", band(fixed))
for q in (0.0, 0.4, 0.8):
    R = np.eye(4)
    R[:3, :3] = axis_angle(np.array([0, 0, 1.0]), q)
    print(f"動顎(q={q}) 指先帯 x 範囲:", band(mesh_pts_in_ee("moving_jaw_so101_v1_link", T_g_jaw0 @ R)))
