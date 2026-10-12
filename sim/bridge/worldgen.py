"""タスク設定 → Gazebo ワールド SDF / タスク物体 SDF の生成 (標準ライブラリのみ)。

ワールドは「ロボット・作業台・照明・カメラ」だけの静的部分。タスク物体 (立方体・ドアなど) は
エピソードごとにシードで配置が変わるので、ワールドには入れず SpawnEntity サービスで生成する。
物理ステップ = 1 / (fps × substeps): 制御 1 周期が整数ステップになり、カメラ更新がずれない。
"""
from __future__ import annotations

from typing import Any

WORLD_NAME = "so101_world"
ROBOT_MODEL = "so101"


def _pose(p) -> str:
    return " ".join(f"{float(v):.8g}" for v in p)


def make_world_sdf(task: dict[str, Any], *, substeps: int = 10, engine: str = "ogre",
                   sensors_per_s: float | None = None) -> str:
    fps = float(task["fps"])
    dt = 1.0 / (fps * substeps)
    # Gazebo の sim 時刻は整数 ns。制御周期・物理 dt が整数 ns でないと、カメラの更新位相が
    # 周期ごとに数 ns ずつずれて「画像が 1 ステップ遅れて出る/出ない」が不規則になる (実測)。
    for label, sec in (("制御周期", 1.0 / fps), ("物理dt", dt)):
        ns = sec * 1e9
        if abs(ns - round(ns)) > 1e-3:
            raise ValueError(f"fps={fps} substeps={substeps}: {label}={ns:.4f}ns が整数 ns になりません。fps を 25/20/40/50 等にしてください")
    cam = task["camera"]
    rate = sensors_per_s or fps
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="{WORLD_NAME}">
    <physics name="p" type="dart"><max_step_size>{dt:.10g}</max_step_size><real_time_factor>0</real_time_factor></physics>
    <gravity>0 0 -9.81</gravity>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>{engine}</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <pose>0 0 3 0 0 0</pose><direction>-0.4 0.2 -1</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><cast_shadows>true</cast_shadows>
    </light>
    <model name="table"><static>true</static><pose>0.2 0 -0.02 0 0 0</pose><link name="l">
      <collision name="c"><geometry><box><size>1.2 1.2 0.04</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><box><size>1.2 1.2 0.04</size></box></geometry>
        <material><ambient>0.55 0.5 0.45 1</ambient><diffuse>0.55 0.5 0.45 1</diffuse></material></visual>
    </link></model>
    <include><uri>model://{ROBOT_MODEL}</uri><name>{ROBOT_MODEL}</name><pose>0 0 0 0 0 0</pose></include>
    <model name="front_camera"><static>true</static><pose>{_pose(cam["pose"])}</pose><link name="l">
      <sensor name="{cam["name"]}" type="camera">
        <camera><horizontal_fov>{cam["hfov"]}</horizontal_fov>
          <image><width>{cam["width"]}</width><height>{cam["height"]}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>6</far></clip></camera>
        <always_on>1</always_on><update_rate>{rate}</update_rate><topic>/camera/{cam["name"]}</topic>
      </sensor>
    </link></model>
  </world>
</sdf>
"""


def make_pick_place_objects(task: dict[str, Any], scene: dict[str, Any]) -> list[tuple[str, str, tuple]]:
    """[(entity_name, sdf, pose6)] を返す。立方体は動的、目標ゾーンは見た目だけの薄板 (衝突なし)。
    pose は SDF 内に書かず別に返す: ROS の SpawnEntity は pose 欄が常に埋まっていて (既定=原点)、
    SDF 内の <pose> を黙って上書きしてしまうため (実測: 物体が全部原点に出た)。"""
    c = task["scene"]["cube"]
    s = c["size"]
    m = c["mass"]
    i = m * s * s / 6.0
    col = " ".join(str(v) for v in c["color"])
    cu = scene["cube"]
    cube = f"""<sdf version="1.8"><model name="cube">
<link name="l"><inertial><mass>{m}</mass><inertia><ixx>{i:.4e}</ixx><iyy>{i:.4e}</iyy><izz>{i:.4e}</izz></inertia></inertial>
<collision name="c"><geometry><box><size>{s} {s} {s}</size></box></geometry>
<surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface></collision>
<visual name="v"><geometry><box><size>{s} {s} {s}</size></box></geometry><material><ambient>{col}</ambient><diffuse>{col}</diffuse></material></visual>
</link></model></sdf>"""
    t = task["scene"]["target"]
    h = t["half_extent"]
    target = f"""<sdf version="1.8"><model name="target"><static>true</static>
<link name="l"><visual name="v"><geometry><box><size>{2 * h} {2 * h} 0.001</size></box></geometry>
<material><ambient>0.1 0.7 0.2 1</ambient><diffuse>0.1 0.7 0.2 1</diffuse></material></visual></link></model></sdf>"""
    return [("target", target, (t["center"][0], t["center"][1], 0.0005, 0, 0, 0)),
            ("cube", cube, (cu["x"], cu["y"], cu["z"] + 0.001, 0, 0, cu["yaw"]))]


def _box(name, size, pose, color, collide=True, mu=1.0, extra=""):
    sx, sy, sz = size
    col = (f'<collision name="{name}_c"><pose>{_pose(pose)}</pose><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
           f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>') if collide else ""
    return (f'{col}<visual name="{name}_v"><pose>{_pose(pose)}</pose><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
            f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>')


def _cyl(name, radius, length, pose, color, collide=True, mu=1.5):
    """見た目は円柱、衝突は同寸の直方体。円柱 × 把持メッシュ (顎の凸包) の接触は DART で極端に遅い
    (顎を閉じた瞬間に 1 反復 80ms: 実測)。直方体 (立方体の把持で実績あり) なら高速。"""
    geo = f'<geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>'
    col = ""
    if collide:
        d = 2 * radius
        col = (f'<collision name="{name}_c"><pose>{_pose(pose)}</pose><geometry><box><size>{d} {d} {length}</size></box></geometry>'
               f'<surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>')
    return f'{col}<visual name="{name}_v"><pose>{_pose(pose)}</pose>{geo}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>'


from vrso101.tasks.door_geometry import Z0, door_geometry  # noqa: E402  (寸法の正本。スクリプトデモと共有)


def make_door_objects(task: dict[str, Any], scene: dict[str, Any]) -> list[tuple[str, str, tuple]]:
    g = door_geometry(task)
    d = task["scene"]["door"]
    w, h, t, z0 = g["w"], g["h"], g["t"], g["z0"]
    hx, hy = scene["door"]["hinge_x"], scene["door"]["hinge_y"]
    is_lever = "lever" in task["scene"]
    panel_mass = 0.05
    ixx = panel_mass * (w * w + h * h) / 12 + panel_mass * (w / 2) ** 2   # ヒンジ軸まわりの概算
    joints = ["door_hinge"] + (["lever"] if is_lever else [])
    body = [_box("panel", (t, w, h), (0, w / 2, z0 + h / 2, 0, 0, 0), "0.75 0.65 0.5 1")]
    if not is_lever:   # 頭付きペグ: 面から水平に突き出した軸 + 先端の円盤。頭の裏 (面側) に顎を回して引く
        hz, hyp = g["handle_z"], g["handle_y"]
        pr, pl, hr, ht = g["peg_r"], g["peg_len"], g["head_r"], g["head_t"]
        # 円柱の軸は SDF 既定で z → pitch 90° で x 軸向き
        body.append(_cyl("peg", pr, pl, (-t / 2 - pl / 2, hyp, hz, 0, 1.5708, 0), "0.2 0.2 0.25 1"))
        body.append(_cyl("head", hr, ht, (-t / 2 - pl - ht / 2, hyp, hz, 0, 1.5708, 0), "0.15 0.15 0.2 1"))
    panel = (f'<link name="panel"><pose>0 0 0 0 0 0</pose><inertial><pose>0 {w / 2} {z0 + h / 2} 0 0 0</pose><mass>{panel_mass}</mass>'
             f'<inertia><ixx>{ixx:.3e}</ixx><iyy>{ixx:.3e}</iyy><izz>{ixx:.3e}</izz></inertia></inertial>' + "".join(body) + "</link>")
    jx = (f'<joint name="door_hinge" type="revolute"><parent>world</parent><child>panel</child><axis><xyz>0 0 1</xyz>'
          f'<limit><lower>-1.4</lower><upper>1.4</upper></limit><dynamics><damping>{d["damping"]}</damping><friction>{d["friction"]}</friction></dynamics></axis></joint>')
    lever = ""
    plugins = ""
    if is_lever:
        sy, sz, L, r, lx = g["spindle_y"], g["spindle_z"], g["lever_len"], g["lever_r"], g["lever_x"]
        lever = (f'<link name="lever_link"><pose>{-t / 2} {sy} {sz} 0 0 0</pose><inertial><pose>{lx + t / 2} {L / 2} 0 0 0 0</pose><mass>0.01</mass>'
                 f'<inertia><ixx>2e-6</ixx><iyy>2e-6</iyy><izz>2e-6</izz></inertia></inertial>'
                 + _cyl("hub", 0.009, -lx - t / 2, ((lx + t / 2) / 2, 0, 0, 0, 1.5708, 0), "0.2 0.2 0.25 1", collide=False)
                 + _cyl("arm", r, L, (lx + t / 2, L / 2, 0, 1.5708, 0, 0), "0.2 0.2 0.25 1", mu=1.5) + "</link>")
        jx += (f'<joint name="lever" type="revolute"><parent>panel</parent><child>lever_link</child><axis><xyz>-1 0 0</xyz>'
               f'<limit><lower>0</lower><upper>0.9</upper></limit><dynamics><damping>0.003</damping><friction>{g["lever_friction"]}</friction></dynamics></axis></joint>')
        if g["lever_spring"] > 0:   # ばね戻りあり (spring>0)。0 なら摩擦で下げた位置に留まる (サムラッチ)
            plugins += (f'<plugin filename="ignition-gazebo-joint-position-controller-system" name="ignition::gazebo::systems::JointPositionController">'
                    f'<joint_name>lever</joint_name><topic>/door/cmd/lever</topic><p_gain>{g["lever_spring"]}</p_gain><i_gain>0</i_gain><d_gain>0.0</d_gain>'
                    f'<cmd_min>-0.5</cmd_min><cmd_max>0.5</cmd_max></plugin>')   # 目標 0 への弱いばね (離すと戻る)
    plugins += ('<plugin filename="ignition-gazebo-joint-state-publisher-system" name="ignition::gazebo::systems::JointStatePublisher">'
                + "".join(f"<joint_name>{j}</joint_name>" for j in joints) + "</plugin>")
    door_sdf = f'<sdf version="1.8"><model name="door">{panel}{lever}{jx}{plugins}</model></sdf>'
    # 静的フレーム: ヒンジ柱 (+ レバーの受け金)
    fr = [_cyl("hinge_post", 0.008, h + 0.01, (0, 0, (h + 0.01) / 2, 0, 0, 0), "0.5 0.5 0.55 1", collide=False)]   # 蝶番の見た目だけ。衝突を付けると板と食い込んで勝手に回る (実測)
    if is_lever:
        sz = g["spindle_z"]
        # 受け金 = U 字スロット: アーム先端 (y=w+0.015) を前後 (ロボット側/ドア側) の 2 枚の板で挟み、押しても引いても
        # 当たるようにする。レバーを回すと先端がスロットの下へ抜ける。支柱は先端より奥 (y>w+0.035) に立てて干渉させない。
        front_x = g["lever_x"] - g["lever_r"] - 0.006
        back_x = g["lever_x"] + g["lever_r"] + 0.006
        for nm, px in (("keeper_front", front_x), ("keeper_back", back_x)):   # 下端 sz+0.002: アーム上端 (sz+r) と重なる
            fr.append(_box(nm, (0.006, 0.025, 0.030), (px, w + 0.0275, sz + 0.017, 0, 0, 0), "0.5 0.5 0.55 1"))
        cx, span = (front_x + back_x) / 2, abs(front_x - back_x) + 0.03
        fr.append(_box("keeper_post", (span, 0.010, sz + 0.032), (cx, w + 0.045, (sz + 0.032) / 2, 0, 0, 0), "0.5 0.5 0.55 1"))
    frame_sdf = f'<sdf version="1.8"><model name="door_frame"><static>true</static><link name="l">{"".join(fr)}</link></model></sdf>'
    return [("door_frame", frame_sdf, (hx, hy, 0, 0, 0, 0)), ("door", door_sdf, (hx, hy, 0, 0, 0, 0))]


def make_task_objects(task: dict[str, Any], scene: dict[str, Any]) -> list[tuple[str, str, tuple]]:
    kind = task["success"]["type"]
    if kind == "pick_place":
        return make_pick_place_objects(task, scene)
    if kind in ("open_door", "lever_door"):
        return make_door_objects(task, scene)
    raise NotImplementedError(f"タスク {task['task_id']} のシーン生成は未実装です")
