// Three.js ビューポート: SO-101 (URDF) + 作業台 + 物体 + WebXR コントローラ入力。
// 状態の正本は Gazebo: ここで関節を独自に進めず、WS で届く q (計測値) を表示するだけ。
import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import URDFLoader from "urdf-loader";
import { useStore } from "../lib/store";
import { sendWs } from "../lib/ws";
import { inputState } from "../lib/input";
import { api } from "../lib/api";

export type ViewportApi = { enterVr: () => Promise<void>; vrActive: boolean };
type Props = {
  height?: number | string;
  dragMode: "camera" | "ee";           // EE: 左ドラッグ/1 本指で EE を動かす
  onVrState?: (s: { active: boolean; following: boolean; paused: boolean; connected: boolean }) => void;
  apiRef?: React.MutableRefObject<ViewportApi | null>;
  vrScale?: number;
};

const ORANGE = new THREE.Color(0xf0a030);

// ドアの 3D 表示 (寸法は sim/bridge/worldgen.py / tasks/door_geometry.py と同じ式)。物理の正本は Gazebo で、ここは表示のみ。
function buildDoor(task: any) {
  const d = task.scene.door, w = d.width, h = d.height, t = d.thickness, z0 = 0.004;
  const group = new THREE.Group();
  const mat = (c: number) => new THREE.MeshStandardMaterial({ color: c, roughness: 0.7 });
  const cyl = (r: number, len: number, c: number) => new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 16), mat(c));
  const post = cyl(0.008, h + 0.01, 0x808090);
  post.rotation.x = Math.PI / 2; post.position.set(0, 0, (h + 0.01) / 2);
  group.add(post);
  // door モデル (板・ペグ・レバー)。ドアが開くとモデル全体がヒンジ軸まわりに回る (Gazebo の "door" の姿勢)。
  // 固定のもの (ヒンジ柱・受け金) は group 直下に置いて回さない。
  const model = new THREE.Group();
  group.add(model);
  const pivot = new THREE.Group();     // 「panel」リンクのモデル内での相対姿勢 (通常は単位)
  model.add(pivot);
  const panel = new THREE.Mesh(new THREE.BoxGeometry(t, w, h), mat(0xc0a680));
  panel.position.set(0, w / 2, z0 + h / 2);
  pivot.add(panel);
  let lever: THREE.Group | null = null;
  const hd = task.scene.handle;
  if (hd) {
    const hz = z0 + hd.height, hy = w - hd.edge_offset;
    const peg = cyl(hd.peg_radius, hd.peg_length, 0x333340);       // 円柱は y 軸向きなので z 回りに 90° して x 軸向きに
    peg.rotation.z = Math.PI / 2; peg.position.set(-t / 2 - hd.peg_length / 2, hy, hz);
    const head = cyl(hd.head_radius, hd.head_thickness, 0x222230);
    head.rotation.z = Math.PI / 2; head.position.set(-t / 2 - hd.peg_length - hd.head_thickness / 2, hy, hz);
    pivot.add(peg, head);
  }
  const lv = task.scene.lever;
  if (lv) {
    const sy = w - lv.edge_offset, sz = z0 + lv.axis_height, lx = -t / 2 - (lv.standoff ?? 0.014);
    lever = new THREE.Group();
    lever.position.set(-t / 2, sy, sz);
    const arm = cyl(lv.radius, lv.length, 0x333340);
    arm.position.set(lx + t / 2, lv.length / 2, 0);
    lever.add(arm);
    // ハブ: 板の面からレバー軸までをつなぐ円柱 (Gazebo の SDF の hub と同じ。x 軸向き)
    const hub = cyl(0.009, -lx - t / 2, 0x333340);
    hub.rotation.z = Math.PI / 2; hub.position.set((lx + t / 2) / 2, 0, 0);
    lever.add(hub);
    model.add(lever);   // レバーはドアと一緒に回る
    // 受け金 (U 字スロット)
    const fx = lx - lv.radius - 0.006, bx = lx + lv.radius + 0.006;
    for (const px of [fx, bx]) {
      const m = new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.025, 0.03), mat(0x808090));
      m.position.set(px, w + 0.0275, sz + 0.017);
      group.add(m);
    }
    const span = Math.abs(fx - bx) + 0.03;
    const kp = new THREE.Mesh(new THREE.BoxGeometry(span, 0.01, sz + 0.032), mat(0x808090));
    kp.position.set((fx + bx) / 2, w + 0.045, (sz + 0.032) / 2);
    group.add(kp);
  }
  return { group, model, pivot, lever };
}

export default function Viewport({ height = "100%", dragMode, onVrState, apiRef }: Props) {
  const mount = useRef<HTMLDivElement>(null);
  const dragRef = useRef(dragMode);
  dragRef.current = dragMode;
  const [loadErr, setLoadErr] = useState<string | null>(null);

  useEffect(() => {
    const el = mount.current!;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x10151b);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.xr.enabled = true;
    el.appendChild(renderer.domElement);
    renderer.domElement.style.touchAction = "none";

    const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 20);
    camera.up.set(0, 0, 1);                       // ロボット座標 (z 上) のまま表示する
    camera.position.set(-0.35, -0.55, 0.38);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0.2, 0, 0.08);
    controls.enableDamping = true;
    controls.touches = { ONE: THREE.TOUCH.ROTATE, TWO: THREE.TOUCH.DOLLY_PAN };

    // XR 用リグ: WebXR は y 上・-z 前。ロボット(z 上, +x 前)を「ユーザーから見て奥へ伸びる」向きに置く。
    const rig = new THREE.Group();
    scene.add(rig);
    const world = new THREE.Group();   // ロボット座標系
    rig.add(world);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 1.1));
    const sun = new THREE.DirectionalLight(0xffffff, 1.4);
    sun.position.set(0.5, -0.4, 1.2);
    scene.add(sun);

    const table = new THREE.Mesh(new THREE.BoxGeometry(1.2, 1.2, 0.04), new THREE.MeshStandardMaterial({ color: 0x8c8073, roughness: 0.9 }));
    table.position.set(0.2, 0, -0.02);
    world.add(table);
    world.add(new THREE.GridHelper(1.0, 20, 0x445566, 0x2a3440).rotateX(Math.PI / 2));
    world.children[world.children.length - 1].position.set(0.2, 0, 0.001);

    // ---- ロボット (URDF) ----
    let robot: any = null;
    const mgr = new THREE.LoadingManager();
    const loader = new URDFLoader(mgr);
    loader.loadMeshCb = (path: string, manager: THREE.LoadingManager, done: (o: THREE.Object3D, e?: Error) => void) => {
      // STL をオレンジ色の Phong で読む (urdf-loader 既定は無色グレー)
      new STLLoader(manager).load(path, (g) => {
        g.computeVertexNormals();
        done(new THREE.Mesh(g, new THREE.MeshPhongMaterial({ color: ORANGE, shininess: 30 })));
      }, undefined, (e) => done(new THREE.Object3D(), e as any));
    };
    loader.load("/api/robot/urdf/so101_new_calib.urdf", (r: any) => { robot = r; world.add(r); },
      undefined, (e: any) => setLoadErr("ロボットモデルの読み込みに失敗: " + (e?.message ?? e)));

    // ---- 物体 (状態から生成) ----
    const objects = new Map<string, THREE.Object3D>();
    let doorObj: ReturnType<typeof buildDoor> | null = null;
    const target = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 0.002), new THREE.MeshStandardMaterial({ color: 0x20c040 }));
    target.visible = false;
    world.add(target);
    const eeMarker = new THREE.Mesh(new THREE.SphereGeometry(0.006), new THREE.MeshBasicMaterial({ color: 0x40e0ff }));
    world.add(eeMarker);

    const ensureObject = (name: string, task: any): THREE.Object3D => {
      let o = objects.get(name);
      if (o) return o;
      if (name === "cube") {
        const s = task?.scene?.cube?.size ?? 0.03;
        o = new THREE.Mesh(new THREE.BoxGeometry(s, s, s), new THREE.MeshStandardMaterial({ color: 0xd03030 }));
      } else {
        o = new THREE.Mesh(new THREE.BoxGeometry(0.03, 0.03, 0.03), new THREE.MeshStandardMaterial({ color: 0x8080ff }));
      }
      world.add(o);
      objects.set(name, o);
      return o;
    };

    // ---- VR 用 HUD (録画状態・通信状態・デッドマン): 没入中は画面の UI が見えないので 3D 空間に出す ----
    const hudCanvas = document.createElement("canvas");
    hudCanvas.width = 768; hudCanvas.height = 192;
    const hudTex = new THREE.CanvasTexture(hudCanvas);
    const hud = new THREE.Mesh(new THREE.PlaneGeometry(0.6, 0.15), new THREE.MeshBasicMaterial({ map: hudTex, transparent: true, depthTest: false }));
    hud.position.set(0, 1.55, -0.9);
    hud.renderOrder = 999;
    hud.visible = false;
    scene.add(hud);
    let lastHud = 0;
    const drawHud = () => {
      const st = useStore.getState(), live = st.live;
      const g = hudCanvas.getContext("2d")!;
      g.clearRect(0, 0, hudCanvas.width, hudCanvas.height);
      g.fillStyle = "rgba(10,14,20,0.82)"; g.roundRect(0, 0, hudCanvas.width, hudCanvas.height, 24); g.fill();
      g.font = "bold 44px sans-serif";
      const rec = live?.recording;
      g.fillStyle = rec ? "#ff4a40" : "#9fe0a8";
      g.fillText(rec ? `● REC ${rec.frames}f ${rec.seconds}s` : (live?.mode === "idle" ? "待機" : String(live?.mode ?? "未接続")), 24, 62);
      g.font = "34px sans-serif";
      g.fillStyle = following ? "#ffd060" : "#889";
      g.fillText(following ? "追従中 (グリップで一時停止)" : paused ? "一時停止中 (離すと追従)" : "コントローラ待ち", 24, 112);
      g.fillStyle = st.wsState === "open" ? "#9fe0a8" : "#ff7a70";
      g.fillText(`通信: ${st.wsState === "open" ? "OK" : "切断"}  teleop: ${live?.teleop ?? "-"}  ${live?.task_state?.success ? "✔成功" : ""}`, 24, 160);
      hudTex.needsUpdate = true;
    };

    // ---- 描画ループ ----
    const qSmooth: number[] = [];
    let lastTaskId: string | null = null;
    const resize = () => {
      const w = el.clientWidth || 300, h = el.clientHeight || 300;
      renderer.setSize(w, h, false);   // CSS で 100% にする (インライン固定幅だとグリッドが縮まない)
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
    };
    resize();
    const ro = new ResizeObserver(resize);
    ro.observe(el);

    // ---- EE ドラッグ (PC マウス / タッチ 1 本指): マウスに EE が追従する ----
    // 押した瞬間の EE 位置とカーソル下の点の差を保ち (EE が飛ばない)、カーソルの動きをそのまま EE 目標位置の
    // 変位にして送る。通常は EE の高さの水平面上、Shift 中はカメラ正面向きの鉛直面上で動かす (上下だけ変わる)。
    const drag = { on: false, id: -1, vertical: false, ee0: new THREE.Vector3(), hit0: new THREE.Vector3(), target: new THREE.Vector3() };
    const ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
    const hitPlane = (e: PointerEvent, vertical: boolean, out: THREE.Vector3) => {
      const r = renderer.domElement.getBoundingClientRect();
      ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
      ray.setFromCamera(ndc, camera);
      const n = new THREE.Vector3(0, 0, 1);
      if (vertical) { camera.getWorldDirection(n); n.z = 0; if (n.lengthSq() < 1e-6) return false; n.normalize(); }
      const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(n, drag.ee0);
      return ray.ray.intersectPlane(plane, out) !== null;
    };
    const anchor = (e: PointerEvent, vertical: boolean) => {
      const ee = useStore.getState().live?.ee;
      if (!ee) return false;
      drag.ee0.set(ee[0], ee[1], ee[2]);
      drag.vertical = vertical;
      if (!hitPlane(e, vertical, drag.hit0)) return false;
      drag.target.copy(drag.ee0);
      return true;
    };
    const sendDrag = () => {
      if (!drag.on) return;
      sendWs({ type: "ee", pos: [drag.target.x, drag.target.y, drag.target.z] });
    };
    const onDown = (e: PointerEvent) => {
      if (dragRef.current !== "ee" || renderer.xr.isPresenting) return;
      if (e.pointerType === "mouse" && e.button !== 0) return;
      if (drag.on) return;   // 2 本目以降の指は無視 (最初の 1 本だけが EE を動かす)
      if (!anchor(e, e.shiftKey || inputState.vertical)) return;
      inputState.dragging = true;
      drag.on = true; drag.id = e.pointerId;
      renderer.domElement.setPointerCapture(e.pointerId);
    };
    const onMove = (e: PointerEvent) => {
      if (!drag.on || e.pointerId !== drag.id) return;
      const vert = e.shiftKey || inputState.vertical;   // iPad/iPhone には Shift が無いので、画面の「上下」切替でも同じ
      if (vert !== drag.vertical && !anchor(e, vert)) return;   // Shift の切替で基準を取り直す (飛ばない)
      const hit = new THREE.Vector3();
      if (!hitPlane(e, drag.vertical, hit)) return;
      const d = hit.sub(drag.hit0);
      if (drag.vertical) drag.target.set(drag.ee0.x, drag.ee0.y, drag.ee0.z + d.z);
      else drag.target.set(drag.ee0.x + d.x, drag.ee0.y + d.y, drag.ee0.z);
    };
    const onUp = (e: PointerEvent) => { if (!drag.on || e.pointerId !== drag.id) return; inputState.dragging = false; drag.on = false; sendWs({ type: "release" }); };
    renderer.domElement.addEventListener("pointerdown", onDown);
    renderer.domElement.addEventListener("pointermove", onMove);
    renderer.domElement.addEventListener("pointerup", onUp);
    renderer.domElement.addEventListener("pointercancel", onUp);
    const dragTimer = window.setInterval(sendDrag, 40);

    // ---- WebXR ----
    let vrActive = false, following = false, paused = false, vrFrames = 0, calibrating = false;
    const ctrl = [renderer.xr.getController(0), renderer.xr.getController(1)];
    ctrl.forEach((c) => scene.add(c));
    const reportVr = () => onVrState?.({ active: vrActive, following, paused, connected: vrActive });
    const quat = new THREE.Quaternion(), pos = new THREE.Vector3(), fwdV = new THREE.Vector3();
    let lastVrSend = 0;

    const vrFrame = () => {
      const session = renderer.xr.getSession();
      if (!session) return;
      // 右コントローラを操作に使う (無ければ左)
      let idx = -1;
      const srcs = Array.from(session.inputSources);
      srcs.forEach((s, i) => { if (s.handedness === "right" && s.gamepad) idx = i; });
      if (idx < 0) srcs.forEach((s, i) => { if (s.gamepad && idx < 0) idx = i; });
      if (idx < 0) return;
      const gp = srcs[idx].gamepad!;
      const c = ctrl[Math.min(idx, 1)];
      const trigger = gp.buttons[0]?.value ?? 0;       // トリガー: グリッパー
      const grip = gp.buttons[1]?.pressed ?? false;     // グリップ: 一時停止 (握っている間は追従しない。離した位置で基準を取り直す)
      const aBtn = gp.buttons[4]?.pressed ?? false;     // A/X: キャリブレーション
      c.matrixWorld.decompose(pos, quat, new THREE.Vector3());
      // コントローラの動きに常時追従する。ただし (1) 没入直後は追跡が安定するまで待つ、(2) 未追跡のコントローラは
      // 原点 (0,0,0) にいるので追従しない、(3) グリップを握っている間は一時停止 (持ち替え用)。
      vrFrames++;
      const tracked = pos.lengthSq() > 1e-8;
      paused = grip;
      following = tracked && vrFrames > 20 && !grip;
      const now = performance.now();
      if (now - lastVrSend > 16) {
        lastVrSend = now;
        sendWs({ type: "vr", pos: [pos.x, pos.y, pos.z], quat: [quat.x, quat.y, quat.z, quat.w], deadman: following, gripper: 1 - trigger });
      }
      if (aBtn && !calibrating) {
        calibrating = true;
        fwdV.set(0, 0, -1).applyQuaternion(quat);       // コントローラが向いている方向 = ロボットの前
        api("/api/teleop/calibrate", { body: { forward: [fwdV.x, fwdV.y, fwdV.z] } }).catch(() => {});
        setTimeout(() => (calibrating = false), 1000);
      }
      reportVr();
    };

    const applyRigForXr = (on: boolean) => {
      if (on) {   // ロボット(+x 前, z 上)を XR(-z 前, y 上) へ: Rx(-90°) の後 Ry(+90°)。机の高さ 0.75m、ユーザーの 0.55m 前方
        rig.rotation.set(0, 0, 0);
        rig.quaternion.setFromEuler(new THREE.Euler(0, Math.PI / 2, 0)).multiply(new THREE.Quaternion().setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0)));
        rig.position.set(0, 0.75, -0.55);
      } else { rig.quaternion.identity(); rig.position.set(0, 0, 0); }
    };

    const enterVr = async () => {
      const xr = (navigator as any).xr;
      const session = await xr.requestSession("immersive-vr", { optionalFeatures: ["local-floor", "bounded-floor"] });
      session.addEventListener("end", () => { vrActive = false; following = false; paused = false; applyRigForXr(false); sendWs({ type: "release" }); reportVr(); });
      await renderer.xr.setSession(session);
      vrFrames = 0; vrActive = true; applyRigForXr(true); reportVr();
    };
    if (apiRef) apiRef.current = { enterVr, get vrActive() { return vrActive; } } as ViewportApi;

    renderer.setAnimationLoop(() => {
      const st = useStore.getState();
      const live = st.live;
      const taskKey = `${live?.task_id ?? ""}:${st.tasks.length}`;   // tasks が後から届いてもシーンを作り直す
      if (taskKey !== lastTaskId) {
        lastTaskId = taskKey;
        objects.forEach((o) => world.remove(o)); objects.clear();
        const t = st.tasks.find((x) => x.task_id === live?.task_id) as any;
        if (doorObj) { world.remove(doorObj.group); doorObj = null; }
        if (t?.scene?.door) { doorObj = buildDoor(t); world.add(doorObj.group); }
        const tg = t?.scene?.target;
        target.visible = !!tg;
        if (tg) { target.scale.set(tg.half_extent * 2, tg.half_extent * 2, 1); target.position.set(tg.center[0], tg.center[1], 0.001); }
      }
      if (robot && live?.q && st.robot) {
        st.robot.joint_names.forEach((n: string, i: number) => {
          const cur = qSmooth[i] ?? live.q![i];
          qSmooth[i] = cur + (live.q![i] - cur) * 0.35;      // 10Hz の状態を滑らかに (表示だけ。実状態は進めない)
          robot.setJointValue(n, qSmooth[i]);
        });
      }
      if (live?.objects) {
        const t = st.tasks.find((x) => x.task_id === live.task_id);
        for (const [name, o] of Object.entries(live.objects)) {
          if (doorObj) {
            // "door" = モデルの姿勢 (位置=ヒンジ、向き=ドアの開き)。panel / lever_link は「モデル座標での相対姿勢」
            if (name === "door") { doorObj.group.position.set(o.pos[0], o.pos[1], o.pos[2]); doorObj.model.quaternion.set(o.quat[0], o.quat[1], o.quat[2], o.quat[3]); }
            else if (name === "panel") { doorObj.pivot.position.set(o.pos[0], o.pos[1], o.pos[2]); doorObj.pivot.quaternion.set(o.quat[0], o.quat[1], o.quat[2], o.quat[3]); }
            else if (name === "lever_link" && doorObj.lever) {   // レバーのモデル内の位置と回転 (レバーを下げると回る)
              doorObj.lever.position.set(o.pos[0], o.pos[1], o.pos[2]);
              doorObj.lever.quaternion.set(o.quat[0], o.quat[1], o.quat[2], o.quat[3]);
            }
            continue;
          }
          const m = ensureObject(name, t);
          m.position.set(o.pos[0], o.pos[1], o.pos[2]);
          m.quaternion.set(o.quat[0], o.quat[1], o.quat[2], o.quat[3]);
        }
      }
      if (live?.ee) eeMarker.position.set(live.ee[0], live.ee[1], live.ee[2]);
      hud.visible = renderer.xr.isPresenting;
      if (renderer.xr.isPresenting) {
        vrFrame();
        const nowMs = performance.now();
        if (nowMs - lastHud > 200) { lastHud = nowMs; drawHud(); }
      } else controls.update();
      renderer.render(scene, camera);
    });

    const setControlsForDrag = () => {
      controls.mouseButtons = dragRef.current === "ee"
        ? { LEFT: -1 as any, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.ROTATE }
        : { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.PAN };
      controls.touches = dragRef.current === "ee" ? { ONE: -1 as any, TWO: THREE.TOUCH.DOLLY_ROTATE } : { ONE: THREE.TOUCH.ROTATE, TWO: THREE.TOUCH.DOLLY_PAN };
    };
    setControlsForDrag();
    const modeTimer = window.setInterval(setControlsForDrag, 300);

    return () => {
      window.clearInterval(dragTimer); window.clearInterval(modeTimer);
      renderer.setAnimationLoop(null); ro.disconnect();
      renderer.domElement.removeEventListener("pointerdown", onDown);
      renderer.domElement.removeEventListener("pointermove", onMove);
      renderer.domElement.removeEventListener("pointerup", onUp);
      renderer.domElement.removeEventListener("pointercancel", onUp);
      controls.dispose(); renderer.dispose(); el.removeChild(renderer.domElement);
      if (apiRef) apiRef.current = null;
    };
  }, []);

  return (
    <div className="viewport" style={{ height }}>
      <div ref={mount} style={{ width: "100%", height: "100%" }} />
      {loadErr && <div className="vp-err">{loadErr}</div>}
    </div>
  );
}
