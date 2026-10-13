import { useEffect, useRef, useState } from "react";
import Viewport, { ViewportApi } from "../components/Viewport";
import TeleopPanel from "../components/TeleopPanel";
import { CameraView, JointBars, SimStatusPanel, Badge, MockWarning, act } from "../components/common";
import { api } from "../lib/api";
import { detectXr, deviceClass, XrSupport } from "../lib/xr";
import { sendWs } from "../lib/ws";
import { useStore } from "../lib/store";

export default function VrTeleop() {
  const [xr, setXr] = useState<XrSupport | null>(null);
  const [vr, setVr] = useState({ active: false, following: false, paused: false, connected: false });
  const [yawDeg, setYawDeg] = useState(0);
  const [scale, setScale] = useState(1);
  const [speed, setSpeed] = useState(1);
  const [dragMode, setDragMode] = useState<"camera" | "ee">("camera");
  const apiRef = useRef<ViewportApi | null>(null);
  const status = useStore((s) => s.status);
  const dev = deviceClass();
  useEffect(() => { detectXr().then(setXr); }, []);
  const calibrate = () => {
    const r = (yawDeg * Math.PI) / 180;   // 「ロボットの前」を指す方向 (WebXR: 右=+x, 前=-z)
    return act(() => api("/api/teleop/calibrate", { body: { forward: [Math.sin(r), 0, -Math.cos(r)], scale } }), "キャリブレーションを更新しました");
  };
  return (
    <div>
      <h2>VR Teleoperation</h2>
      <MockWarning />
      {!status?.task_id && <div className="banner warn">先に Simulation 画面でタスクを読み込んでください。</div>}
      <div className="topbar">
        <button className="primary" disabled={!xr?.supported || !status?.task_id} onClick={() => act(() => apiRef.current!.enterVr())}>VR 開始</button>
        {xr && !xr.supported && <span className="muted">⚠ {xr.reason}</span>}
        {xr?.supported && <Badge kind="ok">WebXR 対応</Badge>}
        {vr.active && <Badge kind={vr.following ? "rec" : "warn"}>{vr.following ? "追従中" : vr.paused ? "一時停止中 (グリップを離すと追従)" : "コントローラ待ち"}</Badge>}
        <span className="grow" />
        <Badge kind="info">端末: {dev === "phone" ? "iPhone" : dev === "tablet" ? "iPad/タブレット" : "PC"} — {xr?.supported ? "VR + PC 操作" : "PC/タッチ操作モード"}</Badge>
        <button className={dragMode === "ee" ? "primary" : ""} onClick={() => setDragMode(dragMode === "ee" ? "camera" : "ee")}>ドラッグ: {dragMode === "ee" ? "EE" : "視点"}</button>
      </div>
      <div className="split">
        <div className="vp-col vp-tall"><Viewport apiRef={apiRef} dragMode={dragMode} onVrState={setVr} /></div>
        <div>
          <div className="card"><h3>VR 操作 (右コントローラ)</h3>
            <div className="muted">手先がコントローラーの動きに常時追従 (没入した瞬間のポーズが基準なので飛ばない) / グリップ=一時停止 (握っている間は止まり、離した位置で基準を取り直す。持ち替え用) / トリガー=グリッパー / A・X ボタン=コントローラの向きを「ロボットの前」に再設定</div>
            <div className="row"><b>向き補正</b><input type="number" value={yawDeg} onChange={(e) => setYawDeg(+e.target.value)} style={{ width: 80 }} />°
              <b>移動スケール</b><input type="range" min={0.2} max={2} step={0.1} value={scale} onChange={(e) => setScale(+e.target.value)} /><span className="mono">{scale.toFixed(1)}×</span></div>
            <button onClick={calibrate}>適用</button>
            <div className="muted" style={{ marginTop: 6 }}>現在: yaw {(status?.frame?.yaw_offset ?? 0).toFixed(2)} rad / scale {status?.frame?.scale ?? 1}</div>
            <button className="danger" style={{ marginTop: 8 }} onClick={() => sendWs({ type: "release" })}>即時停止 (追従解除)</button>
          </div>
          {!vr.active && <div className="card"><h3>PC / タッチ操作</h3><TeleopPanel speed={speed} setSpeed={setSpeed} /></div>}
          <CameraView />
          <div className="card"><h3>関節状態</h3><JointBars /></div>
          <div className="card"><h3>通信状態</h3><SimStatusPanel /></div>
        </div>
      </div>
    </div>
  );
}
