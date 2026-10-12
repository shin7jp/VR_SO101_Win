import { useEffect, useRef, useState } from "react";
import Viewport from "../components/Viewport";
import TeleopPanel from "../components/TeleopPanel";
import { CameraView, JointBars, SimStatusPanel, TaskSelect, MockWarning, Collapsible, act } from "../components/common";
import { api } from "../lib/api";
import { useStore } from "../lib/store";

export default function Simulation() {
  const status = useStore((s) => s.status);
  const tasks = useStore((s) => s.tasks);
  const [task, setTask] = useState(status?.task_id ?? "pick_place");
  const [speed, setSpeed] = useState(1);
  const [dragMode, setDragMode] = useState<"camera" | "ee">("camera");
  useEffect(() => { if (status?.task_id) setTask(status.task_id); }, [status?.task_id]);   // 再読込後も実際のタスクに合わせる
  const busy = useRef(false);
  // 連打しても 1 回だけ実行 (バックエンド側も冪等だが、画面側でも二重送信を防ぐ)
  const run = async (fn: () => Promise<any>, msg: string) => {
    if (busy.current) return;
    busy.current = true;
    try { await act(fn, msg); } finally { busy.current = false; useStore.getState().refresh(); }
  };
  const impl = tasks.find((t) => t.task_id === task)?.implemented;
  return (
    <div>
      <h2>Simulation</h2>
      <MockWarning />
      <div className="topbar">
        <TaskSelect value={task} onChange={setTask} onlyImplemented />
        <button className="primary" disabled={!impl} onClick={() => run(() => api("/api/sim/task", { body: { task_id: task } }), "タスクを読み込みました")}>読み込み / 初期化</button>
        <button disabled={!status?.task_id || status.mode === "recording"} onClick={() => run(() => api("/api/sim/reset", { body: {} }), "リセットしました")}>リセット (新しい配置)</button>
        <button onClick={() => run(() => api("/api/sim/connect", { method: "POST" }), "シムに接続しました")}>シム再接続</button>
        <span className="grow" />
        <button className={dragMode === "ee" ? "primary" : ""} onClick={() => setDragMode(dragMode === "ee" ? "camera" : "ee")}>ドラッグ: {dragMode === "ee" ? "EE を動かす" : "視点操作"}</button>
      </div>
      <div className="split">
        <div className="vp-col vp-tall"><Viewport dragMode={dragMode} /></div>
        <div>
          <CameraView />
          <div className="card" style={{ marginTop: 12 }}><h3>関節状態</h3><JointBars /></div>
          <div className="card"><h3>状態</h3><SimStatusPanel /></div>
          <div className="card"><Collapsible title="PC / タッチ操作"><TeleopPanel speed={speed} setSpeed={setSpeed} /></Collapsible></div>
        </div>
      </div>
    </div>
  );
}
