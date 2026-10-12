import { useEffect, useRef, useState } from "react";
import Viewport from "../components/Viewport";
import TeleopPanel from "../components/TeleopPanel";
import { Badge, CameraView, JointBars, MockWarning, TaskSelect, act } from "../components/common";
import { api, fmtSec, pct } from "../lib/api";
import { useStore } from "../lib/store";

export default function DataCollection() {
  const status = useStore((s) => s.status);
  const live = useStore((s) => s.live);
  const events = useStore((s) => s.events);
  const [task, setTask] = useState(status?.task_id ?? "pick_place");
  const [speed, setSpeed] = useState(1);
  const [autoStop, setAutoStop] = useState(true);
  const [seed, setSeed] = useState("");
  const [dragMode, setDragMode] = useState<"camera" | "ee">("camera");
  const [eps, setEps] = useState<any>({ items: [], stats: {} });
  useEffect(() => { if (status?.task_id) setTask(status.task_id); }, [status?.task_id]);   // 再読込後も実際のタスクに合わせる
  const busy = useRef(false);
  const recording = status?.mode === "recording";
  const taskLoaded = status?.task_id === task;

  const load = () => api(`/api/episodes?task_id=${task}`).then(setEps).catch(() => {});
  useEffect(() => { load(); }, [task, events.length]);
  const once = async (fn: () => Promise<any>, ok?: string) => {
    if (busy.current) return; busy.current = true;
    try { await act(fn, ok); } finally { busy.current = false; useStore.getState().refresh(); load(); }
  };
  const label = (id: string, body: any) => act(() => api(`/api/episodes/${task}/${id}`, { method: "PATCH", body })).then(load);
  const del = (id: string) => confirm("このエピソードを削除します (元に戻せません)") && act(() => api(`/api/episodes/${task}/${id}`, { method: "DELETE" })).then(load);

  return (
    <div>
      <h2>Data Collection</h2>
      <MockWarning />
      {recording && <div className="banner rec">● 収録中 — {live?.recording?.frames ?? 0} フレーム / {fmtSec(live?.recording?.seconds)} (ID {live?.recording?.episode_id})</div>}
      <div className="topbar">
        <TaskSelect value={task} onChange={setTask} onlyImplemented />
        <button disabled={recording} onClick={() => once(() => api("/api/sim/task", { body: { task_id: task } }), "タスクを読み込みました")}>{taskLoaded ? "再読み込み" : "読み込み"}</button>
        <button disabled={recording || !taskLoaded} onClick={() => once(() => api("/api/sim/reset", { body: {} }), "リセットしました")}>次のエピソード (リセット)</button>
        <span className="grow" />
        <button className={dragMode === "ee" ? "primary" : ""} onClick={() => setDragMode(dragMode === "ee" ? "camera" : "ee")}>ドラッグ: {dragMode === "ee" ? "EE" : "視点"}</button>
      </div>
      <div className="split">
        <div>
          <div className="vp-tall"><Viewport dragMode={dragMode} /></div>
        </div>
        <div>
          <div className="card">
            {!recording ? (
              <button className="recbtn start" disabled={!taskLoaded || status?.mode !== "idle"} onClick={() => once(() => api("/api/episodes/start", { body: { auto_stop: autoStop, seed: seed ? +seed : null } }))}>● 収録開始</button>
            ) : (
              <>
                <button className="recbtn stop" onClick={() => once(() => api("/api/episodes/stop", { body: {} }))}>■ 停止して保存</button>
                <button className="danger" style={{ width: "100%", marginTop: 8 }} onClick={() => once(() => api("/api/episodes/stop", { body: { discard: true } }), "破棄 (中断として記録)")}>破棄</button>
              </>
            )}
            {!taskLoaded && <div className="muted">先にタスクを読み込んでください</div>}
            <div className="row"><label><input type="checkbox" checked={autoStop} onChange={(e) => setAutoStop(e.target.checked)} /> 成功したら自動停止</label></div>
            <div className="row"><b>シード</b><input type="number" placeholder="自動" value={seed} min={0} max={9999} onChange={(e) => setSeed(e.target.value)} style={{ width: 100 }} /><span className="muted">0–9999 (評価用 10000+ は使えません)</span></div>
            {live?.task_state && <div className="kv"><div><span>成否 (物理判定)</span><b>{live.task_state.success ? "成功" : live.task_state.detail || "-"}</b></div>
              {Object.entries(live.task_state.metrics ?? {}).map(([k, v]: any) => <div key={k}><span>{k}</span><b className="mono">{(+v).toFixed(3)}</b></div>)}</div>}
          </div>
          <CameraView />
          <div className="card" style={{ marginTop: 12 }}><h3>関節状態</h3><JointBars /></div>
          <div className="card"><h3>操作 (PC / タッチ)</h3><TeleopPanel speed={speed} setSpeed={setSpeed} /></div>
        </div>
      </div>
          <div className="card">
            <h3>エピソード一覧 — {eps.stats.total ?? 0} 本 / 成功率 {pct(eps.stats.success_rate)} / 総収録 {fmtSec(eps.stats.recorded_seconds)}</h3>
            <table>
              <thead><tr><th>ID</th><th>結果</th><th>評価</th><th>長さ</th><th>品質</th><th>種別</th><th /></tr></thead>
              <tbody>
                {[...eps.items].reverse().slice(0, 30).map((m: any) => (
                  <tr key={m.episode_id} className={m.excluded ? "excl" : ""}>
                    <td className="mono">{m.episode_id}<div className="muted">seed {m.seed}</div></td>
                    <td><Badge kind={m.status === "success" ? "ok" : m.status === "failure" ? "warn" : "err"}>{m.status}</Badge></td>
                    <td>{m.label ?? "-"}</td>
                    <td>{m.num_frames ?? 0}f</td>
                    <td>{m.quality?.ok === false ? <Badge kind="err">NG</Badge> : <Badge kind="ok">OK</Badge>}{m.quality?.issues?.slice(0, 2).map((i: any) => <div key={i.code} className="muted">{i.message}</div>)}</td>
                    <td><span className="tag">{m.collection_mode ?? "?"}</span></td>
                    <td style={{ whiteSpace: "nowrap" }}>
                      <button className="ghost" onClick={() => label(m.episode_id, { label: "success" })}>成功</button>
                      <button className="ghost" onClick={() => label(m.episode_id, { label: "failure" })}>失敗</button>
                      <button className="ghost" onClick={() => label(m.episode_id, { excluded: !m.excluded })}>{m.excluded ? "戻す" : "除外"}</button>
                      <button className="ghost" onClick={() => del(m.episode_id)}>削除</button>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
            <div className="muted">失敗エピソードは自動削除しません。内容を確認して「除外」するか削除してください (除外されたものは学習データに入りません)。</div>
          </div>
    </div>
  );
}
