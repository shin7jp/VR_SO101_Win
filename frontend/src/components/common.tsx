import { useEffect, useMemo, useRef, useState } from "react";
import { useStore, act } from "../lib/store";
import { api, fmtSec } from "../lib/api";

export function Badge({ kind, children }: { kind?: "ok" | "err" | "warn" | "rec" | "info" | "idle"; children: React.ReactNode }) {
  return <span className={"badge " + (kind ?? "idle")}>{children}</span>;
}

export function ModeBadge() {
  const mode = useStore((s) => s.status?.mode ?? s.live?.mode ?? "disconnected");
  const map: Record<string, [string, any]> = {
    recording: ["● 収録中", "rec"], idle: ["待機", "ok"], inference: ["推論中", "info"], error: ["異常", "err"], disconnected: ["シム未接続", "warn"],
  };
  const [t, k] = map[mode] ?? [mode, "idle"];
  return <Badge kind={k}>{t}</Badge>;
}

export function MockWarning() {
  const mock = useStore((s) => s.live?.is_mock ?? s.status?.is_mock);
  if (!mock) return null;
  return <div className="banner warn">⚠ モックシム: Gazebo ではなく疑似シミュレーションです。ここで得たデータ・結果は Gazebo の検証結果ではありません。</div>;
}

// Gazebo カメラ (MJPEG)。学習・推論に使う画像と同じ経路の映像。
export function CameraView({ src = "/api/sim/camera.mjpg", label }: { src?: string; label?: string }) {
  const mock = useStore((s) => s.live?.is_mock ?? s.status?.is_mock);
  label = label ?? (mock ? "モック映像 (Gazebo ではありません)" : "Gazebo カメラ (front)");
  const [err, setErr] = useState(false);
  const [nonce, setNonce] = useState(0);
  return (
    <div className="camera">
      <div className="cam-label">{label}</div>
      {err ? <div className="cam-err">映像なし。シム接続・タスク選択を確認してください <button onClick={() => { setErr(false); setNonce((n) => n + 1); }}>再読込</button></div>
        : <img src={`${src}?n=${nonce}`} alt="camera" onError={() => setErr(true)} />}
    </div>
  );
}

export function JointBars() {
  const robot = useStore((s) => s.robot);
  const q = useStore((s) => s.live?.q);
  if (!robot || !q) return <div className="muted">関節状態なし</div>;
  return (
    <div className="joints">
      {robot.joint_names.map((n: string, i: number) => {
        const lo = robot.lower[i], hi = robot.upper[i], v = q[i];
        const p = Math.max(0, Math.min(1, (v - lo) / (hi - lo)));
        const near = p < 0.04 || p > 0.96;
        return (
          <div className="jrow" key={n}>
            <span className="jn">{n}</span>
            <div className="jbar"><div className={"jfill" + (near ? " warn" : "")} style={{ width: `${p * 100}%` }} /></div>
            <span className="mono jv">{v.toFixed(2)}</span>
          </div>
        );
      })}
      <div className="muted">単位: ラジアン (グリッパー含む)</div>
    </div>
  );
}

export function SimStatusPanel() {
  const st = useStore((s) => s.status);
  const live = useStore((s) => s.live);
  const ws = useStore((s) => s.wsState);
  return (
    <div className="kv">
      <div><span>WebSocket</span><b>{ws === "open" ? "接続" : ws === "connecting" ? "接続中…" : "切断 (自動再接続)"}</b></div>
      <div><span>シム</span><b>{st?.backend ?? "-"} / {st?.connected ? "接続" : "未接続"}</b></div>
      <div><span>タスク</span><b>{st?.task_id ?? "-"}</b></div>
      <div><span>シム時刻</span><b className="mono">{live?.sim_time?.toFixed(2) ?? "-"} s</b></div>
      <div><span>テレオペ</span><b>{live?.teleop ?? "-"}</b></div>
      <div><span>制御周期</span><b className="mono">{live?.tick_ms ?? "-"} ms{live && live.overruns > 0 ? ` (遅延 ${live.overruns})` : ""}</b></div>
      {st?.last_error && <div className="err-line">⚠ {st.last_error.code}: {st.last_error.message}</div>}
    </div>
  );
}

// 学習の損失曲線 (外部ライブラリなしの SVG)
export function LineChart({ series, height = 180, logY = true }: { series: { name: string; color: string; pts: [number, number][] }[]; height?: number; logY?: boolean }) {
  const W = 560, H = height, pad = 34;
  const all = series.flatMap((s) => s.pts);
  if (all.length < 2) return <div className="muted">データ待ち…</div>;
  const xs = all.map((p) => p[0]), ys = all.map((p) => (logY ? Math.log10(Math.max(p[1], 1e-6)) : p[1]));
  const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
  const sx = (x: number) => pad + ((x - x0) / Math.max(x1 - x0, 1e-9)) * (W - pad - 8);
  const sy = (y: number) => H - pad + 6 - (((logY ? Math.log10(Math.max(y, 1e-6)) : y) - y0) / Math.max(y1 - y0, 1e-9)) * (H - pad - 8);
  const tick = (v: number) => (logY ? Math.pow(10, v) : v);
  return (
    <svg viewBox={`0 0 ${W} ${H}`} className="chart" role="img">
      {[0, 0.5, 1].map((f) => { const v = y0 + (y1 - y0) * f; const y = H - pad + 6 - f * (H - pad - 8);
        return <g key={f}><line x1={pad} x2={W - 8} y1={y} y2={y} stroke="#2a3440" /><text x={2} y={y + 4} fill="#8a97a6" fontSize="10">{tick(v).toPrecision(2)}</text></g>; })}
      <text x={pad} y={H - 6} fill="#8a97a6" fontSize="10">step {x0}</text>
      <text x={W - 70} y={H - 6} fill="#8a97a6" fontSize="10">step {x1}</text>
      {series.map((s) => <polyline key={s.name} fill="none" stroke={s.color} strokeWidth="1.6" points={s.pts.map((p) => `${sx(p[0])},${sy(p[1])}`).join(" ")} />)}
      {series.map((s, i) => <text key={s.name} x={pad + 8 + i * 90} y={14} fill={s.color} fontSize="11">● {s.name}</text>)}
    </svg>
  );
}

export function useJobs(kind?: string, interval = 2000) {
  const [jobs, setJobs] = useState<any[]>([]);
  const tick = useStore((s) => s.jobsTick);
  const load = async () => { try { setJobs(await api(`/api/jobs${kind ? `?kind=${kind}` : ""}`)); } catch { /* 表示は据え置き */ } };
  useEffect(() => { load(); const t = window.setInterval(load, interval); return () => window.clearInterval(t); }, [kind, tick]);
  return { jobs, reload: load };
}

export function JobCard({ job, onStop }: { job: any; onStop?: () => void }) {
  const p = job.progress ?? {};
  const pc = p.total ? Math.min(100, (100 * (p.step ?? 0)) / p.total) : null;
  const kind = job.state === "succeeded" ? "ok" : job.state === "failed" ? "err" : job.state === "running" ? "info" : "warn";
  return (
    <div className="card job">
      <div className="row between"><b>{job.kind} · {job.id}</b><Badge kind={kind as any}>{job.state}</Badge></div>
      {pc != null && <div className="pbar"><div style={{ width: pc + "%" }} /></div>}
      <div className="muted">{p.step ?? 0}{p.total ? ` / ${p.total}` : ""}　{job.last_metric ? `loss ${job.last_metric.loss}` : ""}</div>
      {job.error && <div className="err-line">{job.error}</div>}
      {onStop && job.state === "running" && <button className="danger" onClick={onStop}>停止</button>}
    </div>
  );
}

export function Collapsible({ title, children, open: o = true }: { title: string; children: React.ReactNode; open?: boolean }) {
  const [open, setOpen] = useState(o);
  return (
    <div className="collapsible">
      <button className="ghost" onClick={() => setOpen(!open)}>{open ? "▾" : "▸"} {title}</button>
      {open && <div>{children}</div>}
    </div>
  );
}

export function Toasts() {
  const toasts = useStore((s) => s.toasts);
  const dismiss = useStore((s) => s.dismiss);
  return <div className="toasts">{toasts.map((t) => <div key={t.id} className={"toast " + t.kind} onClick={() => dismiss(t.id)}>{t.text}</div>)}</div>;
}

export function TaskSelect({ value, onChange, onlyImplemented = false }: { value: string; onChange: (v: string) => void; onlyImplemented?: boolean }) {
  const tasks = useStore((s) => s.tasks);
  return (
    <select value={value} onChange={(e) => onChange(e.target.value)}>
      {tasks.map((t) => <option key={t.task_id} value={t.task_id} disabled={onlyImplemented && !t.implemented}>{t.name}{t.implemented ? "" : " (シーン未実装)"}</option>)}
    </select>
  );
}

export { act, fmtSec };
