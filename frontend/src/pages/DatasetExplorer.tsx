import { useEffect, useState } from "react";
import { Badge, LineChart, TaskSelect, act, useJobs, JobCard } from "../components/common";
import { api, fmtSec } from "../lib/api";
import { useStore } from "../lib/store";

export default function DatasetExplorer() {
  const robot = useStore((s) => s.robot);
  const [task, setTask] = useState("pick_place");
  const [eps, setEps] = useState<any>({ items: [], stats: {} });
  const [sel, setSel] = useState<any>(null);
  const [detail, setDetail] = useState<any>(null);
  const [joint, setJoint] = useState(0);
  const [filter, setFilter] = useState("all");
  const [datasets, setDatasets] = useState<any[]>([]);
  const [name, setName] = useState("v1");
  const [onlySuccess, setOnlySuccess] = useState(true);
  const [mode, setMode] = useState("");
  const [validation, setValidation] = useState<any>(null);
  const { jobs } = useJobs("export");

  const load = () => {
    api(`/api/episodes?task_id=${task}`).then(setEps).catch(() => {});
    api(`/api/datasets?task_id=${task}`).then(setDatasets).catch(() => {});
  };
  useEffect(() => { load(); setSel(null); }, [task]);
  useEffect(() => { if (jobs[0]?.state === "succeeded") load(); }, [jobs[0]?.state]);
  useEffect(() => { if (sel) api(`/api/episodes/${task}/${sel.episode_id}`).then(setDetail).catch(() => setDetail(null)); else setDetail(null); }, [sel?.episode_id]);

  const items = eps.items.filter((m: any) => filter === "all" || (filter === "excluded" ? m.excluded : (m.label ?? m.status) === filter));
  const patch = async (body: any) => { await act(() => api(`/api/episodes/${task}/${sel.episode_id}`, { method: "PATCH", body })); load(); };
  const traj = detail?.trajectory;
  const series = traj ? [
    { name: "観測 q", color: "#3aa0ff", pts: traj.t.map((t: number, i: number) => [t, traj.q[i][joint]] as [number, number]) },
    { name: "指令 action", color: "#f0a030", pts: traj.t.map((t: number, i: number) => [t, traj.q_cmd[i][joint]] as [number, number]) }] : [];
  const exportDs = () => act(() => api("/api/datasets/export", { body: { task_id: task, name, only_success: onlySuccess, collection_mode: mode || null } }), "変換ジョブを開始しました");
  const validate = async (n: string) => setValidation({ name: n, ...(await act(() => api(`/api/datasets/${task}/${n}/validate`)) ?? { ok: false, problems: ["検証に失敗"] }) });

  return (
    <div>
      <h2>Dataset Explorer</h2>
      <div className="topbar">
        <TaskSelect value={task} onChange={setTask} />
        <select value={filter} onChange={(e) => setFilter(e.target.value)}>
          <option value="all">すべて</option><option value="success">成功</option><option value="failure">失敗</option><option value="aborted">中断</option><option value="excluded">除外</option>
        </select>
        <span className="muted">{eps.stats.total ?? 0} 本 / {eps.stats.frames ?? 0} フレーム / {fmtSec(eps.stats.recorded_seconds)}</span>
      </div>
      <div className="grid2">
        <div className="card" style={{ maxHeight: 520, overflow: "auto" }}>
          <table>
            <thead><tr><th>ID / seed</th><th>結果</th><th>フレーム</th><th>種別</th></tr></thead>
            <tbody>
              {items.map((m: any) => (
                <tr key={m.episode_id} className={(sel?.episode_id === m.episode_id ? "sel " : "") + (m.excluded ? "excl" : "")} onClick={() => setSel(m)}>
                  <td className="mono">{m.episode_id}<div className="muted">seed {m.seed}{m.excluded ? " · 除外" : ""}</div></td>
                  <td><Badge kind={m.status === "success" ? "ok" : m.status === "failure" ? "warn" : "err"}>{m.status}</Badge> {m.label && <span className="tag">評価:{m.label}</span>}</td>
                  <td>{m.num_frames ?? 0}</td><td><span className="tag">{m.collection_mode ?? "?"}</span></td>
                </tr>
              ))}
            </tbody>
          </table>
          {items.length === 0 && <div className="muted">エピソードがありません</div>}
        </div>
        <div className="card">
          {!sel ? <div className="muted">左の一覧からエピソードを選んでください</div> : (
            <>
              <div className="row between"><b className="mono">{sel.episode_id}</b><Badge>{detail?.meta?.status ?? sel.status}</Badge></div>
              {sel.preview !== false && sel.status !== "recording" && <video key={sel.episode_id} src={`/api/episodes/${task}/${sel.episode_id}/video`} controls playsInline muted loop />}
              <div className="row">
                <button onClick={() => patch({ label: "success" })}>成功と評価</button>
                <button onClick={() => patch({ label: "failure" })}>失敗と評価</button>
                <button onClick={() => patch({ excluded: !detail?.meta?.excluded })}>{detail?.meta?.excluded ? "除外を解除" : "学習から除外"}</button>
                <button className="danger" onClick={async () => { if (confirm("削除します (元に戻せません)")) { await act(() => api(`/api/episodes/${task}/${sel.episode_id}`, { method: "DELETE" })); setSel(null); load(); } }}>削除</button>
              </div>
              {robot && traj && <>
                <div className="row"><b>関節軌跡</b><select value={joint} onChange={(e) => setJoint(+e.target.value)}>{robot.joint_names.map((n: string, i: number) => <option key={n} value={i}>{n}</option>)}</select></div>
                <LineChart series={series} logY={false} />
              </>}
              <h3>品質</h3>
              {detail?.meta?.quality ? <>
                <div className="kv"><div><span>判定</span><b>{detail.meta.quality.ok ? "OK" : "NG"}</b></div><div><span>欠落フレーム</span><b>{detail.meta.quality.dropped_frames ?? 0}</b></div>
                  <div><span>step 中央値/最大</span><b className="mono">{detail.meta.quality.latency?.step_ms_median?.toFixed(1)} / {detail.meta.quality.latency?.step_ms_max?.toFixed(1)} ms</b></div></div>
                {detail.meta.quality.issues.map((i: any) => <div key={i.code} className="err-line">[{i.level}] {i.message}</div>)}
              </> : <div className="muted">品質情報なし (中断/収録中)</div>}
              <details><summary className="muted">メタデータ</summary><pre className="mono" style={{ fontSize: 11, overflow: "auto" }}>{JSON.stringify(detail?.meta, null, 1)}</pre></details>
            </>
          )}
        </div>
      </div>
      <h3>LeRobot データセット</h3>
      <div className="grid2">
        <div className="card">
          <b>エクスポート (LeRobot 形式へ変換)</b>
          <div className="row"><input type="text" value={name} onChange={(e) => setName(e.target.value)} placeholder="データセット名" />
            <select value={mode} onChange={(e) => setMode(e.target.value)}><option value="">全種別</option><option value="human">human のみ</option><option value="scripted">scripted のみ</option></select></div>
          <div className="row"><label><input type="checkbox" checked={onlySuccess} onChange={(e) => setOnlySuccess(e.target.checked)} /> 成功のみ</label></div>
          <button className="primary" onClick={exportDs}>変換を開始</button>
          <div className="muted">除外・中断・品質 NG は自動的に対象外になります。</div>
          {jobs.filter((j) => j.state === "running" || j === jobs[0]).slice(0, 1).map((j) => <JobCard key={j.id} job={j} />)}
        </div>
        <div className="card">
          <table><thead><tr><th>名前</th><th>エピソード</th><th>フレーム</th><th>除外</th><th /></tr></thead>
            <tbody>{datasets.map((d) => (
              <tr key={d.name}><td>{d.name}<div className="muted">{d.created_at?.slice(0, 16)} · lerobot {d.lerobot_version}</div></td><td>{d.num_episodes}</td><td>{d.num_frames}</td>
                <td>{d.num_skipped}<div className="muted">{Object.entries(d.skip_reasons ?? {}).map(([k, v]) => `${k}:${v}`).join(" ")}</div></td>
                <td><button onClick={() => validate(d.name)}>検証</button></td></tr>))}</tbody></table>
          {datasets.length === 0 && <div className="muted">データセットなし</div>}
          {validation && <div className={validation.ok ? "muted" : "err-line"}>{validation.name}: {validation.ok ? `OK (${validation.num_episodes} ep / ${validation.num_frames} frames, 画像デコード確認済)` : validation.problems?.join(" / ")}</div>}
        </div>
      </div>
    </div>
  );
}
