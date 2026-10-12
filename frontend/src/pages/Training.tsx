import { useEffect, useState } from "react";
import { Badge, JobCard, LineChart, TaskSelect, act, useJobs } from "../components/common";
import { api } from "../lib/api";

export default function Training() {
  const [task, setTask] = useState("pick_place");
  const [datasets, setDatasets] = useState<any[]>([]);
  const [ds, setDs] = useState("");
  const [devs, setDevs] = useState<any>(null);
  const [cfg, setCfg] = useState({ steps: 2000, batch_size: 8, device: "cuda", save_freq: 500, chunk_size: 50, num_workers: 2, lr: "", name: "" });
  const [validation, setValidation] = useState<any>(null);
  const [selJob, setSelJob] = useState<string | null>(null);
  const [jobDetail, setJobDetail] = useState<any>(null);
  const { jobs, reload } = useJobs("train", 1500);

  useEffect(() => { api("/api/system/devices").then((d) => { setDevs(d); if (!d.devices.cuda) setCfg((c) => ({ ...c, device: "cpu" })); }).catch(() => {}); }, []);
  useEffect(() => { api(`/api/datasets?task_id=${task}`).then((d) => { setDatasets(d); setDs(d[0]?.name ?? ""); }).catch(() => {}); }, [task]);
  useEffect(() => { setValidation(null); if (ds) api(`/api/datasets/${task}/${ds}/validate`).then(setValidation).catch(() => {}); }, [ds, task]);
  const active = jobs.find((j) => j.state === "running");
  const shown = selJob ?? active?.id ?? jobs[0]?.id ?? null;
  useEffect(() => {
    if (!shown) { setJobDetail(null); return; }
    const f = () => api(`/api/jobs/${shown}?log_lines=25`).then(setJobDetail).catch(() => {});
    f(); const t = window.setInterval(f, 2000); return () => window.clearInterval(t);
  }, [shown]);

  const start = async () => {
    const r = await act(() => api("/api/training/start", { body: { task_id: task, dataset: ds, steps: cfg.steps, batch_size: cfg.batch_size, device: cfg.device, save_freq: cfg.save_freq, chunk_size: cfg.chunk_size, num_workers: cfg.num_workers, lr: cfg.lr ? +cfg.lr : null, name: cfg.name || null } }), "学習を開始しました");
    if (r) { setSelJob(r.id); reload(); }
  };
  const stop = async (id: string) => { await act(() => api(`/api/jobs/${id}/stop`, { method: "POST" }), "停止しました"); reload(); };
  const m = jobDetail?.metrics ?? [];
  const meta = datasets.find((d) => d.name === ds);
  const num = (k: keyof typeof cfg) => (e: any) => setCfg({ ...cfg, [k]: +e.target.value });
  return (
    <div>
      <h2>ACT Training</h2>
      <div className="grid2">
        <div className="card">
          <div className="row"><TaskSelect value={task} onChange={setTask} />
            <select value={ds} onChange={(e) => setDs(e.target.value)}>{datasets.length === 0 && <option value="">データセットなし</option>}{datasets.map((d) => <option key={d.name} value={d.name}>{d.name}</option>)}</select></div>
          {meta && <div className="kv"><div><span>使用エピソード</span><b>{meta.num_episodes} 本 / {meta.num_frames} フレーム</b></div><div><span>除外されたエピソード</span><b>{meta.num_skipped}</b></div>
            <div><span>種別</span><b>{Object.entries(meta.collection_modes ?? {}).map(([k, v]) => `${k}:${v}`).join(" ")}</b></div></div>}
          {validation && <div className={validation.ok ? "muted" : "err-line"}>データセット検証: {validation.ok ? "OK (特徴量・次元・画像デコード確認済)" : validation.problems?.join(" / ")}</div>}
          {meta && meta.num_episodes < 20 && <div className="banner warn" style={{ marginTop: 8 }}>エピソードが {meta.num_episodes} 本と少なめです。ACT は通常 50 本前後以上を推奨します (過学習・汎化不足に注意)。</div>}
          <h3>学習設定</h3>
          <div className="row"><b>ステップ数</b><input type="number" value={cfg.steps} onChange={num("steps")} style={{ width: 100 }} />
            <b>バッチ</b><input type="number" value={cfg.batch_size} onChange={num("batch_size")} style={{ width: 70 }} />
            <b>保存間隔</b><input type="number" value={cfg.save_freq} onChange={num("save_freq")} style={{ width: 90 }} /></div>
          <div className="row"><b>デバイス</b>
            <select value={cfg.device} onChange={(e) => setCfg({ ...cfg, device: e.target.value })}>
              <option value="cuda" disabled={!devs?.devices.cuda}>cuda{devs?.cuda_name ? ` (${devs.cuda_name})` : " (利用不可)"}</option>
              <option value="mps" disabled={!devs?.devices.mps}>mps{devs?.devices.mps ? "" : " (利用不可)"}</option><option value="cpu">cpu</option></select>
            <b>chunk</b><input type="number" value={cfg.chunk_size} onChange={num("chunk_size")} style={{ width: 70 }} />
            <b>workers</b><input type="number" value={cfg.num_workers} onChange={num("num_workers")} style={{ width: 60 }} /></div>
          <div className="row"><b>LR</b><input type="text" placeholder="既定 1e-5" value={cfg.lr} onChange={(e) => setCfg({ ...cfg, lr: e.target.value })} style={{ width: 90 }} />
            <b>名前</b><input type="text" placeholder="任意" value={cfg.name} onChange={(e) => setCfg({ ...cfg, name: e.target.value })} style={{ width: 120 }} /></div>
          <button className="primary" disabled={!ds || !!active || (validation && !validation.ok)} onClick={start}>{active ? "学習中 (同時に 1 つまで)" : "学習開始"}</button>
          <div className="muted">外部送信なし (W&B・Hub 公開は無効)。ResNet18 の ImageNet 事前学習重みのみ初回に公開サーバーから取得します。</div>
        </div>
        <div className="card">
          <div className="row between"><b>損失の推移</b>{jobDetail && <Badge kind={jobDetail.state === "running" ? "info" : jobDetail.state === "succeeded" ? "ok" : "err"}>{jobDetail.state}</Badge>}</div>
          <LineChart series={[
            { name: "loss", color: "#3aa0ff", pts: m.map((x: any) => [x.step, x.loss] as [number, number]) },
            { name: "l1", color: "#34c759", pts: m.filter((x: any) => x.l1_loss != null).map((x: any) => [x.step, x.l1_loss] as [number, number]) }]} />
          {jobDetail && <JobCard job={jobDetail} onStop={() => stop(jobDetail.id)} />}
          {jobDetail?.log_tail && <pre className="mono" style={{ fontSize: 11, maxHeight: 180, overflow: "auto", background: "#0b0f13", padding: 8, borderRadius: 8 }}>{jobDetail.log_tail}</pre>}
        </div>
      </div>
      <h3>学習履歴</h3>
      <div className="card"><table><thead><tr><th>ジョブ</th><th>モデル</th><th>状態</th><th>ステップ</th><th>最新 loss</th></tr></thead><tbody>
        {jobs.map((j) => <tr key={j.id} className={shown === j.id ? "sel" : ""} onClick={() => setSelJob(j.id)}><td className="mono">{j.id}</td><td>{j.params?.model_id}</td><td>{j.state}</td><td>{j.progress?.step}/{j.progress?.total}</td><td>{j.last_metric?.loss ?? "-"}</td></tr>)}
      </tbody></table>{jobs.length === 0 && <div className="muted">履歴なし</div>}</div>
    </div>
  );
}
