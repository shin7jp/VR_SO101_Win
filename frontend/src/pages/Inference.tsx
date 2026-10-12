import { useEffect, useState } from "react";
import { Badge, CameraView, JobCard, JointBars, TaskSelect, act, useJobs } from "../components/common";
import { api, fmtSec } from "../lib/api";
import { useStore } from "../lib/store";

export default function Inference() {
  const live = useStore((s) => s.events.find((e) => e.type === "inference" && e.kind === "step"));
  const [task, setTask] = useState("pick_place");
  const [models, setModels] = useState<any[]>([]);
  const [model, setModel] = useState(localStorage.getItem("vrso101.model") ?? "");
  const [cks, setCks] = useState<string[]>(["last"]);
  const [seed0, setSeed0] = useState(10000);
  const [count, setCount] = useState(5);
  const [nas, setNas] = useState(10);
  const [speed, setSpeed] = useState(1);
  const [maxSteps, setMaxSteps] = useState("");
  const [device, setDevice] = useState("cuda");
  const [warnings, setWarnings] = useState<string[]>([]);
  const { jobs, reload } = useJobs("infer", 1500);
  const active = jobs.find((j) => j.state === "running");
  const [detail, setDetail] = useState<any>(null);
  useEffect(() => { api(`/api/models?task_id=${task}`).then((m) => { const ok = m.filter((x: any) => x.checkpoints?.length); setModels(ok); if (!ok.find((x: any) => x.model_id === model)) setModel(ok[0]?.model_id ?? ""); }).catch(() => {}); }, [task]);
  const cur = models.find((m) => m.model_id === model);
  const shown = active?.id ?? jobs[0]?.id;
  useEffect(() => {
    if (!shown) return setDetail(null);
    const f = () => api(`/api/jobs/${shown}?log_lines=15`).then(setDetail).catch(() => {});
    f(); const t = window.setInterval(f, 1500); return () => window.clearInterval(t);
  }, [shown, active?.state]);

  const start = async () => {
    const seeds = Array.from({ length: count }, (_, i) => seed0 + i);
    const r = await act(() => api("/api/inference/start", { body: { task_id: task, model_id: model, checkpoints: cks, seeds, n_action_steps: nas, speed_scale: speed, max_steps: maxSteps ? +maxSteps : null, device } }), "推論を開始しました (シムをワーカーへ引き渡します)");
    if (r?.warnings) setWarnings(r.warnings);
    reload();
  };
  const stop = () => act(() => api("/api/inference/stop", { method: "POST" }), "停止を要求しました (動画を保存してから終了します)");
  const toggleCk = (n: string) => setCks(cks.includes(n) ? cks.filter((x) => x !== n) : [...cks, n]);
  const eps = detail?.result?.episodes ?? [];
  const lv = detail?.result?.live ?? live;
  return (
    <div>
      <h2>Gazebo Inference</h2>
      <div className="grid2">
        <div className="card">
          <div className="row"><TaskSelect value={task} onChange={setTask} />
            <select value={model} onChange={(e) => setModel(e.target.value)}>{models.length === 0 && <option value="">モデルなし</option>}{models.map((m) => <option key={m.model_id} value={m.model_id}>{m.model_id}</option>)}</select></div>
          {cur && <>
            <div className="kv"><div><span>データセット</span><b className="mono">{cur.dataset?.repo_id}</b></div><div><span>学習 steps / 使用エピソード</span><b>{cur.train_config?.steps} / {cur.dataset?.num_episodes}</b></div></div>
            <div className="row"><b>チェックポイント</b>{cur.checkpoints.map((c: any) => <label key={c.name} title={c.ok ? "" : "破損"}><input type="checkbox" disabled={!c.ok} checked={cks.includes(c.name) || (cks.includes("last") && c.name === cur.checkpoints[cur.checkpoints.length - 1].name)} onChange={() => toggleCk(c.name)} /> {c.name}{c.ok ? "" : " ⚠破損"}</label>)}
              <label><input type="checkbox" checked={cks.includes("last")} onChange={() => toggleCk("last")} /> last</label></div>
          </>}
          <div className="row"><b>評価シード</b><input type="number" value={seed0} onChange={(e) => setSeed0(+e.target.value)} style={{ width: 90 }} /><b>から</b><input type="number" min={1} max={100} value={count} onChange={(e) => setCount(+e.target.value)} style={{ width: 60 }} /><b>回</b></div>
          <div className="muted">10000 以上は学習に使っていない評価用シード。複数の初期配置で評価してください (単一シードの成功を一般的な成功とみなさない)。</div>
          <div className="row"><b>再推論まで</b><input type="number" min={1} max={200} value={nas} onChange={(e) => setNas(+e.target.value)} style={{ width: 70 }} /><b>ステップ</b>
            <b>速度制限</b><input type="range" min={0.2} max={2} step={0.1} value={speed} onChange={(e) => setSpeed(+e.target.value)} /><span className="mono">{speed.toFixed(1)}×</span></div>
          <div className="row"><b>最大ステップ</b><input type="number" placeholder="タスク既定" value={maxSteps} onChange={(e) => setMaxSteps(e.target.value)} style={{ width: 100 }} />
            <b>デバイス</b><select value={device} onChange={(e) => setDevice(e.target.value)}><option>cuda</option><option>cpu</option></select></div>
          {warnings.map((w) => <div key={w} className="banner warn">{w}</div>)}
          <div className="row">
            <button className="primary" disabled={!model || !!active || cks.length === 0} onClick={start}>推論開始</button>
            <button className="danger" disabled={!active} onClick={stop}>即時停止</button>
          </div>
          <div className="muted">推論中はシムを推論ワーカーが専有します (テレオペ・収録は不可)。読み込み失敗・画像/関節が取得できない場合は動作を開始せず、原因を下に表示します。</div>
        </div>
        <div>
          <CameraView src="/api/inference/live.mjpg" label="推論中の Gazebo カメラ (録画される映像)" />
          {lv && <div className="card" style={{ marginTop: 12 }}><div className="kv"><div><span>ステップ</span><b>{lv.step}</b></div><div><span>sim 時刻</span><b className="mono">{lv.sim_time}</b></div>
            <div><span>物理判定</span><b>{lv.success ? "成功" : "-"}</b></div>
            {lv.q && <div><span>関節</span><b className="mono" style={{ fontSize: 11 }}>{lv.q.map((v: number) => v.toFixed(2)).join(" ")}</b></div>}</div></div>}
        </div>
      </div>
      {detail && <div className="card"><div className="row between"><b>実行結果</b><Badge kind={detail.state === "running" ? "info" : detail.state === "succeeded" ? "ok" : "err"}>{detail.state}</Badge></div>
        <JobCard job={detail} />
        {detail.error && <div className="banner err">{detail.error}</div>}
        <table><thead><tr><th>チェックポイント</th><th>seed</th><th>結果</th><th>理由</th><th>sim 時間</th><th>保存</th><th /></tr></thead><tbody>
          {eps.map((e: any) => <tr key={e.episode_id}><td>{e.checkpoint}</td><td>{e.seed}</td><td><Badge kind={e.success ? "ok" : e.outcome === "abnormal" ? "err" : "warn"}>{e.outcome}</Badge></td><td>{e.reason}</td><td>{fmtSec(e.sim_seconds)}</td>
            <td>{e.saved ? "✔ 保存済み" : <span className="err-line">{e.save_error}</span>}</td><td><a href="#videos">動画</a></td></tr>)}
        </tbody></table>
        {detail.log_tail && <details><summary className="muted">ログ</summary><pre className="mono" style={{ fontSize: 11, overflow: "auto", maxHeight: 160 }}>{detail.log_tail}</pre></details>}
      </div>}
    </div>
  );
}
