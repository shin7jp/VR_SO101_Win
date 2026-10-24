import { useEffect, useState } from "react";
import { Badge, TaskSelect, act, downloadFile } from "../components/common";
import { api, fmtSec } from "../lib/api";

export default function VideoGallery() {
  const [task, setTask] = useState("pick_place");
  const [succ, setSucc] = useState("all");
  const [runs, setRuns] = useState<any[]>([]);
  const [sel, setSel] = useState<any>(null);
  const [cmp, setCmp] = useState<string[]>([]);
  const load = () => api(`/api/inference/videos?task_id=${task}${succ === "all" ? "" : `&success=${succ === "ok"}`}`).then(setRuns).catch(() => {});
  useEffect(() => { load(); setSel(null); }, [task, succ]);
  const del = async (r: any) => { if (confirm("この推論動画とメタデータを削除します")) { await act(() => api(`/api/inference/videos/${task}/${r.episode_id}`, { method: "DELETE" })); setSel(null); load(); } };
  const toggleCmp = (id: string) => setCmp(cmp.includes(id) ? cmp.filter((x) => x !== id) : [...cmp, id].slice(-2));
  const url = (r: any, dl = false) => `/api/inference/videos/${task}/${r.episode_id}/video${dl ? "?download=true" : ""}`;
  const compared = runs.filter((r) => cmp.includes(r.episode_id));
  return (
    <div>
      <h2>Inference Video Gallery</h2>
      <div className="topbar"><TaskSelect value={task} onChange={setTask} />
        <select value={succ} onChange={(e) => setSucc(e.target.value)}><option value="all">すべて</option><option value="ok">成功のみ</option><option value="ng">失敗のみ</option></select>
        <span className="muted">{runs.length} 本 (保存済みのみ再生可)</span></div>
      {compared.length === 2 && <div className="card"><h3>比較 (同じ画面で再生)</h3><div className="grid2">
        {compared.map((r) => <div key={r.episode_id}><div className="muted mono">{r.policy?.model_id} · {r.policy?.checkpoint} · seed {r.seed} · <Badge kind={r.success ? "ok" : "warn"}>{r.outcome}</Badge></div><video src={url(r)} controls playsInline muted loop /></div>)}</div></div>}
      <div className="grid2">
        <div className="card" style={{ maxHeight: 600, overflow: "auto" }}>
          <table><thead><tr><th /><th>日時 / ID</th><th>ポリシー</th><th>seed</th><th>結果</th><th>時間</th></tr></thead><tbody>
            {runs.map((r) => <tr key={r.episode_id} className={sel?.episode_id === r.episode_id ? "sel" : ""} onClick={() => setSel(r)}>
              <td onClick={(e) => e.stopPropagation()}><input type="checkbox" checked={cmp.includes(r.episode_id)} onChange={() => toggleCmp(r.episode_id)} title="比較 (2 本まで)" /></td>
              <td className="mono">{r.started_at?.slice(5, 16)}<div className="muted">{r.episode_id}</div></td>
              <td className="mono">{r.policy?.model_id?.split("-").slice(-2).join("-")}<div className="muted">{r.policy?.checkpoint}</div></td><td>{r.seed}</td>
              <td><Badge kind={r.success ? "ok" : r.outcome === "abnormal" ? "err" : "warn"}>{r.outcome}</Badge>{!r.saved && <Badge kind="err">未保存</Badge>}</td><td>{fmtSec(r.sim_seconds)}</td></tr>)}
          </tbody></table>
          {runs.length === 0 && <div className="muted">推論動画がありません。Gazebo Inference で実行してください。</div>}
        </div>
        <div className="card">
          {!sel ? <div className="muted">動画を選択してください</div> : <>
            {sel.saved ? <video key={sel.episode_id} src={url(sel)} controls playsInline loop autoPlay muted /> : <div className="banner err">動画が保存されていません: {sel.save_error}</div>}
            <div className="row"><button onClick={() => downloadFile(url(sel, true))}>ダウンロード (MP4)</button><button className="danger" onClick={() => del(sel)}>削除</button></div>
            <div className="kv">
              <div><span>結果</span><b>{sel.outcome} ({sel.reason})</b></div>
              <div><span>ポリシー / ckpt</span><b className="mono">{sel.policy?.model_id} / {sel.policy?.checkpoint}</b></div>
              <div><span>データセット</span><b className="mono">{sel.dataset}</b></div>
              <div><span>フレーム / 推論FPS</span><b>{sel.frames} / {sel.inference_fps}</b></div>
              <div><span>実行時間 (wall / sim)</span><b>{sel.wall_seconds}s / {sel.sim_seconds}s</b></div>
              <div><span>再推論間隔 / 速度制限</span><b>{sel.policy?.n_action_steps} step / {sel.policy?.speed_scale}×</b></div>
              <div><span>seed / 評価区分</span><b>{sel.seed} / {sel.evaluation_split}{sel.train_eval_overlap ? " ⚠学習と重複" : ""}</b></div>
              <div><span>バックエンド</span><b>{sel.sim?.backend}</b></div>
            </div>
            <details><summary className="muted">metadata.json 全体</summary><pre className="mono" style={{ fontSize: 11, overflow: "auto", maxHeight: 220 }}>{JSON.stringify(sel, null, 1)}</pre></details>
          </>}
        </div>
      </div>
    </div>
  );
}
