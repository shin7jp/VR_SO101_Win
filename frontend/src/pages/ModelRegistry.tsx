import { useEffect, useState } from "react";
import { Badge, LineChart, TaskSelect, act } from "../components/common";
import { api } from "../lib/api";

export default function ModelRegistry() {
  const [task, setTask] = useState("pick_place");
  const [models, setModels] = useState<any[]>([]);
  const [sel, setSel] = useState<any>(null);
  const load = () => api(`/api/models?task_id=${task}`).then(setModels).catch(() => {});
  useEffect(() => { load(); setSel(null); }, [task]);
  const del = async (m: any) => { if (confirm(`${m.model_id} を削除します (重み・設定・履歴すべて)`)) { await act(() => api(`/api/models/${task}/${m.model_id}`, { method: "DELETE" }), "削除しました"); setSel(null); load(); } };
  return (
    <div>
      <h2>Model Registry</h2>
      <div className="topbar"><TaskSelect value={task} onChange={setTask} /><span className="muted">{models.length} モデル</span></div>
      <div className="grid2">
        <div className="card"><table><thead><tr><th>モデル</th><th>状態</th><th>データセット</th><th>steps</th><th>loss</th></tr></thead><tbody>
          {models.map((m) => <tr key={m.model_id} className={sel?.model_id === m.model_id ? "sel" : ""} onClick={() => setSel(m)}>
            <td className="mono">{m.model_id}</td><td><Badge kind={m.status === "trained" ? "ok" : m.status === "training" ? "info" : "err"}>{m.status}</Badge></td>
            <td>{m.dataset?.name} <span className="muted">({m.dataset?.num_episodes}ep)</span></td><td>{m.train_config?.steps}</td><td>{m.result?.final_loss ?? "-"}</td></tr>)}
        </tbody></table>{models.length === 0 && <div className="muted">学習済みモデルがありません。ACT Training で学習してください。</div>}</div>
        <div className="card">
          {!sel ? <div className="muted">モデルを選択してください</div> : <>
            <b className="mono">{sel.model_id}</b>
            <div className="kv" style={{ marginTop: 8 }}>
              <div><span>データセット ID</span><b className="mono">{sel.dataset?.repo_id}</b></div>
              <div><span>学習条件</span><b>steps {sel.train_config?.steps} / batch {sel.train_config?.batch_size} / {sel.train_config?.device} / chunk {sel.train_config?.chunk_size}</b></div>
              <div><span>fps / 関節</span><b>{sel.fps} fps / {sel.robot?.joint_names?.length} 関節 ({sel.robot?.units})</b></div>
              <div><span>LeRobot</span><b>{sel.lerobot_version}</b></div>
              <div><span>学習に使ったシード</span><b className="mono" style={{ maxWidth: 220, overflow: "hidden", textOverflow: "ellipsis" }}>{(sel.dataset?.seeds ?? []).slice(0, 12).join(",")}{(sel.dataset?.seeds?.length ?? 0) > 12 ? "…" : ""}</b></div>
              <div><span>結果</span><b>{sel.result ? `final ${sel.result.final_loss} / min ${sel.result.min_loss}` : "-"}</b></div>
            </div>
            {sel.result?.error && <div className="err-line">{sel.result.error}</div>}
            {sel.result?.loss_history?.length > 1 && <LineChart height={140} series={[{ name: "loss", color: "#3aa0ff", pts: sel.result.loss_history.map((x: any) => [x.step, x.loss] as [number, number]) }]} />}
            <h3>チェックポイント</h3>
            <table><tbody>{sel.checkpoints?.map((c: any) => <tr key={c.name}><td className="mono">{c.name}</td><td>{c.size_mb} MB</td><td>{c.ok ? <Badge kind="ok">OK</Badge> : <Badge kind="err">破損</Badge>}</td></tr>)}</tbody></table>
            <div className="row"><button className="primary" onClick={() => { localStorage.setItem("vrso101.model", sel.model_id); location.hash = "infer"; }}>この推論へ →</button>
              <button className="danger" onClick={() => del(sel)}>削除</button></div>
          </>}
        </div>
      </div>
    </div>
  );
}
