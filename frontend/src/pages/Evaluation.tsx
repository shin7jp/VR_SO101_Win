import { useEffect, useState } from "react";
import { Badge, TaskSelect } from "../components/common";
import { api, fmtSec, pct } from "../lib/api";

const f = (x: any, d = 3) => (x == null ? "-" : (+x).toFixed(d));

export default function Evaluation() {
  const [task, setTask] = useState("pick_place");
  const [ev, setEv] = useState<any>(null);
  useEffect(() => { api(`/api/evaluation?task_id=${task}`).then(setEv).catch(() => setEv(null)); }, [task]);
  const inf = ev?.inference;
  return (
    <div>
      <h2>Evaluation</h2>
      <div className="topbar"><TaskSelect value={task} onChange={setTask} /></div>
      {ev?.warnings?.map((w: string) => <div key={w} className="banner warn">{w}</div>)}
      <div className="muted">収録データの品質・ACT の学習結果・Gazebo 推論結果は別々に集計します。</div>
      <h3>① 収録データ品質</h3>
      <div className="card"><div className="kv">
        <div><span>エピソード (利用 / 全体)</span><b>{ev?.data_quality.used} / {ev?.data_quality.episodes}</b></div>
        <div><span>総フレーム</span><b>{ev?.data_quality.frames}</b></div>
        <div><span>フレーム欠落率</span><b>{pct(ev?.data_quality.frame_drop_rate)} ({ev?.data_quality.dropped_frames} 枚)</b></div>
        <div><span>検出された問題</span><b>{Object.entries(ev?.data_quality.issues ?? {}).map(([k, v]) => `${k}:${v}`).join(" ") || "なし"}</b></div></div></div>
      <h3>② ACT 学習結果</h3>
      <div className="card"><table><thead><tr><th>モデル</th><th>状態</th><th>steps</th><th>データセット</th><th>最終 loss</th><th>最小 loss</th></tr></thead><tbody>
        {ev?.training.map((t: any) => <tr key={t.model_id}><td className="mono">{t.model_id}</td><td>{t.status}</td><td>{t.steps}</td><td>{t.dataset}</td><td>{f(t.result?.final_loss)}</td><td>{f(t.result?.min_loss)}</td></tr>)}</tbody></table>
        {ev?.training.length === 0 && <div className="muted">学習結果なし</div>}</div>
      <h3>③ Gazebo 推論結果</h3>
      {!inf || inf.n === 0 ? <div className="card muted">推論結果がありません</div> : <>
        <div className="grid2">
          <div className="card"><div className="kv">
            <div><span>エピソード成功率</span><b>{pct(inf.success_rate)} ({inf.n_success}/{inf.n}) <span className="muted">95%CI {pct(inf.success_ci95[0])}–{pct(inf.success_ci95[1])}</span></b></div>
            <div><span>ユニークシード数</span><b>{inf.unique_seeds}{inf.unique_seeds < 10 ? " ⚠ 少ない" : ""}</b></div>
            <div><span>タイムアウト率 / 異常停止率</span><b>{pct(inf.timeout_rate)} / {pct(inf.abnormal_rate)}</b></div>
            <div><span>タスク完了時間 (成功平均)</span><b>{fmtSec(inf.mean_time_to_complete_s)}</b></div>
            <div><span>推論 FPS (制御ループ)</span><b>{f(inf.mean_inference_fps, 1)}</b></div>
            <div><span>動画保存率</span><b>{pct(inf.saved_rate)}</b></div></div></div>
          <div className="card"><div className="kv">
            <div><span>把持成功率 (持ち上げ到達)</span><b>{pct(inf.grasp_success_rate)}</b></div>
            <div><span>物体配置精度 (成功時平均)</span><b>{inf.mean_place_error_m != null ? `${(inf.mean_place_error_m * 1000).toFixed(1)} mm` : "-"}</b></div>
            <div><span>ドア開放角 (最大平均)</span><b>{inf.mean_door_angle_rad != null ? `${f(inf.mean_door_angle_rad, 2)} rad` : "-"}</b></div>
            <div><span>レバー操作率</span><b>{pct(inf.lever_actuated_rate)}</b></div>
            <div><span>評価用シードでの実行</span><b>{inf.eval_split_runs}/{inf.n}</b></div>
            <div><span>学習と重複したシード</span><b>{inf.overlap_runs}</b></div></div></div>
        </div>
        <div className="card"><b>チェックポイント比較</b> {!ev.comparison.same_seed_set && <Badge kind="warn">シード集合が揃っていません (公平な比較になりません)</Badge>}
          <table><thead><tr><th>モデル</th><th>ckpt</th><th>再推論</th><th>n</th><th>成功率</th><th>平均時間</th><th>timeout</th><th>FPS</th></tr></thead><tbody>
            {ev.comparison.rows.map((r: any) => <tr key={r.model_id + r.checkpoint + r.n_action_steps}><td className="mono">{r.model_id}</td><td>{r.checkpoint}</td><td>{r.n_action_steps}</td><td>{r.n}</td>
              <td><b>{pct(r.success_rate)}</b> <span className="muted">[{pct(r.success_ci95[0])}–{pct(r.success_ci95[1])}]</span></td><td>{fmtSec(r.mean_time_to_complete_s)}</td><td>{pct(r.timeout_rate)}</td><td>{f(r.mean_inference_fps, 1)}</td></tr>)}</tbody></table></div>
      </>}
    </div>
  );
}
