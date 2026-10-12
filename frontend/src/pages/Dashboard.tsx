import { useEffect, useState } from "react";
import { api, pct } from "../lib/api";
import { useStore } from "../lib/store";
import { Badge, SimStatusPanel, useJobs, JobCard, MockWarning } from "../components/common";

export default function Dashboard() {
  const tasks = useStore((s) => s.tasks);
  const [diag, setDiag] = useState<any>(null);
  const [counts, setCounts] = useState<any>({});
  const { jobs } = useJobs();
  useEffect(() => {
    api("/api/system/diagnostics").then(setDiag).catch(() => {});
    (async () => {
      const c: any = {};
      for (const t of tasks) {
        const [e, m, v] = await Promise.all([api(`/api/episodes?task_id=${t.task_id}`), api(`/api/models?task_id=${t.task_id}`), api(`/api/inference/videos?task_id=${t.task_id}`)]);
        c[t.task_id] = { eps: e.stats, models: m.length, videos: v.length, succ: v.filter((x: any) => x.success).length };
      }
      setCounts(c);
    })().catch(() => {});
  }, [tasks.length]);
  const bad = diag?.items?.filter((i: any) => !i.ok) ?? [];
  const running = jobs.filter((j) => j.state === "running");
  return (
    <div>
      <h2>Dashboard</h2>
      <MockWarning />
      {bad.length > 0 && <div className="banner err">環境に問題があります: {bad.map((b: any) => b.name).join(", ")} — Settings で復旧手順を確認してください</div>}
      <div className="grid2">
        <div className="card"><h3>システム</h3><SimStatusPanel /></div>
        <div className="card"><h3>実行中のジョブ</h3>
          {running.length === 0 ? <div className="muted">なし</div> : running.map((j) => <JobCard key={j.id} job={j} />)}
        </div>
      </div>
      <h3>タスク</h3>
      <div className="grid2">
        {tasks.map((t) => {
          const c = counts[t.task_id];
          return (
            <div className="card" key={t.task_id}>
              <div className="row between"><b>{t.name}</b>{t.implemented ? <Badge kind="ok">シーン実装済</Badge> : <Badge kind="warn">シーン未実装</Badge>}</div>
              <div className="muted">{t.description}</div>
              {c && <div className="kv" style={{ marginTop: 8 }}>
                <div><span>収録エピソード</span><b>{c.eps.total} (成功率 {pct(c.eps.success_rate)})</b></div>
                <div><span>収録時間</span><b>{Math.round(c.eps.recorded_seconds)} 秒</b></div>
                <div><span>学習済みモデル</span><b>{c.models}</b></div>
                <div><span>推論動画</span><b>{c.videos} (成功 {c.succ})</b></div>
              </div>}
            </div>
          );
        })}
      </div>
    </div>
  );
}
