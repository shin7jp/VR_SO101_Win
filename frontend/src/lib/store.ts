import { create } from "zustand";
import { api } from "./api";

export type Task = { task_id: string; name: string; description: string; instruction: string; fps: number; max_steps: number; implemented: boolean; success: any; camera: any };
export type Toast = { id: number; kind: "info" | "error" | "ok"; text: string };
export type LiveState = {
  mode: string; task_id: string | null; q: number[] | null; q_cmd: number[] | null; ee: number[] | null; teleop: string;
  sim_time: number | null; task_state: any; objects: Record<string, { pos: number[]; quat: number[] }>;
  recording: { episode_id: string; frames: number; seconds: number } | null; tick_ms: number; overruns: number; step_ms: number | null; is_mock: boolean;
};

type S = {
  status: any | null; live: LiveState | null; robot: any | null; tasks: Task[];
  wsState: "connecting" | "open" | "closed"; lastSeq: number; events: any[]; toasts: Toast[]; jobsTick: number;
  refresh: () => Promise<void>; loadStatic: () => Promise<void>;
  setWs: (s: "connecting" | "open" | "closed") => void; ingest: (ev: any) => void;
  toast: (kind: Toast["kind"], text: string) => void; dismiss: (id: number) => void;
};

let tid = 1;
export const useStore = create<S>((set, get) => ({
  status: null, live: null, robot: null, tasks: [], wsState: "connecting", lastSeq: 0, events: [], toasts: [], jobsTick: 0,
  refresh: async () => { try { set({ status: await api("/api/sim/status") }); } catch { /* 接続断は wsState で表示 */ } },
  loadStatic: async () => {
    try {
      const [robot, tasks] = await Promise.all([api("/api/robot"), api("/api/tasks")]);
      set({ robot, tasks });
    } catch (e: any) { get().toast("error", e.message); }
  },
  setWs: (wsState) => set({ wsState }),
  ingest: (ev) => {
    if (ev.seq) set({ lastSeq: Math.max(get().lastSeq, ev.seq) });
    if (ev.type === "state") set({ live: ev });
    else if (ev.type === "hello") set({ status: ev.status });
    else {
      set({ events: [ev, ...get().events].slice(0, 100) });
      if (ev.type === "mode" || ev.type === "episode") get().refresh();
      if (ev.type === "job" || ev.type === "inference") set({ jobsTick: get().jobsTick + 1 });
      if (ev.type === "error") get().toast("error", `${ev.code}: ${ev.message}`);
      if (ev.type === "episode" && ev.event === "finished") get().toast(ev.meta.status === "success" ? "ok" : "info", `エピソード終了: ${ev.meta.status} (${ev.meta.num_frames} フレーム)`);
    }
  },
  toast: (kind, text) => {
    const id = tid++;
    set({ toasts: [...get().toasts, { id, kind, text }] });
    setTimeout(() => get().dismiss(id), kind === "error" ? 9000 : 4000);
  },
  dismiss: (id) => set({ toasts: get().toasts.filter((t) => t.id !== id) }),
}));

export const act = async <T,>(fn: () => Promise<T>, okMsg?: string): Promise<T | undefined> => {
  try { const r = await fn(); if (okMsg) useStore.getState().toast("ok", okMsg); return r; }
  catch (e: any) { useStore.getState().toast("error", e.message ?? String(e)); return undefined; }
};
