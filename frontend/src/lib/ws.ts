// 自動再接続つき WebSocket。切断→再接続時は最後の seq から再送を要求し、欠落していたら REST で状態を取り直す。
import { getToken } from "./api";
import { useStore } from "./store";

let sock: WebSocket | null = null;
let timer: number | undefined;
let backoff = 500;
let closedByUs = false;

export function sendWs(msg: unknown): boolean {
  if (sock && sock.readyState === WebSocket.OPEN) { sock.send(JSON.stringify(msg)); return true; }
  return false;
}

export function connectWs() {
  closedByUs = false;
  const st = useStore.getState();
  st.setWs("connecting");
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const since = st.lastSeq > 0 ? `since=${st.lastSeq}` : "";
  const tk = getToken() ? `token=${encodeURIComponent(getToken())}` : "";
  const q = [since, tk].filter(Boolean).join("&");
  sock = new WebSocket(`${proto}://${location.host}/ws${q ? "?" + q : ""}`);
  sock.onopen = () => { backoff = 500; };
  sock.onmessage = (m) => {
    const ev = JSON.parse(m.data);
    if (ev.type === "hello") {
      useStore.getState().setWs("open");
      if (ev.resync_required) { useStore.getState().refresh(); useStore.getState().toast("info", "再接続: 欠落イベントがあったため状態を再取得しました"); }
    }
    if (ev.type === "pong") return;
    useStore.getState().ingest(ev);
  };
  sock.onclose = () => {
    useStore.getState().setWs("closed");
    if (closedByUs) return;
    timer = window.setTimeout(connectWs, backoff);
    backoff = Math.min(backoff * 2, 5000);
  };
  sock.onerror = () => sock?.close();
}

export function disconnectWs() { closedByUs = true; window.clearTimeout(timer); sock?.close(); }
