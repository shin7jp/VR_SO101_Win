// REST クライアント。エラーは {error:{code,message}} 形式で返るので、利用者に出せる文面へ変換する。
export class ApiFail extends Error {
  constructor(public status: number, public code: string, message: string) {
    super(message);
  }
}

const tokenKey = "vrso101.token";
export const getToken = () => localStorage.getItem(tokenKey) ?? "";
export const setToken = (t: string) => localStorage.setItem(tokenKey, t);

export async function api<T = any>(path: string, opts: { method?: string; body?: unknown } = {}): Promise<T> {
  const headers: Record<string, string> = {};
  if (opts.body !== undefined) headers["Content-Type"] = "application/json";
  const tk = getToken();
  if (tk) headers["X-VRSO101-Token"] = tk;
  let res: Response;
  try {
    res = await fetch(path, { method: opts.method ?? (opts.body !== undefined ? "POST" : "GET"), headers,
      body: opts.body !== undefined ? JSON.stringify(opts.body) : undefined });
  } catch (e) {
    throw new ApiFail(0, "NETWORK", "バックエンドに接続できません。サーバー (uvicorn) が起動しているか確認してください");
  }
  if (!res.ok) {
    let code = "HTTP_" + res.status, msg = res.statusText;
    try {
      const j = await res.json();
      code = j.error?.code ?? code;
      msg = j.error?.message ?? msg;
      if (j.error?.detail) msg += " " + JSON.stringify(j.error.detail).slice(0, 200);
    } catch { /* 本文なし */ }
    throw new ApiFail(res.status, code, msg);
  }
  const ct = res.headers.get("content-type") ?? "";
  return (ct.includes("json") ? res.json() : (res.text() as unknown)) as Promise<T>;
}

export const fmtSec = (s?: number | null) => (s == null ? "-" : s >= 60 ? `${Math.floor(s / 60)}分${Math.round(s % 60)}秒` : `${s.toFixed(1)}秒`);
export const pct = (x?: number | null) => (x == null ? "-" : (x * 100).toFixed(0) + "%");
