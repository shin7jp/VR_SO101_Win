import { useEffect, useState } from "react";
import { Badge, act } from "../components/common";
import { api, getToken, setToken } from "../lib/api";
import { detectXr, deviceClass, XrSupport } from "../lib/xr";

export default function Settings() {
  const [diag, setDiag] = useState<any>(null);
  const [devs, setDevs] = useState<any>(null);
  const [sys, setSys] = useState<any>(null);
  const [xr, setXr] = useState<XrSupport | null>(null);
  const [token, setTok] = useState(getToken());
  const load = () => { api("/api/system/diagnostics").then(setDiag).catch(() => {}); api("/api/system/devices").then(setDevs).catch(() => {}); api("/api/system").then(setSys).catch(() => {}); };
  useEffect(() => { load(); detectXr().then(setXr); }, []);
  return (
    <div>
      <h2>Settings / 環境診断</h2>
      <div className="grid2">
        <div className="card"><div className="row between"><b>環境診断</b><button onClick={load}>再診断</button></div>
          {diag?.items?.map((i: any) => <div key={i.name} style={{ margin: "6px 0" }}>
            <Badge kind={i.ok ? "ok" : i.level === "warning" || i.level === "info" ? "warn" : "err"}>{i.ok ? "OK" : i.level === "info" ? "情報" : i.level === "warning" ? "注意" : "NG"}</Badge> <b>{i.name}</b> <span className="muted">{i.detail}</span>
            {!i.ok && i.fix && <div className="err-line">復旧: {i.fix}</div>}</div>)}
        </div>
        <div className="card"><b>この端末 / アプリ</b>
          <div className="kv" style={{ marginTop: 8 }}>
            <div><span>端末クラス</span><b>{deviceClass()}</b></div>
            <div><span>WebXR</span><b>{xr?.supported ? "対応" : xr?.reason}</b></div>
            <div><span>HTTPS (secure context)</span><b>{window.isSecureContext ? "はい" : "いいえ (WebXR 不可)"}</b></div>
            <div><span>バージョン / データ保存先</span><b className="mono" style={{ fontSize: 11 }}>{sys?.version} / {sys?.data_dir}</b></div>
            <div><span>PyTorch / デバイス</span><b>{devs?.torch} / cuda {devs?.devices?.cuda ? "○" : "×"} mps {devs?.devices?.mps ? "○" : "×"}</b></div></div>
          <h3>API トークン (サーバーが VRSO101_TOKEN を設定している場合)</h3>
          <div className="row"><input type="password" value={token} onChange={(e) => setTok(e.target.value)} /><button onClick={() => { setToken(token); location.reload(); }}>保存</button></div>
          <h3>設定のエクスポート</h3>
          <a href="/api/settings/export"><button>ロボット/シム/タスク設定を JSON で保存</button></a>
        </div>
      </div>
      <div className="card"><b>既知の制約 (README と同じ内容)</b>
        <ul className="muted">
          <li>Gazebo: Humble + Fortress (WSL2)。ヘッドレス描画は ogre (v1) のみ動作。ogre2 は WSL2 では落ちる。</li>
          <li>Open Door / Rotating Lever Door のシーンは Phase 6 の範囲 (未実装のものは UI で選択不可)。</li>
          <li>VR (WebXR) は実機ヘッドセットで未検証 (この開発環境にヘッドセットなし)。コードと座標変換は単体テスト済み。</li>
        </ul></div>
    </div>
  );
}
