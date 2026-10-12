// PC (キーボード) / iPhone・iPad (方向パッド・グリッパースライダー) 用の操作パネル。
// 押している間だけ EE 速度指令を送り続け、離す/途切れると バックエンド側で保持 (安全停止) になる。
import { useEffect, useRef, useState } from "react";
import { sendWs } from "../lib/ws";
import { isTouchDevice } from "../lib/xr";
import { inputState } from "../lib/input";

const BASE = 0.08; // m/s (バックエンドの speed 倍率が掛かる)
const ROT = 2.4;   // rad/s (手首。初期値 1.2 の 2 倍。実際の速さは関節の速度制限が上限)

type Vec = { v: [number, number, number]; w: [number, number, number] };

export default function TeleopPanel({ speed, setSpeed }: { speed: number; setSpeed: (s: number) => void }) {
  const held = useRef<Record<string, boolean>>({});
  const [grip, setGrip] = useState(0.45);       // 0=閉 1=開
  const gripRef = useRef(grip);
  gripRef.current = grip;
  // グリッパー指令は「自分が操作した直後だけ」送る。何も触っていないパネルが 40ms ごとに自分の値を送ると、
  // 別タブ/別端末のパネルと指令を奪い合ってグリッパーがふらつく (実測: q_cmd が 2 値を交互に往復)。
  const gripTouched = useRef(0);
  const firstGrip = useRef(true);
  useEffect(() => { if (firstGrip.current) { firstGrip.current = false; return; } gripTouched.current = performance.now(); }, [grip]);
  const movingRef = useRef(false);
  const [, force] = useState(0);
  const [vert, setVert] = useState(inputState.vertical);
  const [pressed, setPressed] = useState<Record<string, boolean>>({});   // 画面のキー表示用 (押している間ハイライト)
  const touch = isTouchDevice();

  const compute = (): Vec | null => {
    const h = held.current;
    const v: [number, number, number] = [0, 0, 0], w: [number, number, number] = [0, 0, 0];
    if (h.w || h.fwd) v[0] += BASE; if (h.s || h.back) v[0] -= BASE;
    if (h.a || h.left) v[1] += BASE; if (h.d || h.right) v[1] -= BASE;
    if (h.r || h.up) v[2] += BASE; if (h.f || h.down) v[2] -= BASE;
    if (h.q) w[0] += ROT; if (h.e) w[0] -= ROT;
    if (h.ArrowUp) w[1] += ROT; if (h.ArrowDown) w[1] -= ROT;
    if (h.ArrowLeft) w[2] += ROT; if (h.ArrowRight) w[2] -= ROT;
    const any = v.some((x) => x) || w.some((x) => x);
    return any ? { v, w } : null;
  };

  useEffect(() => {
    const kd = (e: KeyboardEvent) => {
      const el = e.target as HTMLElement;
      const tag = el?.tagName;
      // 文字入力欄 (シード等) だけはキーを渡さない。スライダー/ボタン/チェックボックスにフォーカスが残っていると
      // キーが無視される (途中で効かなくなる) ほか、矢印でスライダーが動き Space でボタンが押されてしまうので、
      // フォーカスを外してから操作キーとして扱う。
      if (tag === "SELECT" || tag === "TEXTAREA" || (tag === "INPUT" && !["range", "checkbox", "radio"].includes((el as HTMLInputElement).type))) return;
      const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
      const ours = k === "z" || k === "x" || k === " " || "wasdrfqe".includes(k) && k.length === 1 || k.startsWith("Arrow");
      if (ours && (tag === "BUTTON" || tag === "INPUT")) el.blur();
      if (k === "z") setGrip((g) => Math.max(0, g - 0.1));
      else if (k === "x") setGrip((g) => Math.min(1, g + 0.1));
      else if (k === " ") { e.preventDefault(); if (!e.repeat) setGrip((g) => (g > 0.2 ? 0 : 0.6)); }
      else if (ours) { held.current[k] = true; if (k.startsWith("Arrow")) e.preventDefault(); }
      if (ours) setPressed((p) => (p[k] ? p : { ...p, [k]: true }));
    };
    const ku = (e: KeyboardEvent) => { const k = e.key.length === 1 ? e.key.toLowerCase() : e.key; delete held.current[k]; if (k === " ") e.preventDefault(); setPressed((p) => { if (!p[k]) return p; const n = { ...p }; delete n[k]; return n; }); };
    const blur = () => { held.current = {}; setPressed({}); sendWs({ type: "release" }); };
    window.addEventListener("keydown", kd); window.addEventListener("keyup", ku); window.addEventListener("blur", blur);
    const t = window.setInterval(() => {
      const c = compute();
      const g = performance.now() - gripTouched.current < 1000 ? gripRef.current : undefined;   // 操作後 1 秒だけ送る
      if (c) { movingRef.current = true; sendWs({ type: "nudge", v: c.v, w: c.w, gripper: g }); }
      else if (movingRef.current) { movingRef.current = false; sendWs({ type: "nudge", v: [0, 0, 0], w: [0, 0, 0], gripper: g }); }   // 離した瞬間に 1 回だけ速度 0 (残ると 0.4 秒流れる)
      else if (g !== undefined) sendWs({ type: "nudge", v: [0, 0, 0], w: [0, 0, 0], gripper: g });
    }, 40);
    return () => { window.removeEventListener("keydown", kd); window.removeEventListener("keyup", ku); window.removeEventListener("blur", blur); window.clearInterval(t); sendWs({ type: "release" }); };
  }, []);

  useEffect(() => { sendWs({ type: "speed", value: speed }); }, [speed]);

  const hold = (key: string) => ({
    onPointerDown: (e: React.PointerEvent) => { (e.target as HTMLElement).setPointerCapture(e.pointerId); held.current[key] = true; force((x) => x + 1); },
    onPointerUp: () => { delete held.current[key]; force((x) => x + 1); },
    onPointerCancel: () => { delete held.current[key]; force((x) => x + 1); },
  });
  const on = (k: string) => (held.current[k] ? " on" : "");

  return (
    <div className="teleop">
      <div className="row"><b>EE 速度</b>
        <input type="range" min={0.2} max={2} step={0.1} value={speed} onChange={(e) => setSpeed(+e.target.value)} />
        <span className="mono">{speed.toFixed(1)}×</span></div>
      <div className="row"><b>グリッパー</b>
        <input type="range" min={0} max={1} step={0.02} value={grip} onChange={(e) => setGrip(+e.target.value)} className="grip" />
        <span className="mono">{grip < 0.15 ? "閉" : grip > 0.6 ? "開" : `${(grip * 100).toFixed(0)}%`}</span></div>
      {touch && (
        <div className="row"><b>ドラッグ</b>
          <button className={vert ? "primary" : ""} onClick={() => { inputState.vertical = !inputState.vertical; setVert(inputState.vertical); }}>
            {vert ? "上下 (Shift 相当)" : "水平"}</button>
          <span className="muted">「ドラッグ: EE」時、1 本指で EE がついてきます</span></div>
      )}
      <div className="dpad">
        <button className={"big" + on("fwd")} {...hold("fwd")}>▲ 前</button>
        <button className={"big" + on("left")} {...hold("left")}>◀ 左</button>
        <button className={"big" + on("right")} {...hold("right")}>右 ▶</button>
        <button className={"big" + on("back")} {...hold("back")}>▼ 後</button>
        <button className={"big" + on("up")} {...hold("up")}>⤒ 上</button>
        <button className={"big" + on("down")} {...hold("down")}>⤓ 下</button>
      </div>
      {(
        <div className="keymap">
          <div className="hint" style={{ marginBottom: 4 }}>{touch ? "ショートカットキー (iPad / iPhone は外付けキーボード接続時。画面のボタンでも操作できます)" : "ショートカットキー"}</div>
          {([
            ["移動", [["w", "W 前"], ["s", "S 後"], ["a", "A 左"], ["d", "D 右"], ["r", "R 上"], ["f", "F 下"]]],
            ["手首", [["q", "Q ロール+"], ["e", "E ロール−"], ["ArrowUp", "↑ ピッチ+"], ["ArrowDown", "↓ ピッチ−"], ["ArrowLeft", "← ヨー+"], ["ArrowRight", "→ ヨー−"]]],
            ["グリッパー", [["z", "Z 閉じる"], ["x", "X 開く"], [" ", "Space 開閉"]]],
          ] as [string, [string, string][]][]).map(([title, ks]) => (
            <div key={title} className="krow"><span className="ktitle">{title}</span>
              {ks.map(([k, label]) => <kbd key={k} className={pressed[k] ? "on" : ""}>{label}</kbd>)}</div>
          ))}
          <div className="hint">{touch ? "ビューポートは「ドラッグ: EE」で、1 本指ドラッグ=EE がついてくる (水平)。上の「ドラッグ」を「上下」にすると上下だけ動く (PC の Shift 相当)。" : "ビューポートは「ドラッグ: EE」で、左ドラッグ=水平移動 / Shift+ドラッグ=上下。"}</div>
        </div>
      )}
    </div>
  );
}
