// WebXR 対応状況を実行時に検出する。非対応なら「VR 開始」ボタンは無効にし、理由を出す (有効のまま放置しない)。
export type XrSupport = { supported: boolean; reason: string };

export async function detectXr(): Promise<XrSupport> {
  if (!window.isSecureContext) return { supported: false, reason: "HTTPS ではないため WebXR は使えません (https:// でアクセスしてください)" };
  const xr = (navigator as any).xr;
  if (!xr) return { supported: false, reason: "このブラウザは WebXR 非対応です (iOS Safari など)。PC/タッチ操作を使ってください" };
  try {
    const ok = await xr.isSessionSupported("immersive-vr");
    return ok ? { supported: true, reason: "" } : { supported: false, reason: "VR ヘッドセットが接続されていないか、immersive-vr に対応していません" };
  } catch (e: any) {
    return { supported: false, reason: "WebXR の確認に失敗: " + (e?.message ?? e) };
  }
}

export const isTouchDevice = () => "ontouchstart" in window || navigator.maxTouchPoints > 0;
export const deviceClass = (): "phone" | "tablet" | "desktop" => {
  const w = Math.min(window.innerWidth, window.innerHeight);
  if (isTouchDevice() && w < 500) return "phone";
  if (isTouchDevice()) return "tablet";
  return "desktop";
};
