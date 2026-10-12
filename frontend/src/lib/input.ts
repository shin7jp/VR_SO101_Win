// ビューポートの EE ドラッグ中かどうか。操作パネルの「入力なし (速度 0)」ハートビートが、
// ドラッグの速度指令を 40ms ごとに 0 で上書きして打ち消してしまうのを避けるための共有フラグ。
export const inputState = { dragging: false, vertical: false };   // vertical: ドラッグで上下だけ動かす (Shift の代わり。タッチ用)
