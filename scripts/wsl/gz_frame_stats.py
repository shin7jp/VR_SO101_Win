"""Gazebo カメラ 1 フレームの画素統計を出す (gate0.sh から使う)。

  python3 gz_frame_stats.py /gate0/camera [--min-std 5]

フレーム数・解像度が正常でも、GPU 経路が壊れていると全画素 0 やノイズ画像になる
(gl_env.sh 参照)。全画素 0 は std=0 なので検出できる。ノイズは平均・標準偏差だけでは
正常画像と区別が付かないので、隣り合う画素の差が大きすぎる画像も不合格にする。
"""
import re
import subprocess
import sys

import numpy as np

_ESC = {0x6E: 10, 0x72: 13, 0x74: 9, 0x22: 34, 0x27: 39, 0x5C: 92}


def _unescape(s: bytes) -> bytes:
    """protobuf テキスト形式のバイト列エスケープ (\\ooo と \\n など) を戻す。"""
    out, i = bytearray(), 0
    while i < len(s):
        c = s[i]
        if c != 0x5C:
            out.append(c)
            i += 1
            continue
        n = s[i + 1]
        if 0x30 <= n <= 0x37:
            j = i + 1
            while j < len(s) and j < i + 4 and 0x30 <= s[j] <= 0x37:
                j += 1
            out.append(int(s[i + 1:j], 8) & 255)
            i = j
        else:
            out.append(_ESC.get(n, n))
            i += 2
    return bytes(out)


def main() -> int:
    topic = sys.argv[1]
    min_std = float(sys.argv[sys.argv.index("--min-std") + 1]) if "--min-std" in sys.argv else 5.0
    txt = subprocess.run(["ign", "topic", "-e", "-t", topic, "-n", "1"], capture_output=True, timeout=30).stdout
    w = int(re.search(rb"^width: (\d+)", txt, re.M).group(1))
    h = int(re.search(rb"^height: (\d+)", txt, re.M).group(1))
    data = _unescape(re.search(rb'data: "(.*)"\n', txt, re.S).group(1))
    a = np.frombuffer(data[: w * h * 3], np.uint8).reshape(h, w, 3).astype(np.float32)
    std = float(a.std())
    # 隣り合う画素の差の平均。滑らかなシーンなら小さく、行が崩れたノイズ画像では大きい
    roughness = float(np.abs(np.diff(a, axis=1)).mean())
    print(f"pixels: mean={a.mean():.1f} std={std:.1f} roughness={roughness:.1f}")
    if std < min_std:
        print("[NG] 画像がほぼ一様 (全画素 0 など)。GL の経路が壊れている可能性: scripts/wsl/gl_env.sh")
        return 1
    if roughness > 20:
        print("[NG] 画像がノイズ状。GL の経路が壊れている可能性: scripts/wsl/gl_env.sh")
        return 1
    print("[OK] 画素の中身は正常")
    return 0


if __name__ == "__main__":
    sys.exit(main())
