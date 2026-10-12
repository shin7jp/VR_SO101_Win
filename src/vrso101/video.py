"""動画の書き出し・検証 (PyAV)。

- 収録フレームは FFV1(可逆) で保存する。LeRobot 変換時にもう一度 H.264 へ非可逆圧縮されるので、
  ここで劣化させると学習画像が二重に劣化し、推論時の生フレームとの分布差が広がる。
- 閲覧・ダウンロード用は H.264 yuv420p + faststart (ブラウザ/iOS Safari で再生できる唯一の無難な形式。
  OpenCV の mp4v や AV1 は再生できない環境がある)。
"""
from __future__ import annotations

from fractions import Fraction
from pathlib import Path

import av
import numpy as np


class VideoWriter:
    def __init__(self, path: str | Path, fps: float, width: int, height: int, *, codec: str = "libx264",
                 crf: int = 18, container_opts: dict | None = None):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.width, self.height = width, height
        self._frames = 0
        self._closed = False
        self._c = av.open(str(self.path), "w", options=container_opts or {})
        rate = Fraction(fps).limit_denominator(1000)
        self._s = self._c.add_stream(codec, rate=rate)
        self._s.width, self._s.height = width, height
        if codec == "ffv1":
            self._s.pix_fmt = "bgr0"          # 可逆: RGB を YUV 変換せずそのまま
        else:
            self._s.pix_fmt = "yuv420p"       # H.264 でブラウザ再生できる唯一の無難な画素形式
            self._s.options = {"crf": str(crf), "preset": "veryfast"}

    def write(self, rgb: np.ndarray) -> None:
        if rgb.shape != (self.height, self.width, 3) or rgb.dtype != np.uint8:
            raise ValueError(f"フレーム形状/型が不正: {rgb.shape} {rgb.dtype} (期待 {(self.height, self.width, 3)} uint8)")
        fmt = "rgb24"
        frame = av.VideoFrame.from_ndarray(np.ascontiguousarray(rgb), format=fmt)
        for pkt in self._s.encode(frame):
            self._c.mux(pkt)
        self._frames += 1

    @property
    def frames(self) -> int:
        return self._frames

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for pkt in self._s.encode(None):
            self._c.mux(pkt)
        self._c.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def write_mp4(path: str | Path, frames, fps: float, crf: int = 20) -> int:
    """H.264/faststart の MP4 を書く。戻り値 = フレーム数。"""
    it = iter(frames)
    first = next(it)
    h, w = first.shape[:2]
    with VideoWriter(path, fps, w, h, codec="libx264", crf=crf, container_opts={"movflags": "+faststart"}) as vw:
        vw.write(first)
        for f in it:
            vw.write(f)
    return vw.frames


def read_frames(path: str | Path):
    """全フレームを RGB uint8 で順に yield する。"""
    with av.open(str(path)) as c:
        for fr in c.decode(video=0):
            yield fr.to_ndarray(format="rgb24")


def probe_video(path: str | Path) -> dict:
    """実際にデコードしてフレーム数・解像度・fps・コーデックを返す (ヘッダの自己申告を信用しない)。
    壊れたファイルは例外にする: 保存完了の確認 (UI で『保存済み』と出す条件) に使う。"""
    p = Path(path)
    if not p.exists() or p.stat().st_size == 0:
        raise FileNotFoundError(f"動画がありません/空です: {p}")
    n = 0
    with av.open(str(p)) as c:
        s = c.streams.video[0]
        codec, fps = s.codec_context.name, float(s.average_rate or 0)
        w, h = s.codec_context.width, s.codec_context.height
        for _ in c.decode(video=0):
            n += 1
    if n == 0:
        raise ValueError(f"動画にフレームがありません: {p}")
    return {"frames": n, "width": w, "height": h, "fps": fps, "codec": codec, "bytes": p.stat().st_size}
