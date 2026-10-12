"""WebSocket 配信用イベントバス。session_id + 単調増加 seq を付け、切断→再接続で状態を再同期できるようにする。

- 高頻度の `state` は再送しない (再接続時は最新スナップショットを送る)。
- 重要イベント (エピソード終了・ジョブ状態・エラー) はリングバッファに残し、`since=<seq>` で取りこぼし分を再送する。
  バッファから溢れて欠落した場合は resync_required を返し、クライアントが REST で状態を取り直す。
"""
from __future__ import annotations

import asyncio
import threading
import uuid
from collections import deque
from typing import Any

DURABLE = {"episode", "job", "error", "mode", "inference"}


class EventBus:
    def __init__(self, ring: int = 500):
        self.session_id = uuid.uuid4().hex[:12]
        self._seq = 0
        self._ring: deque[dict] = deque(maxlen=ring)
        self._lock = threading.Lock()
        self._subs: set[asyncio.Queue] = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self.latest_state: dict[str, Any] = {}

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    def publish(self, ev: dict[str, Any]) -> dict[str, Any]:
        """任意スレッドから呼べる。"""
        with self._lock:
            self._seq += 1
            ev = {"seq": self._seq, "session_id": self.session_id, **ev}
            if ev["type"] == "state":
                self.latest_state = ev
            if ev["type"] in DURABLE:
                self._ring.append(ev)
            subs = list(self._subs)
        if self._loop is not None:
            for q in subs:
                self._loop.call_soon_threadsafe(self._offer, q, ev)
        return ev

    @staticmethod
    def _offer(q: asyncio.Queue, ev: dict) -> None:
        try:
            q.put_nowait(ev)
        except asyncio.QueueFull:
            # 遅いクライアントは古い state から捨てる (重要イベントは落とさない)
            try:
                while True:
                    old = q.get_nowait()
                    if old["type"] != "state":
                        q.put_nowait(old)
                        break
            except asyncio.QueueEmpty:
                pass
            try:
                q.put_nowait(ev)
            except asyncio.QueueFull:
                pass

    def subscribe(self, since: int | None) -> tuple[asyncio.Queue, list[dict], bool]:
        q: asyncio.Queue = asyncio.Queue(maxsize=200)
        with self._lock:
            self._subs.add(q)
            replay, resync = [], False
            if since is not None:
                oldest = self._ring[0]["seq"] if self._ring else self._seq + 1
                if since + 1 < oldest and self._seq > since:
                    resync = True   # 欠落あり
                replay = [e for e in self._ring if e["seq"] > since]
            return q, replay, resync

    def unsubscribe(self, q: asyncio.Queue) -> None:
        with self._lock:
            self._subs.discard(q)

    @property
    def seq(self) -> int:
        return self._seq
