from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass, field
from typing import Any


@dataclass
class SimChannel:
    latency_ms: float = 0.0
    drop_rate: float = 0.0
    rng: random.Random = field(default_factory=random.Random)
    sent: int = 0
    recv: int = 0
    tx_bytes: int = 0
    rx_bytes: int = 0
    _queue: deque[tuple[float, bytes, Any]] = field(default_factory=deque)

    def send(self, now_sec: float, payload: bytes, meta: Any = None) -> bool:
        self.sent += 1
        self.tx_bytes += len(payload)
        if self.rng.random() < self.drop_rate:
            return False
        deliver_at = now_sec + self.latency_ms / 1000.0
        self._queue.append((deliver_at, payload, meta))
        return True

    def receive_ready(self, now_sec: float) -> list[tuple[bytes, Any]]:
        ready: list[tuple[bytes, Any]] = []
        while self._queue and self._queue[0][0] <= now_sec:
            _, payload, meta = self._queue.popleft()
            self.recv += 1
            self.rx_bytes += len(payload)
            ready.append((payload, meta))
        return ready
