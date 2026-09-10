from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field


@dataclass
class _CallbackStats:
    calls: int = 0
    processed: int = 0
    dropped: int = 0
    total_seconds: float = 0.0
    max_seconds: float = 0.0
    durations: deque[float] = field(default_factory=lambda: deque(maxlen=4096))


class CallbackPerformanceMonitor:
    """Bounded, low-overhead callback timing aggregated for periodic logging."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._started_monotonic = time.monotonic()
        self._stats: dict[str, _CallbackStats] = {}
        self._queue_depth_max = 0
        self._stale_sample_dropped = 0

    def record(
        self,
        name: str,
        duration_seconds: float,
        *,
        processed: bool = True,
        dropped: int = 0,
        queue_depth: int = 0,
    ) -> None:
        duration = max(0.0, float(duration_seconds))
        dropped = max(0, int(dropped))
        with self._lock:
            stats = self._stats.setdefault(name, _CallbackStats())
            stats.calls += 1
            stats.processed += int(bool(processed))
            stats.dropped += dropped
            stats.total_seconds += duration
            stats.max_seconds = max(stats.max_seconds, duration)
            stats.durations.append(duration)
            self._queue_depth_max = max(self._queue_depth_max, max(0, int(queue_depth)))
            self._stale_sample_dropped += dropped

    def snapshot_and_reset(self) -> dict:
        now = time.monotonic()
        with self._lock:
            elapsed = max(now - self._started_monotonic, 1e-6)
            raw_stats = self._stats
            queue_depth_max = self._queue_depth_max
            stale_sample_dropped = self._stale_sample_dropped
            self._stats = {}
            self._queue_depth_max = 0
            self._stale_sample_dropped = 0
            self._started_monotonic = now

        callbacks = {}
        for name, stats in raw_stats.items():
            ordered = sorted(stats.durations)
            p99_index = max(0, math.ceil(len(ordered) * 0.99) - 1) if ordered else 0
            callbacks[name] = {
                "calls": stats.calls,
                "processed": stats.processed,
                "dropped": stats.dropped,
                "rate_hz": round(stats.calls / elapsed, 2),
                "total_ms": round(stats.total_seconds * 1000.0, 3),
                "p99_ms": round(ordered[p99_index] * 1000.0, 3) if ordered else 0.0,
                "max_ms": round(stats.max_seconds * 1000.0, 3),
            }
        return {
            "window_seconds": round(elapsed, 3),
            "callbacks": callbacks,
            "callback_queue_depth": queue_depth_max,
            "stale_sample_dropped": stale_sample_dropped,
        }
