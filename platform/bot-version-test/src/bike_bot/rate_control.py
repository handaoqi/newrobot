from __future__ import annotations

import math


def selected_inference_rate_hz(
    normal_rate_hz: float,
    person_follow_rate_hz: float,
    person_follow_enabled: bool,
) -> float:
    """Return the active inference rate; non-positive values disable throttling."""
    selected = person_follow_rate_hz if person_follow_enabled else normal_rate_hz
    return max(0.0, float(selected))


class InferenceRateLimiter:
    """Monotonic start-to-start limiter that never tries to catch up old frames."""

    def __init__(self) -> None:
        self._next_due_at = 0.0
        self._rate_hz = 0.0

    def delay_seconds(self, now: float) -> float:
        return max(0.0, self._next_due_at - now)

    def mark_started(self, started_at: float, rate_hz: float) -> None:
        rate_hz = max(0.0, float(rate_hz))
        if rate_hz <= 0.0:
            self._next_due_at = started_at
            self._rate_hz = 0.0
            return
        interval = 1.0 / rate_hz
        if self._next_due_at <= 0.0 or rate_hz != self._rate_hz:
            self._next_due_at = started_at + interval
        else:
            self._next_due_at += interval
            if self._next_due_at <= started_at:
                skipped = math.floor((started_at - self._next_due_at) / interval) + 1
                self._next_due_at += skipped * interval
        self._rate_hz = rate_hz

    def reset(self) -> None:
        self._next_due_at = 0.0
        self._rate_hz = 0.0
