"""Single-owner arbitration for navigation recovery actions."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class RecoveryLease:
    owner: str
    generation: int
    reason: str
    started_at: float


@dataclass
class RecoveryBudget:
    """Finite budget that forces SAFE_HOLD when exhausted."""

    max_attempts: int = 6
    max_duration_seconds: float = 180.0
    max_distance_m: float = 8.0
    attempts: int = 0
    started_at: float | None = None
    distance_m: float = 0.0

    def begin_episode(self) -> None:
        if self.started_at is None:
            self.started_at = time.monotonic()

    def record_attempt(self, distance_m: float = 0.0) -> None:
        self.begin_episode()
        self.attempts += 1
        self.distance_m += max(0.0, float(distance_m))

    def exhausted(self) -> bool:
        if self.attempts >= self.max_attempts:
            return True
        if self.started_at is not None and (time.monotonic() - self.started_at) >= self.max_duration_seconds:
            return True
        if self.distance_m >= self.max_distance_m:
            return True
        return False

    def reset(self) -> None:
        self.attempts = 0
        self.started_at = None
        self.distance_m = 0.0

    def snapshot(self) -> dict:
        elapsed = None
        if self.started_at is not None:
            elapsed = time.monotonic() - self.started_at
        return {
            "attempts": self.attempts,
            "max_attempts": self.max_attempts,
            "distance_m": round(self.distance_m, 3),
            "max_distance_m": self.max_distance_m,
            "elapsed_seconds": None if elapsed is None else round(elapsed, 2),
            "max_duration_seconds": self.max_duration_seconds,
            "exhausted": self.exhausted(),
        }


class RecoveryArbiter:
    """Allow at most one recovery owner to issue movement at a time."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._generation = 0
        self._lease: RecoveryLease | None = None
        self._budget = RecoveryBudget()

    def acquire(self, owner: str, reason: str = "", *, distance_m: float = 0.0) -> RecoveryLease | None:
        with self._lock:
            if self._lease is not None:
                return None
            if self._budget.exhausted():
                return None
            self._budget.record_attempt(distance_m)
            self._generation += 1
            self._lease = RecoveryLease(owner, self._generation, reason, time.monotonic())
            return self._lease

    def release(self, lease: RecoveryLease | None) -> bool:
        if lease is None:
            return False
        with self._lock:
            if self._lease != lease:
                return False
            self._lease = None
            return True

    def force_release(self) -> None:
        with self._lock:
            self._lease = None

    def budget_exhausted(self) -> bool:
        with self._lock:
            return self._budget.exhausted()

    def reset_budget(self) -> None:
        with self._lock:
            self._budget.reset()

    def snapshot(self) -> dict:
        with self._lock:
            lease = self._lease
            return {
                "owner": lease.owner if lease else "NONE",
                "recovery_generation": lease.generation if lease else self._generation,
                "reason": lease.reason if lease else "",
                "started_at": lease.started_at if lease else None,
                "budget": self._budget.snapshot(),
            }
