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


class RecoveryArbiter:
    """Allow at most one recovery owner to issue movement at a time."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._generation = 0
        self._lease: RecoveryLease | None = None

    def acquire(self, owner: str, reason: str = "") -> RecoveryLease | None:
        with self._lock:
            if self._lease is not None:
                return None
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

    def snapshot(self) -> dict:
        with self._lock:
            lease = self._lease
            return {
                "owner": lease.owner if lease else "NONE",
                "recovery_generation": lease.generation if lease else self._generation,
                "reason": lease.reason if lease else "",
                "started_at": lease.started_at if lease else None,
            }

