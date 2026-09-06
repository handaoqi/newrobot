from __future__ import annotations

import threading
import time
import json
from collections import deque
from datetime import datetime, timezone
from typing import Callable


LEVELS = {"DEBUG", "INFO", "WARNING", "ERROR"}
MODULES = {
    "localization", "navigation", "avoidance", "relocalization",
    "waypoint", "planner", "boundary", "system",
}
SENSITIVE_KEYS = {"password", "secret", "token", "authorization", "credential", "private_key"}


def _clean(value, depth=0):
    if depth > 5:
        return "<truncated>"
    if isinstance(value, dict):
        return {
            str(key)[:96]: "<redacted>" if any(secret in str(key).lower() for secret in SENSITIVE_KEYS) else _clean(item, depth + 1)
            for key, item in list(value.items())[:100]
        }
    if isinstance(value, list):
        return [_clean(item, depth + 1) for item in value[:50]]
    if isinstance(value, str):
        return value[:2000]
    if value is None or isinstance(value, (bool, int, float)):
        return value
    return str(value)[:500]


def _bounded_data(value) -> dict:
    cleaned = _clean(value if isinstance(value, dict) else {})
    encoded = json.dumps(cleaned, ensure_ascii=False, default=str).encode("utf-8")
    return cleaned if len(encoded) <= 16 * 1024 else {"truncated": True, "original_bytes": len(encoded)}


class StructuredLogEmitter:
    """Small, bounded structured-log buffer independent from Python logging."""

    def __init__(self, publish_batch: Callable[[list[dict], bool], None], *, batch_size: int = 50) -> None:
        self.publish_batch = publish_batch
        self.batch_size = max(1, min(100, int(batch_size)))
        self._lock = threading.RLock()
        self._entries: list[dict] = []
        self._debug_modules: set[str] = set()
        self._debug_expires_monotonic = 0.0
        self._sample_interval = 1.0
        self._last_debug: dict[tuple[str, str], float] = {}
        self._debug_window = deque()
        self._last_rate_warning = 0.0

    def configure_debug(self, *, enabled: bool, modules=None, sample_hz: float = 1.0, expires_at=None) -> dict:
        with self._lock:
            if not enabled:
                self._debug_modules.clear()
                self._debug_expires_monotonic = 0.0
                self._last_debug.clear()
                return {"enabled": False}
            modules = set(modules or MODULES) & MODULES
            sample_hz = max(0.1, min(5.0, float(sample_hz)))
            duration = 900.0
            if expires_at:
                try:
                    expiry = datetime.fromisoformat(str(expires_at).replace("Z", "+00:00"))
                    duration = max(0.0, min(1800.0, (expiry - datetime.now(timezone.utc)).total_seconds()))
                except (TypeError, ValueError):
                    duration = 900.0
            self._debug_modules = modules
            self._debug_expires_monotonic = time.monotonic() + duration
            self._sample_interval = 1.0 / sample_hz
            self._last_debug.clear()
            return {"enabled": True, "modules": sorted(modules), "sample_hz": sample_hz, "expires_in_seconds": round(duration)}

    def debug_enabled(self, module: str) -> bool:
        with self._lock:
            if time.monotonic() >= self._debug_expires_monotonic:
                self._debug_modules.clear()
                return False
            return module in self._debug_modules

    def emit(self, level: str, module: str, event_code: str, message: str, *, data=None, **context) -> bool:
        level = str(level).upper()
        if level not in LEVELS or module not in MODULES:
            return False
        now = time.monotonic()
        if level == "DEBUG":
            if not self.debug_enabled(module):
                return False
            key = (module, event_code)
            with self._lock:
                if now - self._last_debug.get(key, 0.0) < self._sample_interval:
                    return False
                while self._debug_window and now - self._debug_window[0] >= 1.0:
                    self._debug_window.popleft()
                if len(self._debug_window) >= 30:
                    if now - self._last_rate_warning >= 60.0:
                        self._last_rate_warning = now
                        self._entries.append({
                            "occurred_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                            "level": "WARNING", "module": "system",
                            "event_code": "system.debug_rate_limited",
                            "message": "DEBUG 日志超过30条/秒，已限流",
                            "source": "edge", "data": {"limit_per_second": 30},
                        })
                    return False
                self._debug_window.append(now)
                self._last_debug[key] = now
        entry = {
            "occurred_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "level": level,
            "module": module,
            "event_code": str(event_code)[:96],
            "message": str(message)[:500],
            "source": str(context.pop("source", "edge"))[:64],
            "data": _bounded_data(data),
            **{key: value for key, value in context.items() if value is not None},
        }
        urgent = level in {"WARNING", "ERROR"}
        batch = None
        with self._lock:
            self._entries.append(entry)
            if urgent or len(self._entries) >= self.batch_size:
                batch = self._entries
                self._entries = []
        if batch:
            self.publish_batch(batch, urgent)
        return True

    def flush(self) -> int:
        with self._lock:
            batch = self._entries
            self._entries = []
        if batch:
            self.publish_batch(batch, any(item["level"] in {"WARNING", "ERROR"} for item in batch))
        return len(batch)
