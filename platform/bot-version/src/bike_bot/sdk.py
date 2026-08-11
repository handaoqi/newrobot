from __future__ import annotations

import logging
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Any, Callable

from .config import AppConfig

LOGGER = logging.getLogger(__name__)


@dataclass
class RobotSdkSample:
    connected: bool
    battery_level: int | None
    signal_strength: int
    latency_ms: float | None


class RobotSdkClient:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self._lock = Lock()
        self._mode_lock = Lock()
        self._sdk_app: Any | None = None
        self._remote_takeover_active = False
        self._recent_checks: deque[tuple[bool, float | None]] = deque(maxlen=10)

    def execute(self, handler: Callable[[Any], Any]) -> Any:
        if not self.is_remote_takeover_active:
            raise RuntimeError("remote takeover is not active")
        app = self._get_sdk_app()
        with self._lock:
            return handler(app)

    def sample_status(self) -> RobotSdkSample:
        if (
            self.config.control.dry_run
            or not self.config.control.sdk_enabled
            or not self.is_remote_takeover_active
        ):
            return RobotSdkSample(
                connected=True,
                battery_level=None,
                signal_strength=self.config.runtime.signal_strength,
                latency_ms=None,
            )

        started_at = time.perf_counter()
        connected = False
        battery_level: int | None = None
        latency_ms: float | None = None
        try:
            app = self._get_sdk_app()
            with self._lock:
                connected = bool(app.checkConnect())
                latency_ms = (time.perf_counter() - started_at) * 1000
                if connected:
                    battery_level = self._clamp_percent(int(app.getBatteryPower()))
        except Exception:
            LOGGER.exception("robot sdk status sample failed")

        self._recent_checks.append((connected, latency_ms))
        return RobotSdkSample(
            connected=connected,
            battery_level=battery_level,
            signal_strength=self._estimate_signal_strength(),
            latency_ms=latency_ms,
        )

    @property
    def is_remote_takeover_active(self) -> bool:
        with self._mode_lock:
            return self._remote_takeover_active

    def begin_remote_takeover(self) -> None:
        if self.config.control.dry_run:
            with self._mode_lock:
                self._remote_takeover_active = True
            return
        try:
            self._get_sdk_app()
        except Exception:
            with self._mode_lock:
                self._remote_takeover_active = False
            raise
        with self._mode_lock:
            self._remote_takeover_active = True

    def end_remote_takeover(self, *, passive: bool = True) -> None:
        app = self._sdk_app
        if app is not None:
            with self._lock:
                try:
                    if passive and hasattr(app, "passive"):
                        app.passive()
                    elif hasattr(app, "move"):
                        app.move(0.0, 0.0, 0.0)
                except Exception:
                    LOGGER.exception("robot sdk takeover release stop command failed")
                self._release_sdk_app(app)
        with self._mode_lock:
            self._remote_takeover_active = False
        self._recent_checks.clear()

    def _get_sdk_app(self):
        if self._sdk_app is not None:
            return self._sdk_app
        if not self.config.control.sdk_enabled:
            raise RuntimeError("robot SDK is disabled")

        if self.config.control.sdk_lib_path:
            sdk_path = str(Path(self.config.control.sdk_lib_path).resolve())
            if sdk_path not in sys.path:
                sys.path.insert(0, sdk_path)

        import mc_sdk_zsl_1_py  # type: ignore

        app = mc_sdk_zsl_1_py.HighLevel()
        app.initRobot(
            self.config.control.local_ip,
            self.config.control.local_port,
            self.config.control.robot_ip,
        )
        self._sdk_app = app
        return app

    def _release_sdk_app(self, app: Any) -> None:
        release_method_names = (
            "release",
            "close",
            "disconnect",
            "deinitRobot",
            "deInitRobot",
            "destroy",
        )
        for method_name in release_method_names:
            method = getattr(app, method_name, None)
            if callable(method):
                try:
                    method()
                    LOGGER.info("robot sdk released via %s", method_name)
                except Exception:
                    LOGGER.exception("robot sdk release method failed method=%s", method_name)
                break
        else:
            LOGGER.info("robot sdk has no known release method; cleared local SDK reference")
        self._sdk_app = None

    def _estimate_signal_strength(self) -> int:
        if not self._recent_checks:
            return self.config.runtime.signal_strength

        success_rate = sum(1 for connected, _latency_ms in self._recent_checks if connected) / len(self._recent_checks)
        successful_latencies = [
            latency_ms
            for connected, latency_ms in self._recent_checks
            if connected and latency_ms is not None
        ]
        if successful_latencies:
            avg_latency_ms = sum(successful_latencies) / len(successful_latencies)
            latency_score = max(0.0, min(100.0, 100.0 - ((avg_latency_ms - 50.0) / 950.0) * 100.0))
        else:
            latency_score = 0.0

        return self._clamp_percent(round(success_rate * 70 + latency_score * 0.30))

    @staticmethod
    def _clamp_percent(value: int) -> int:
        return max(0, min(100, value))
