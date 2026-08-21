from __future__ import annotations

import subprocess
import time
import threading

from .config import TeleopControlConfig
from .protocol import ProtocolError


class TeleopControlAdapter:
    """Own the single virtual-remote bridge through systemd."""

    SERVICE_NAME = "roamerx-teleop-bridge.service"

    def __init__(self, config: TeleopControlConfig) -> None:
        self.config = config
        self._last_ready_at = 0.0
        self._last_ready_payload: dict | None = None
        self._ready_lock = threading.Lock()

    def ensure_ready(self) -> dict:
        # MQTT commands can arrive close together. Only one caller may decide
        # whether a bridge needs starting, otherwise duplicate SDK sockets can
        # race to bind the same local UDP port.
        with self._ready_lock:
            now = time.monotonic()
            if self._last_ready_payload is not None and now - self._last_ready_at < 5.0:
                return {**self._last_ready_payload, "cached": True}
            if self._is_active():
                running_payload = {"action": "already_active", "returncode": 0}
                self._last_ready_at = now
                self._last_ready_payload = running_payload
                return running_payload
            payload = self._systemctl("start", timeout_seconds=self.config.command_timeout_seconds)
            if not self._is_active():
                raise ProtocolError("TELEOP_BRIDGE_FAILED", "remote-control bridge did not become active")
            self._last_ready_at = time.monotonic()
            self._last_ready_payload = payload
            return payload

    def recent_ready_status(self) -> dict:
        """Return cached state without blocking a held velocity packet.

        The bridge is a resident systemd service.  A 150 ms held control stream
        must not serialize behind a subprocess health check, especially the
        release/stop packet.
        """
        payload = self._last_ready_payload
        if payload is not None:
            return {**payload, "cached": True}
        return {"action": "readiness_deferred", "cached": False}

    def status(self) -> dict:
        return self._systemctl("status", timeout_seconds=min(self.config.command_timeout_seconds, 10))

    def _systemctl(self, action: str, *, timeout_seconds: int) -> dict:
        completed = subprocess.run(
            ["sudo", "systemctl", action, self.SERVICE_NAME],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_seconds,
        )
        payload = {
            "action": action,
            "returncode": completed.returncode,
            "stdout": completed.stdout[-4000:],
            "stderr": completed.stderr[-4000:],
        }
        if completed.returncode != 0:
            raise ProtocolError("TELEOP_BRIDGE_FAILED", payload["stderr"] or payload["stdout"])
        return payload

    def _is_active(self) -> bool:
        process = subprocess.run(
            ["systemctl", "is-active", "--quiet", self.SERVICE_NAME],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=1,
        )
        return process.returncode == 0
