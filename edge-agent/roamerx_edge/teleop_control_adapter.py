from __future__ import annotations

import subprocess
import time
from pathlib import Path

from .config import TeleopControlConfig
from .protocol import ProtocolError


class TeleopControlAdapter:
    """Starts the low-level SDK velocity bridge without requiring localization."""

    def __init__(self, config: TeleopControlConfig) -> None:
        self.config = config
        self._last_ready_at = 0.0
        self._last_ready_payload: dict | None = None

    def ensure_ready(self) -> dict:
        now = time.monotonic()
        if self._last_ready_payload is not None and now - self._last_ready_at < 5.0:
            return {**self._last_ready_payload, "cached": True}
        running_payload = self._running_bridge_payload()
        if running_payload is not None:
            self._last_ready_at = now
            self._last_ready_payload = running_payload
            return running_payload
        payload = self._run("start", timeout_seconds=self.config.command_timeout_seconds)
        self._last_ready_at = time.monotonic()
        self._last_ready_payload = payload
        return payload

    def status(self) -> dict:
        return self._run("status", timeout_seconds=min(self.config.command_timeout_seconds, 10))

    def stop(self) -> dict:
        self._last_ready_at = 0.0
        self._last_ready_payload = None
        return self._run("stop", timeout_seconds=self.config.command_timeout_seconds)

    def _run(self, action: str, *, timeout_seconds: int) -> dict:
        script = Path(self.config.script_path).expanduser()
        if not script.exists():
            raise ProtocolError("TELEOP_BRIDGE_MISSING", f"teleop control script not found: {script}")
        completed = subprocess.run(
            [str(script), action],
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

    def _running_bridge_payload(self) -> dict | None:
        completed = subprocess.run(
            ["pgrep", "-af", "vel_cmd_udp_pub"],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=1,
        )
        if completed.returncode != 0:
            return None
        return {
            "action": "assume_ready",
            "returncode": 0,
            "stdout": completed.stdout[-4000:],
            "stderr": completed.stderr[-4000:],
        }
