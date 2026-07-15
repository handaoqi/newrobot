from __future__ import annotations

import subprocess
from pathlib import Path

from .config import NavigationStackConfig
from .protocol import ProtocolError


class NavigationStackAdapter:
    """Edge-side wrapper around the real robot navigation stack script."""

    def __init__(self, config: NavigationStackConfig) -> None:
        self.config = config

    def status(self) -> dict:
        return self._run("status", timeout_seconds=min(self.config.command_timeout_seconds, 20))

    def start(self, command: dict | None = None) -> dict:
        return self._run("start", timeout_seconds=max(self.config.command_timeout_seconds, 90))

    def restart(self, command: dict | None = None) -> dict:
        return self._run("restart", timeout_seconds=max(self.config.command_timeout_seconds, 90))

    def recover(self, command: dict | None = None) -> dict:
        status_payload = self.status()
        if status_payload.get("returncode") == 0 and self._looks_ready(status_payload.get("stdout", "")):
            status_payload["action"] = "recover"
            status_payload["recovery"] = "already_ready"
            return status_payload
        restart_payload = self.restart(command)
        restart_payload["action"] = "recover"
        restart_payload["recovery"] = "restart"
        restart_payload["precheck"] = status_payload
        return restart_payload

    def stop(self, command: dict | None = None) -> dict:
        return self._run("stop", timeout_seconds=self.config.command_timeout_seconds)

    def switch_map(self) -> dict:
        """Reload localization and Nav2 after map symlinks changed."""
        self._run("full-stop", timeout_seconds=self.config.command_timeout_seconds)
        return self._run("start", timeout_seconds=max(self.config.command_timeout_seconds, 90))

    def _looks_ready(self, stdout: str) -> bool:
        required = (
            "/planner_server",
            "/controller_server",
            "/bt_navigator",
            "active [3]",
            "/follow_waypoints",
            "/cmd_vel",
        )
        return all(token in stdout for token in required)

    def _run(self, action: str, *, timeout_seconds: int) -> dict:
        script = Path(self.config.script_path).expanduser()
        if not script.exists():
            raise ProtocolError("NAV_SCRIPT_MISSING", f"navigation script not found: {script}")
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
            "stdout": completed.stdout[-6000:],
            "stderr": completed.stderr[-6000:],
        }
        if completed.returncode != 0:
            raise ProtocolError("NAV_COMMAND_FAILED", payload["stderr"] or payload["stdout"])
        return payload
