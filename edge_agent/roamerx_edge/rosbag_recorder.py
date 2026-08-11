from __future__ import annotations

import json
import subprocess
from pathlib import Path

from .protocol import ProtocolError


class RosbagRecorder:
    def __init__(self, script_path: str, timeout_seconds: int = 45) -> None:
        self.script_path = str(Path(script_path).expanduser())
        self.timeout_seconds = max(10, int(timeout_seconds))

    def start(self, label: str) -> dict:
        return self._run("start", label)

    def stop(self) -> dict:
        return self._run("stop")

    def status(self) -> dict:
        return self._run("status")

    def _run(self, action: str, label: str = "") -> dict:
        script = Path(self.script_path)
        if not script.is_file():
            raise ProtocolError("ROSBAG_UNAVAILABLE", f"rosbag script not found: {script}")
        args = [str(script), action]
        if label:
            args.append(label)
        try:
            result = subprocess.run(
                args,
                capture_output=True,
                text=True,
                timeout=self.timeout_seconds,
            )
        except subprocess.TimeoutExpired as exc:
            raise ProtocolError("ROSBAG_COMMAND_TIMEOUT", f"rosbag {action} timed out") from exc
        output = (result.stdout or "").strip()
        if result.returncode != 0:
            message = (result.stderr or output or f"rosbag {action} failed").strip()
            raise ProtocolError("ROSBAG_COMMAND_FAILED", message)
        try:
            payload = json.loads(output.splitlines()[-1]) if output else {}
        except (ValueError, IndexError) as exc:
            raise ProtocolError("ROSBAG_INVALID_STATUS", f"invalid rosbag {action} response") from exc
        payload["available"] = True
        return payload
