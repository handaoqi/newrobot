from __future__ import annotations

import subprocess
from pathlib import Path

from .config import SensorControlConfig
from .protocol import ProtocolError


class SensorControlAdapter:
    def __init__(self, config: SensorControlConfig) -> None:
        self.config = config

    def restart(self, sensor: str) -> dict:
        sensor = str(sensor or "").strip().lower()
        if sensor in {"lidar", "imu", "lidar_imu"}:
            canonical = "lidar_imu"
            script = self.config.lidar_restart_script
        elif sensor == "rtk":
            canonical = "rtk"
            script = self.config.rtk_restart_script
        else:
            raise ProtocolError("SENSOR_NOT_RESTARTABLE", f"unsupported sensor: {sensor}")

        path = Path(script)
        if not path.is_file():
            raise ProtocolError("SENSOR_CONTROL_UNAVAILABLE", f"sensor script not found: {path}")
        try:
            completed = subprocess.run(
                [str(path)],
                capture_output=True,
                text=True,
                timeout=max(5, int(self.config.command_timeout_seconds)),
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise ProtocolError("SENSOR_RESTART_TIMEOUT", f"{canonical} restart timed out") from exc
        if completed.returncode != 0:
            message = (completed.stderr or completed.stdout or "sensor restart failed").strip()
            raise ProtocolError("SENSOR_RESTART_FAILED", message[-1000:])
        return {
            "sensor": canonical,
            "restarted": True,
            "message": (completed.stdout or "").strip()[-1000:],
        }
