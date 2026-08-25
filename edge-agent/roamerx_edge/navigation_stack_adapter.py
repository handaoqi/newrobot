from __future__ import annotations

import shlex
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

    def restart_localization(self) -> dict:
        """Restart only localization so a paused task can reseed it safely."""
        return self._run("restart-localization", timeout_seconds=max(self.config.command_timeout_seconds, 60))

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

    def reload_map(self, pcd_path: str, yaml_path: str) -> dict:
        """Atomically reload the active localization and Nav2 map assets.

        Map activation changes symlinks on disk, while both localization and
        Nav2 keep their maps in memory.  Reload both services before reporting
        the activation as usable so the next initial-pose command targets the
        selected map instead of the previously active one.
        """
        script = (
            "source /opt/ros/humble/setup.bash && "
            "source /home/dogrobot/robot/install/setup.bash && "
            "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-24} "
            "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}; "
            "timeout 25 ros2 service call /load_map_service "
            "robots_dog_msgs/srv/LoadMap "
            + shlex.quote("{pcd_path: '" + pcd_path + "'}")
            + " && timeout 25 ros2 service call /map_server/load_map "
            "nav2_msgs/srv/LoadMap "
            + shlex.quote("{map_url: '" + yaml_path + "'}")
        )
        completed = subprocess.run(
            ["bash", "-lc", script],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=max(self.config.command_timeout_seconds, 60),
        )
        payload = {
            "action": "reload_map",
            "returncode": completed.returncode,
            "stdout": completed.stdout[-6000:],
            "stderr": completed.stderr[-6000:],
            "pcd_path": pcd_path,
            "yaml_path": yaml_path,
        }
        if completed.returncode != 0:
            raise ProtocolError("MAP_RELOAD_FAILED", payload["stderr"] or payload["stdout"])
        return payload

    def reload_map_if_running(self, pcd_path: str, yaml_path: str) -> dict:
        """Reload live consumers, or defer until the next navigation start.

        Mapping deliberately full-stops localization and Nav2.  An automatic
        activation command can arrive immediately after upload, before those
        services exist again; calling ROS load-map services in that window is
        an expected unavailable state rather than an activation failure.
        """
        status_payload = self.status()
        stdout = str(status_payload.get("stdout") or "")
        localization_running = any(token in stdout for token in (
            "localization localization.launch.py",
            "localization_node",
        ))
        navigation_running = any(token in stdout for token in (
            "robot_navigo navigation_bringup.launch.py",
            "navigo_container",
        ))
        if not (localization_running and navigation_running):
            return {
                "action": "reload_map",
                "returncode": 0,
                "deferred": True,
                "reason": "map_consumers_inactive",
                "pcd_path": pcd_path,
                "yaml_path": yaml_path,
                "precheck": status_payload,
            }
        return self.reload_map(pcd_path, yaml_path)

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
