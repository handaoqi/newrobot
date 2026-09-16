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
        status_payload = self.status()
        if status_payload.get("returncode") == 0 and self._looks_ready(status_payload.get("stdout", "")):
            status_payload["action"] = "start"
            status_payload["recovery"] = "already_ready"
            return status_payload
        return self._run("start", timeout_seconds=max(self.config.command_timeout_seconds, 180))

    def prepare(self, command: dict | None = None) -> dict:
        """Start localization and Nav2's map/safety group without controllers."""
        status_payload = self.status()
        if status_payload.get("returncode") == 0 and self._looks_prepared(status_payload.get("stdout", "")):
            status_payload["action"] = "prepare"
            status_payload["recovery"] = "already_prepared"
            return status_payload
        return self._run("prepare", timeout_seconds=max(self.config.command_timeout_seconds, 90))

    def activate_execution(self) -> dict:
        status_payload = self.status()
        if status_payload.get("returncode") == 0 and self._looks_ready(status_payload.get("stdout", "")):
            status_payload["action"] = "activate_execution"
            status_payload["recovery"] = "already_active"
            return status_payload
        return self._run("activate-execution", timeout_seconds=max(self.config.command_timeout_seconds, 45))

    def deactivate_execution(self) -> dict:
        return self._run("deactivate-execution", timeout_seconds=max(self.config.command_timeout_seconds, 30))

    def reconcile(self) -> dict:
        """Report the resident stack's lifecycle state without changing it."""
        status_payload = self.status()
        stdout = str(status_payload.get("stdout") or "")
        return {
            **status_payload,
            "action": "reconcile",
            "stack_prepared": self._looks_prepared(stdout),
            "execution_active": self._looks_ready(stdout),
        }

    def restart(self, command: dict | None = None) -> dict:
        return self._run("restart", timeout_seconds=max(self.config.command_timeout_seconds, 180))

    def restart_localization(self) -> dict:
        """Restart only localization so a paused task can reseed it safely."""
        return self._run("restart-localization", timeout_seconds=max(self.config.command_timeout_seconds, 90))

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

    def shutdown(self) -> dict:
        """Explicit lifecycle teardown alias for callers that require it."""
        return self.stop({"reason": "shutdown"})

    def switch_map(self) -> dict:
        """Reload localization and Nav2 after map symlinks changed."""
        self._run("full-stop", timeout_seconds=self.config.command_timeout_seconds)
        return self._run("start", timeout_seconds=max(self.config.command_timeout_seconds, 90))

    def reload_map(self, pcd_path: str, yaml_path: str) -> dict:
        """Reload the active localization and Nav2 map assets as one operation.

        Map activation changes symlinks on disk, while both localization and
        Nav2 keep their maps in memory.  Reload both services before reporting
        the activation as usable so the next initial-pose command targets the
        selected map instead of the previously active one.
        """
        localization = self.reload_localization_map(pcd_path)
        navigation = self.reload_navigation_map(yaml_path)
        return {
            "action": "reload_map",
            "returncode": 0,
            "deferred": False,
            "pcd_path": pcd_path,
            "yaml_path": yaml_path,
            "localization_reloaded": True,
            "navigation_reloaded": True,
            "localization": localization,
            "navigation": navigation,
        }

    def reload_localization_map(self, pcd_path: str) -> dict:
        """Reload and validate the PCD held by a running localization node."""
        script = (
            "source /opt/ros/humble/setup.bash && "
            "source /home/dogrobot/robot/install/setup.bash && "
            "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-24} "
            "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}; "
            "/home/dogrobot/robot/script/robot/load_localization_map.py "
            + shlex.quote(pcd_path)
            + " --timeout 25"
        )
        return self._run_map_reload_command(
            "reload_localization_map",
            script,
            error_code="LOCALIZATION_MAP_RELOAD_FAILED",
            payload={"pcd_path": pcd_path},
        )

    def reload_navigation_map(self, yaml_path: str) -> dict:
        """Reload the occupancy grid held by a running Nav2 map server."""
        requested_path = Path(yaml_path)
        traversable_path = requested_path.with_name("map_traversable.yaml")
        effective_path = traversable_path if traversable_path.is_file() else requested_path
        script = (
            "source /opt/ros/humble/setup.bash && "
            "source /home/dogrobot/robot/install/setup.bash && "
            "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-24} "
            "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}; "
            "timeout 25 ros2 service call /map_server/load_map "
            "nav2_msgs/srv/LoadMap "
            + shlex.quote("{map_url: '" + str(effective_path) + "'}")
        )
        return self._run_map_reload_command(
            "reload_navigation_map",
            script,
            error_code="NAVIGATION_MAP_RELOAD_FAILED",
            payload={
                "yaml_path": str(effective_path),
                "requested_yaml_path": yaml_path,
                "traversable_grid": effective_path == traversable_path,
            },
            success_markers=("result=0", "result: 0"),
        )

    def _run_map_reload_command(
        self,
        action: str,
        script: str,
        *,
        error_code: str,
        payload: dict,
        success_markers: tuple[str, ...] = (),
    ) -> dict:
        try:
            completed = subprocess.run(
                ["bash", "-lc", script],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=max(self.config.command_timeout_seconds, 60),
            )
        except subprocess.TimeoutExpired as exc:
            result = {
                "action": action,
                "returncode": 124,
                "stdout": self._decode_subprocess_output(exc.stdout)[-6000:],
                "stderr": self._decode_subprocess_output(exc.stderr)[-6000:],
                **payload,
            }
            raise ProtocolError(
                error_code,
                result["stderr"] or result["stdout"] or f"{action} timed out",
                details=result,
            ) from exc
        result = {
            "action": action,
            "returncode": completed.returncode,
            "stdout": completed.stdout[-6000:],
            "stderr": completed.stderr[-6000:],
            **payload,
        }
        if completed.returncode != 0:
            raise ProtocolError(
                error_code,
                result["stderr"] or result["stdout"],
                details=result,
            )
        if success_markers and not any(marker in completed.stdout for marker in success_markers):
            result["returncode"] = 1
            raise ProtocolError(
                error_code,
                result["stdout"] or f"{action} returned no successful service response",
                details=result,
            )
        return result

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
        if localization_running and navigation_running:
            result = self.reload_map(pcd_path, yaml_path)
            result["precheck"] = status_payload
            return result

        if localization_running:
            localization = self.reload_localization_map(pcd_path)
            return {
                "action": "reload_map",
                "returncode": 0,
                "deferred": True,
                "reason": "navigation_consumer_inactive",
                "deferred_consumers": ["navigation"],
                "pcd_path": pcd_path,
                "yaml_path": yaml_path,
                "localization_reloaded": True,
                "navigation_reloaded": False,
                "localization": localization,
                "precheck": status_payload,
            }

        if navigation_running:
            navigation = self.reload_navigation_map(yaml_path)
            return {
                "action": "reload_map",
                "returncode": 0,
                "deferred": True,
                "reason": "localization_consumer_inactive",
                "deferred_consumers": ["localization"],
                "pcd_path": pcd_path,
                "yaml_path": yaml_path,
                "localization_reloaded": False,
                "navigation_reloaded": True,
                "navigation": navigation,
                "precheck": status_payload,
            }

        if not (localization_running or navigation_running):
            return {
                "action": "reload_map",
                "returncode": 0,
                "deferred": True,
                "reason": "map_consumers_inactive",
                "deferred_consumers": ["localization", "navigation"],
                "pcd_path": pcd_path,
                "yaml_path": yaml_path,
                "localization_reloaded": False,
                "navigation_reloaded": False,
                "precheck": status_payload,
            }
        raise AssertionError("unreachable map consumer state")

    def reload_boundary_filter(self) -> dict:
        """Reload the generated keepout mask, bootstrapping new filter nodes when needed."""
        mask_yaml = str(Path(self.config.boundary_filter_dir) / "keepout_mask.yaml")
        if not Path(mask_yaml).exists():
            raise ProtocolError("BOUNDARY_MASK_MISSING", f"keepout mask not found: {mask_yaml}")
        status_payload = self.status()
        stdout = str(status_payload.get("stdout") or "")
        navigation_running = any(token in stdout for token in (
            "robot_navigo navigation_bringup.launch.py", "navigo_container", "/planner_server",
        ))
        if not navigation_running:
            return {"action": "reload_boundary_filter", "deferred": True, "mask_yaml": mask_yaml}
        request = "{map_url: '" + mask_yaml.replace("'", "'\\''") + "'}"
        script = (
            "source /opt/ros/humble/setup.bash && "
            "source /home/dogrobot/robot/install/setup.bash && "
            "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-24} "
            "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}; "
            "timeout 25 ros2 service call /filter_mask_server/load_map nav2_msgs/srv/LoadMap "
            + shlex.quote(request)
        )
        completed = subprocess.run(
            ["bash", "-lc", script], check=False, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=max(self.config.command_timeout_seconds, 35),
        )
        if completed.returncode == 0:
            return {
                "action": "reload_boundary_filter", "deferred": False,
                "mask_yaml": mask_yaml, "stdout": completed.stdout[-3000:],
            }
        restarted = self.restart({"reason": "boundary_filter_bootstrap"})
        return {
            "action": "reload_boundary_filter", "deferred": False,
            "recovery": "navigation_restart", "mask_yaml": mask_yaml,
            "reload_error": (completed.stderr or completed.stdout)[-3000:],
            "restart": restarted,
        }

    def _looks_ready(self, stdout: str) -> bool:
        required = (
            "/planner_server",
            "/controller_server",
            "/bt_navigator",
            "active [3]",
            "/follow_waypoints",
            "/cmd_vel",
            "status: 3",
        )
        return all(token in stdout for token in required)

    @staticmethod
    def _looks_prepared(stdout: str) -> bool:
        return all(token in stdout for token in (
            "localization_node", "navigo_container", "/map_server", "/collision_monitor",
            "execution: configured_inactive",
        ))

    @staticmethod
    def _decode_subprocess_output(value: str | bytes | None) -> str:
        """Normalize TimeoutExpired output so command results stay JSON-safe."""
        if value is None:
            return ""
        if isinstance(value, bytes):
            return value.decode("utf-8", errors="replace")
        return value

    def _run(self, action: str, *, timeout_seconds: int) -> dict:
        script = Path(self.config.script_path).expanduser()
        if not script.exists():
            raise ProtocolError("NAV_SCRIPT_MISSING", f"navigation script not found: {script}")
        try:
            completed = subprocess.run(
                [str(script), action],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout_seconds,
            )
        except subprocess.TimeoutExpired as exc:
            payload = {
                "action": action,
                "returncode": 124,
                "stdout": self._decode_subprocess_output(exc.stdout)[-6000:],
                "stderr": self._decode_subprocess_output(exc.stderr)[-6000:],
            }
            raise ProtocolError(
                "NAV_COMMAND_FAILED",
                payload["stderr"] or payload["stdout"] or f"navigation {action} timed out after {timeout_seconds}s",
            ) from exc
        payload = {
            "action": action,
            "returncode": completed.returncode,
            "stdout": completed.stdout[-6000:],
            "stderr": completed.stderr[-6000:],
        }
        if completed.returncode != 0:
            raise ProtocolError("NAV_COMMAND_FAILED", payload["stderr"] or payload["stdout"])
        return payload
