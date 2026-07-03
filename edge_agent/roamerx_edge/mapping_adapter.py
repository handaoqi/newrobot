from __future__ import annotations

import logging
import subprocess
import time
import uuid
import zipfile
from dataclasses import dataclass
from pathlib import Path

LOGGER = logging.getLogger(__name__)

from .config import MappingConfig
from .media_client import MediaClient
from .protocol import ProtocolError, now_iso


@dataclass
class MappingSession:
    session_id: str
    map_name: str
    route_hint: str
    state: str
    started_at: str
    updated_at: str


class MappingAdapter:
    """Edge-side adapter for onsite SLAM mapping.

    The center platform owns the command lifecycle. This adapter only controls
    local ROS2/SLAM and uploads the generated map package back to the center.
    It intentionally does not control robot motion; onsite motion remains in
    Orche APP / local SDK control for lower latency and safety.
    """

    REQUIRED_FILES = ("map.yaml", "map.pgm")
    OPTIONAL_FILES = ("map.pcd", "map_preview.png", "preview.png", "map.txt")

    def __init__(self, config: MappingConfig, media_client: MediaClient) -> None:
        self.config = config
        self.media_client = media_client
        self.map_dir = Path(config.map_dir).expanduser()
        self.session: MappingSession | None = None
        self._slam_process: subprocess.Popen | None = None

    def start_mapping(self, command: dict) -> dict:
        if self.session and self.session.state in {"starting", "mapping", "saving", "packaging", "uploading"}:
            raise ProtocolError("MAPPING_ALREADY_ACTIVE", "mapping session is already active")
        session_id = command.get("mapping_session_id") or str(uuid.uuid4())
        map_name = command.get("map_name") or f"现场地图 {time.strftime('%Y%m%d-%H%M%S')}"
        self.session = MappingSession(
            session_id=session_id,
            map_name=map_name,
            route_hint=command.get("route_hint", ""),
            state="starting",
            started_at=now_iso(),
            updated_at=now_iso(),
        )
        self.map_dir.mkdir(parents=True, exist_ok=True)
        self._ensure_slam_process()
        self._call_map_state(self.config.start_data)
        self._set_state("mapping")
        return self.status()

    def save_mapping(self, command: dict) -> dict:
        if not self.session:
            # 同步模式：无活跃建图会话时，直接打包最近已有的地图文件并上传
            LOGGER.info("save_mapping called without active session — treating as sync")
            session_dir = self._find_latest_session_dir()
            if not session_dir:
                raise ProtocolError("NO_MAP_FILES", "no previous mapping output found (no session dirs)")
            work_dir = session_dir
            self._validate_map_files(work_dir)
            package_path, metadata = self._package_map(command, work_dir)
            upload_result = self.media_client.upload_map_package(str(package_path), metadata)
            result = self.status()
            result["package_path"] = str(package_path)
            result["upload_result"] = upload_result
            return result

        self._set_state("saving")
        self._call_map_state(self.config.save_data)
        time.sleep(max(0, self.config.save_wait_seconds))
        # SLAM may write output into a timestamped subdirectory (YYYYMMDD_HHMMSS)
        session_dir = self._find_latest_session_dir()
        work_dir = session_dir or self.map_dir
        self._validate_map_files(work_dir)
        self._set_state("packaging")
        package_path, metadata = self._package_map(command, work_dir)
        self._set_state("uploading")
        upload_result = self.media_client.upload_map_package(str(package_path), metadata)
        self._set_state("completed")
        result = self.status()
        result["package_path"] = str(package_path)
        result["upload_result"] = upload_result
        return result

    def cancel_mapping(self, command: dict) -> dict:
        if self.session:
            self._set_state("cancelled")
        return self.status()

    def status(self) -> dict:
        files = {}
        for name in self.REQUIRED_FILES + self.OPTIONAL_FILES:
            path = self.map_dir / name
            files[name] = {"exists": path.exists(), "size": path.stat().st_size if path.exists() else 0}
        if not self.session:
            return {"state": "idle", "files": files}
        return {
            "mapping_session_id": self.session.session_id,
            "map_name": self.session.map_name,
            "route_hint": self.session.route_hint,
            "state": self.session.state,
            "started_at": self.session.started_at,
            "updated_at": self.session.updated_at,
            "map_dir": str(self.map_dir),
            "files": files,
        }

    def _ensure_slam_process(self) -> None:
        if self._slam_process and self._slam_process.poll() is None:
            return
        command = self._shell_prefix() + self.config.slam_command
        self._slam_process = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(5)

    def _call_map_state(self, data: int) -> str:
        payload = f'"{{data: {int(data)}}}"'
        command = (
            self._shell_prefix()
            + f"ros2 service call {self.config.service_name} {self.config.service_type} {payload}"
        )
        result = subprocess.run(
            ["bash", "-lc", command],
            capture_output=True,
            text=True,
            timeout=self.config.command_timeout_seconds,
        )
        output = result.stdout or result.stderr
        if result.returncode != 0:
            raise ProtocolError("MAPPING_SERVICE_FAILED", output.strip() or "ros2 service call failed")
        return output

    def _find_latest_session_dir(self) -> Path | None:
        """Find the most recent SLAM output subdirectory (YYYYMMDD_HHMMSS format)."""
        dirs = []
        for entry in self.map_dir.iterdir():
            if entry.is_dir() and len(entry.name) == 15 and entry.name[8] == "_":
                try:
                    time.strptime(entry.name, "%Y%m%d_%H%M%S")
                    dirs.append(entry)
                except ValueError:
                    pass
        return max(dirs, key=lambda d: d.stat().st_mtime) if dirs else None

    def _validate_map_files(self, work_dir: Path | None = None) -> None:
        base = work_dir or self.map_dir
        missing = [name for name in self.REQUIRED_FILES if not (base / name).exists()]
        if missing:
            raise ProtocolError("MAPPING_FILES_MISSING", f"missing map files: {', '.join(missing)}")

    def _package_map(self, command: dict, work_dir: Path | None = None) -> tuple[Path, dict]:
        base = work_dir or self.map_dir
        version = time.strftime("%Y%m%d-%H%M%S")
        package_path = self.map_dir / f"map_package_{version}.zip"
        files = []
        with zipfile.ZipFile(package_path, "w", zipfile.ZIP_DEFLATED) as archive:
            for name in self.REQUIRED_FILES + self.OPTIONAL_FILES:
                path = base / name
                if path.exists():
                    archive.write(path, arcname=name)
                    files.append(name)
        metadata = {
            "robot_code": self.media_client.robot_id,
            "mapping_session_id": self.session.session_id if self.session else str(uuid.uuid4()),
            # 地图名称使用 session 目录的时间戳，与 .jszr/map/<timestamp> 一致
            "map_name": command.get("map_name") or (work_dir.name if work_dir and work_dir != self.map_dir else (self.session.map_name if self.session else "untitled")),
            "map_version": version,
            "route_hint": self.session.route_hint if self.session else "",
            "frame_id": "map",
            "resolution": command.get("resolution") or 0.05,
            "origin": command.get("origin") or [],
            "created_at": now_iso(),
            "files": files,
        }
        return package_path, metadata

    def _set_state(self, state: str) -> None:
        if self.session:
            self.session.state = state
            self.session.updated_at = now_iso()

    def _shell_prefix(self) -> str:
        workspace_setup = self.config.workspace_setup
        return f"source {self.config.ros_setup} && source {workspace_setup} && "
