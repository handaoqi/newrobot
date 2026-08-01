from __future__ import annotations

import json
import logging
import os
import re
import signal
import subprocess
import time
import uuid
import zipfile
from dataclasses import dataclass
from pathlib import Path

LOGGER = logging.getLogger(__name__)

from .config import MappingConfig
from .keyframe_visibility_filter import filter_with_keyframe_visibility
from .map_preview import generate_map_preview
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
    OPTIONAL_FILES = ("map.pcd", "map_preview.png", "preview.png", "map.txt", "gnss_origin.yaml")
    SAVE_OUTPUT_TIMEOUT_SECONDS = 7200
    SLAM_PROCESS_PATTERNS = (
        "robot_slam.*mapping",
        "/robot_slam/mapping",
        "lib/robot_slam/mapping",
        "ros2 launch robot_slam",
        "slam.launch.py",
    )

    def __init__(self, config: MappingConfig, media_client: MediaClient) -> None:
        self.config = config
        self.media_client = media_client
        self.map_dir = Path(config.map_dir).expanduser()
        self.session: MappingSession | None = None
        self._slam_process: subprocess.Popen | None = None
        self._slam_log_handle = None
        self._slam_log_path: Path | None = None

    @property
    def _slam_process_alive(self) -> bool:
        return self._slam_process is not None and self._slam_process.poll() is None

    @property
    def _any_slam_process_alive(self) -> bool:
        return self._slam_process_alive or bool(self._find_slam_process_pids())

    def start_mapping(self, command: dict) -> dict:
        # 如果有残留 session 但 SLAM 进程已死（崩溃/中断），自动清理
        if self.session and self.session.state in {"starting", "mapping", "saving", "packaging", "uploading"}:
            if self._slam_process_alive:
                raise ProtocolError("MAPPING_ALREADY_ACTIVE", "mapping session is already active")
            LOGGER.warning("Stale session '%s' (state=%s) detected with dead SLAM process — cleaning up", self.session.session_id, self.session.state)
            self._cleanup()
        if not self.session and self._any_slam_process_alive:
            raise ProtocolError("MAPPING_ALREADY_ACTIVE", "mapping process is already running")
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
        self._stop_conflicting_navigation_stack()
        self._ensure_slam_process()
        self._call_map_state(self.config.start_data)
        self._set_state("mapping")
        return self.status()

    def save_mapping(self, command: dict) -> dict:
        if not self.session:
            if self._any_slam_process_alive:
                LOGGER.warning("save_mapping found running SLAM without active session — recovering session and saving")
                self.session = MappingSession(
                    session_id=command.get("mapping_session_id") or str(uuid.uuid4()),
                    map_name=command.get("map_name") or f"现场地图 {time.strftime('%Y%m%d-%H%M%S')}",
                    route_hint=command.get("route_hint", ""),
                    state="saving",
                    started_at=now_iso(),
                    updated_at=now_iso(),
                )
                return self._save_active_mapping(command)
            recoverable_dir = self._find_latest_recoverable_dir()
            complete_dir = self._find_latest_session_dir(require_complete=True)
            if recoverable_dir and (
                not complete_dir
                or self._latest_file_mtime(recoverable_dir) > self._latest_file_mtime(complete_dir)
            ):
                LOGGER.warning("Recovering interrupted map export from %s", recoverable_dir)
                self.session = MappingSession(
                    session_id=command.get("mapping_session_id") or str(uuid.uuid4()),
                    map_name=command.get("map_name") or recoverable_dir.name,
                    route_hint=command.get("route_hint", ""),
                    state="saving",
                    started_at=now_iso(),
                    updated_at=now_iso(),
                )
                self._ensure_slam_process()
                return self._save_active_mapping(command)
            # 同步模式：无活跃建图会话时，直接打包最近已有的地图文件并上传
            LOGGER.info("save_mapping called without active session — treating as sync")
            session_dir = complete_dir
            if not session_dir:
                raise ProtocolError("NO_MAP_FILES", "no previous complete mapping output found")
            work_dir = session_dir
            self._validate_map_files(work_dir)
            package_path, metadata = self._package_map(command, work_dir)
            upload_result = self.media_client.upload_map_package(str(package_path), metadata)
            result = self.status()
            result["package_path"] = str(package_path)
            result["upload_result"] = upload_result
            return result

        if not self._any_slam_process_alive:
            LOGGER.warning(
                "Mapping session %s has no SLAM process; restarting for persistent-keyframe recovery",
                self.session.session_id,
            )
            self._ensure_slam_process()
        return self._save_active_mapping(command)

    def _save_active_mapping(self, command: dict) -> dict:
        progress_dir = self._find_latest_progress_dir()
        progress = self._read_save_progress(progress_dir)
        readiness = self._mapping_readiness(progress, self._any_slam_process_alive)
        if not readiness["ready_for_save"]:
            self._set_state("mapping" if self._any_slam_process_alive else "failed")
            raise ProtocolError("MAPPING_NOT_READY", readiness["message"])

        self._set_state("saving")
        save_started_at = time.time()
        try:
            self._call_map_state(self.config.save_data)
        except ProtocolError:
            self._set_state("mapping" if self._any_slam_process_alive else "failed")
            progress_dir = self._find_latest_progress_dir()
            progress = self._read_save_progress(progress_dir)
            if (
                progress_dir
                and (
                    progress.get("error_code") == "SLAM_DIVERGED"
                    or progress.get("slam_health", {}).get("state") == "diverged"
                )
            ):
                return self._rescue_diverged_mapping(command, progress_dir)
            raise
        # SLAM writes yaml/pgm asynchronously after the save service returns.
        # Wait for an output touched after this save command instead of falling
        # back to an older complete map directory.
        work_dir = self._wait_for_complete_map_dir(save_started_at)
        self._validate_map_files(work_dir)
        self._set_state("packaging")
        package_path, metadata = self._package_map(command, work_dir)
        self._set_state("uploading")
        upload_result = self.media_client.upload_map_package(str(package_path), metadata)
        self._set_state("stopping")
        self._stop_slam_process()
        self._set_state("exited")
        result = self.status()
        result["package_path"] = str(package_path)
        result["upload_result"] = upload_result
        return result

    def _rescue_diverged_mapping(self, command: dict, source_dir: Path) -> dict:
        self._set_state("recovering")
        self._stop_slam_process()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        output_dir = self.map_dir / f"{stamp}_{int(time.time_ns() / 1_000_000) % 1000:03d}"
        script = Path(__file__).resolve().parents[2] / "script" / "robot" / "rescue_diverged_map.py"
        completed = subprocess.run(
            [str(script), str(source_dir), "--output", str(output_dir)],
            capture_output=True,
            text=True,
            timeout=max(900, self.config.command_timeout_seconds),
        )
        if completed.returncode != 0:
            raise ProtocolError(
                "MAPPING_RESCUE_FAILED",
                (completed.stderr or completed.stdout or "diverged map rescue failed").strip(),
            )
        rescue_command = dict(command)
        rescue_command["map_name"] = (
            f"{self.session.map_name}-发散救援" if self.session and self.session.map_name else f"{source_dir.name}-发散救援"
        )
        self._set_state("packaging")
        package_path, metadata = self._package_map(rescue_command, output_dir)
        self._set_state("uploading")
        upload_result = self.media_client.upload_map_package(str(package_path), metadata)
        self._set_state("exited")
        result = self.status()
        result.update(
            rescued=True,
            rescue_source_dir=str(source_dir),
            rescue_output_dir=str(output_dir),
            package_path=str(package_path),
            upload_result=upload_result,
        )
        return result

    def cancel_mapping(self, command: dict) -> dict:
        progress_dir = self._find_latest_progress_dir()
        if self.session:
            self._set_state("cancelled")
        self._stop_slam_process()
        self._mark_progress_cancelled(progress_dir)
        return self.status()

    def status(self) -> dict:
        process_alive = self._any_slam_process_alive
        complete_session_dir = self._find_latest_session_dir(require_complete=True)
        progress_session_dir = self._find_latest_progress_dir()
        latest_session_dir = (
            progress_session_dir
            if process_alive and progress_session_dir
            else complete_session_dir or progress_session_dir
        )
        progress = self._read_save_progress(latest_session_dir)
        readiness = self._mapping_readiness(progress, process_alive)
        files = self._file_snapshot(latest_session_dir or self.map_dir)
        if not self.session:
            progress_stage = str(progress.get("stage") or "")
            save_stages = {
                "recovering",
                "flushing_keyframes",
                "filtering",
                "partitioning_filter",
                "writing_pcd",
                "building_grid",
                "writing_metadata",
            }
            recovered_state = "saving" if progress_stage in save_stages else ("mapping" if process_alive else "idle")
            return {
                "state": recovered_state,
                "map_dir": str(self.map_dir),
                "active_map_dir": str(latest_session_dir or self.map_dir),
                "latest_session_dir": str(latest_session_dir) if latest_session_dir else None,
                "process_alive": process_alive,
                "slam_pids": self._find_slam_process_pids(),
                "slam_log_path": str(self._slam_log_path) if self._slam_log_path else None,
                "save_progress": progress,
                "readiness": readiness,
                "ready_for_motion": readiness["ready_for_motion"],
                "ready_for_save": readiness["ready_for_save"],
                "files": files,
            }
        state = self.session.state
        if (
            progress.get("error_code") == "SLAM_DIVERGED"
            or progress.get("slam_health", {}).get("state") == "diverged"
        ) and state in {"starting", "mapping", "saving"}:
            state = "failed"
        return {
            "mapping_session_id": self.session.session_id,
            "map_name": self.session.map_name,
            "route_hint": self.session.route_hint,
            "state": state,
            "started_at": self.session.started_at,
            "updated_at": self.session.updated_at,
            "map_dir": str(self.map_dir),
            "active_map_dir": str(latest_session_dir or self.map_dir),
            "latest_session_dir": str(latest_session_dir) if latest_session_dir else None,
            "process_alive": process_alive,
            "slam_pids": self._find_slam_process_pids(),
            "slam_log_path": str(self._slam_log_path) if self._slam_log_path else None,
            "save_progress": progress,
            "readiness": readiness,
            "ready_for_motion": readiness["ready_for_motion"],
            "ready_for_save": readiness["ready_for_save"],
            "files": files,
        }

    @staticmethod
    def _mapping_readiness(progress: dict, process_alive: bool) -> dict:
        health = progress.get("slam_health") or {}
        stage = str(progress.get("stage") or "")
        health_state = str(health.get("state") or "unknown")
        imu_initialized = bool(health.get("imu_initialized"))
        keyframe_count = max(
            int(progress.get("keyframe_count") or 0),
            int(progress.get("written_keyframes") or 0),
        )
        updated_at = float(progress.get("updated_at_unix") or 0)
        sample_age_seconds = max(0.0, time.time() - updated_at) if updated_at > 0 else None
        diverged = progress.get("error_code") == "SLAM_DIVERGED" or health_state == "diverged"

        if diverged:
            state = "diverged"
            message = str(progress.get("error") or health.get("warning") or "SLAM 已发散，请停止并处理地图")
        elif not process_alive:
            state = "offline"
            message = "建图进程未运行"
        elif sample_age_seconds is not None and sample_age_seconds > 5.0:
            state = "telemetry_stale"
            message = "超过 5 秒没有收到雷达/IMU 建图状态，请勿移动机器狗"
        elif not progress:
            state = "starting"
            message = "正在启动 SLAM，等待雷达和 IMU 数据"
        elif not imu_initialized or stage == "initializing_imu":
            state = "imu_initializing"
            samples = int(health.get("imu_samples") or 0)
            required = int(health.get("imu_required_samples") or 0)
            suffix = f"（{samples}/{required}）" if required else f"（已采样 {samples}）"
            message = f"IMU 初始化中{suffix}，请保持机器狗静止"
        elif keyframe_count < 1 or stage == "waiting_first_keyframe":
            state = "waiting_first_keyframe"
            message = "IMU 已初始化，正在建立首个有效关键帧，请继续保持静止"
        else:
            state = "ready"
            message = "传感器和首个关键帧正常，可以开始移动建图"

        return {
            "state": state,
            "message": message,
            "process_alive": process_alive,
            "imu_initialized": imu_initialized,
            "imu_samples": int(health.get("imu_samples") or 0),
            "imu_required_samples": int(health.get("imu_required_samples") or 0),
            "keyframe_count": keyframe_count,
            "sample_age_seconds": sample_age_seconds,
            "ready_for_motion": state == "ready",
            "ready_for_save": keyframe_count > 0 or diverged,
        }

    def _cleanup(self) -> None:
        """Kill orphaned SLAM process and reset session state."""
        self._stop_slam_process()
        self.session = None

    def _ensure_slam_process(self) -> None:
        if self._slam_process and self._slam_process.poll() is None:
            return
        command = self._shell_prefix() + self.config.slam_command
        log_dir = Path(self.config.log_dir).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        self._slam_log_path = log_dir / f"mapping-{time.strftime('%Y%m%d-%H%M%S')}.log"
        self._slam_log_handle = self._slam_log_path.open("ab", buffering=0)
        self._slam_process = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=self._slam_log_handle,
            stderr=subprocess.STDOUT,
        )
        time.sleep(5)
        if self._slam_process.poll() is not None:
            return_code = self._slam_process.returncode
            self._slam_process = None
            if self._slam_log_handle:
                self._slam_log_handle.close()
                self._slam_log_handle = None
            raise ProtocolError(
                "MAPPING_SLAM_START_FAILED",
                f"SLAM process exited during startup with code {return_code}; log={self._slam_log_path}",
            )

    def _stop_conflicting_navigation_stack(self) -> None:
        """Stop localization/Nav2 so mapping owns the lidar, IMU, and map TF."""
        command = (
            'script="$HOME/genisom_roamerx_open/script/robot/start_navigation_real.sh"; '
            'if [ -x "$script" ]; then "$script" full-stop; fi'
        )
        result = subprocess.run(
            ["bash", "-lc", command],
            capture_output=True,
            text=True,
            timeout=max(15, self.config.command_timeout_seconds),
        )
        if result.returncode != 0:
            raise ProtocolError(
                "MAPPING_STACK_STOP_FAILED",
                (result.stderr or result.stdout or "failed to stop navigation/localization").strip(),
            )

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
        if re.search(r"\bsuccess\s*[:=]\s*(?:false|False)\b", output):
            raise ProtocolError("MAPPING_SERVICE_REJECTED", output.strip() or "SLAM rejected map state change")
        return output

    def _find_latest_session_dir(
        self,
        *,
        require_complete: bool = False,
        min_mtime: float | None = None,
    ) -> Path | None:
        """Find the most recent SLAM output directory.

        The mapping node normally uses ``YYYYMMDD_HHMMSS``.  When two saves
        occur in the same second it appends a millisecond suffix, for example
        ``YYYYMMDD_HHMMSS_960``; both forms are valid map sessions.
        """
        if not self.map_dir.exists():
            return None
        dirs = []
        for entry in self.map_dir.iterdir():
            if not self._is_session_dir(entry):
                continue
            if require_complete and not self._has_required_files(entry):
                continue
            if min_mtime is not None and self._latest_file_mtime(entry) < min_mtime:
                continue
            dirs.append(entry)
        return max(dirs, key=self._latest_file_mtime) if dirs else None

    def _find_latest_progress_dir(self) -> Path | None:
        if not self.map_dir.exists():
            return None
        dirs = [
            entry
            for entry in self.map_dir.iterdir()
            if self._is_session_dir(entry) and (entry / "save_progress.json").exists()
        ]
        return max(dirs, key=self._latest_file_mtime) if dirs else None

    def _find_latest_recoverable_dir(self) -> Path | None:
        if not self.map_dir.exists():
            return None
        candidates = []
        for entry in self.map_dir.iterdir():
            if not self._is_session_dir(entry):
                continue
            progress = self._read_save_progress(entry)
            if (
                progress.get("recoverable") is True
                and progress.get("stage") != "completed"
                and progress.get("error_code") != "SLAM_DIVERGED"
                and progress.get("slam_health", {}).get("state") != "diverged"
                and (entry / "keyframes" / "keyframes.csv").exists()
                and not self._has_required_files(entry)
            ):
                candidates.append(entry)
        return max(candidates, key=self._latest_file_mtime) if candidates else None

    @staticmethod
    def _read_save_progress(base: Path | None) -> dict:
        if not base:
            return {}
        path = base / "save_progress.json"
        if not path.exists():
            return {}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            return payload if isinstance(payload, dict) else {}
        except (OSError, ValueError):
            LOGGER.exception("failed to read mapping progress %s", path)
            return {}

    def _mark_progress_cancelled(self, base: Path | None) -> None:
        progress = self._read_save_progress(base)
        if not base or not progress or progress.get("stage") in {"completed", "failed"}:
            return
        progress.update(
            stage="cancelled",
            recoverable=False,
            updated_at_unix=int(time.time()),
            error="",
        )
        path = base / "save_progress.json"
        temporary = path.with_suffix(".json.tmp")
        try:
            temporary.write_text(json.dumps(progress, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            temporary.replace(path)
        except OSError:
            LOGGER.exception("failed to mark mapping progress cancelled: %s", path)
            temporary.unlink(missing_ok=True)

    def _wait_for_complete_map_dir(self, min_mtime: float | None = None) -> Path:
        deadline = time.monotonic() + max(
            self.SAVE_OUTPUT_TIMEOUT_SECONDS,
            int(self.config.save_wait_seconds),
        )
        last_missing: list[str] = []
        while time.monotonic() < deadline:
            session_dir = self._find_latest_session_dir(require_complete=True, min_mtime=min_mtime)
            if session_dir:
                return session_dir
            latest = self._find_latest_session_dir()
            base = latest or self.map_dir
            progress = self._read_save_progress(latest)
            if (
                progress.get("stage") == "failed"
                and self._latest_file_mtime(base) >= (min_mtime or 0)
            ):
                raise ProtocolError(
                    "MAPPING_SAVE_FAILED",
                    str(progress.get("error") or "SLAM map export failed"),
                )
            last_missing = self._missing_required_files(base)
            time.sleep(1)
        raise ProtocolError("MAPPING_FILES_MISSING", f"missing map files: {', '.join(last_missing or self.REQUIRED_FILES)}")

    def _latest_file_mtime(self, base: Path) -> float:
        mtimes = [base.stat().st_mtime]
        for name in self.REQUIRED_FILES + self.OPTIONAL_FILES:
            path = base / name
            if path.exists():
                mtimes.append(path.stat().st_mtime)
        for relative in ("save_progress.json", "keyframes/keyframes.csv"):
            path = base / relative
            if path.exists():
                mtimes.append(path.stat().st_mtime)
        return max(mtimes)

    def _validate_map_files(self, work_dir: Path | None = None) -> None:
        base = work_dir or self.map_dir
        progress = self._read_save_progress(base)
        error_code = str(progress.get("error_code") or "")
        health_state = str(progress.get("slam_health", {}).get("state") or "")
        if error_code == "SLAM_DIVERGED" or health_state == "diverged":
            raise ProtocolError(
                "SLAM_DIVERGED",
                str(progress.get("error") or "SLAM health guard rejected this map"),
            )
        if progress and progress.get("stage") == "failed":
            raise ProtocolError(
                error_code or "MAPPING_SAVE_FAILED",
                str(progress.get("error") or "SLAM map export failed"),
            )
        missing = self._missing_required_files(base)
        if missing:
            raise ProtocolError("MAPPING_FILES_MISSING", f"missing map files: {', '.join(missing)}")

    def _missing_required_files(self, base: Path) -> list[str]:
        return [name for name in self.REQUIRED_FILES if not (base / name).exists()]

    def _has_required_files(self, base: Path) -> bool:
        return not self._missing_required_files(base)

    def _is_session_dir(self, path: Path) -> bool:
        if not path.is_dir():
            return False
        if not re.fullmatch(r"\d{8}_\d{6}(?:_\d{3})?(?:_\d+)?", path.name):
            return False
        timestamp = path.name[:15]
        try:
            time.strptime(timestamp, "%Y%m%d_%H%M%S")
            return True
        except ValueError:
            return False

    def _file_snapshot(self, base: Path) -> dict:
        files = {}
        for name in self.REQUIRED_FILES + self.OPTIONAL_FILES:
            path = base / name
            files[name] = {
                "exists": path.exists(),
                "size": path.stat().st_size if path.exists() else 0,
                "path": str(path),
                "is_symlink": path.is_symlink(),
            }
        return files

    def _package_map(self, command: dict, work_dir: Path | None = None) -> tuple[Path, dict]:
        base = work_dir or self.map_dir
        if not self._is_session_dir(base):
            raise ProtocolError("INVALID_MAP_OUTPUT_DIR", f"refusing to package non-session map dir: {base}")
        progress = self._read_save_progress(base)
        rescue_metadata = progress.get("rescue") or {}
        is_rescue = bool(rescue_metadata)
        if is_rescue:
            # Rescue output is already rebuilt from the valid keyframe prefix
            # and intentionally does not copy every source scan file.
            filtered_base = base
            dynamic_filter_result = {
                "enabled": False,
                "skipped": "rescued_map_already_rebuilt",
                "source": str(base),
            }
        else:
            filtered_base, dynamic_filter_result = self._filter_map_outputs(base)
        preview_path = self._generate_map_preview(filtered_base)
        version = time.strftime("%Y%m%d-%H%M%S")
        package_path = self.map_dir / f"map_package_{version}.zip"
        upload_files = ["map.yaml", "map.pgm", "map.txt"]
        if preview_path:
            upload_files.append(preview_path.name)
        if self.config.upload_point_cloud:
            upload_files.append("map.pcd")
        files = []
        with zipfile.ZipFile(package_path, "w", zipfile.ZIP_DEFLATED) as archive:
            for name in upload_files:
                path = filtered_base / name
                if path.exists():
                    archive.write(path, arcname=name)
                    files.append(name)
        # A rescue map must be inspected before it can replace the active
        # navigation map. Normal saves retain the existing auto-activation flow.
        if not is_rescue:
            self._refresh_current_map_links(filtered_base, list(self.REQUIRED_FILES + self.OPTIONAL_FILES))
        metadata = {
            "robot_code": self.media_client.robot_id,
            "mapping_session_id": self.session.session_id if self.session else str(uuid.uuid4()),
            # 地图名称使用 session 目录的时间戳，与 .jszr/map/<timestamp> 一致
            "map_name": command.get("map_name") or (work_dir.name if work_dir and work_dir != self.map_dir else (self.session.map_name if self.session else "untitled")),
            "map_version": version,
            "source_map_dir": str(filtered_base),
            "raw_map_dir": str(base),
            "auto_activate": bool(self.config.auto_activate_uploaded_map and not is_rescue),
            "route_hint": self.session.route_hint if self.session else "",
            "frame_id": "map",
            "resolution": command.get("resolution") or 0.05,
            "origin": command.get("origin") or [],
            "created_at": now_iso(),
            "files": files,
            "dynamic_filter": dynamic_filter_result,
            "slam_health": progress.get("slam_health") or {},
            "rescue": rescue_metadata,
        }
        return package_path, metadata

    def _generate_map_preview(self, base: Path) -> Path | None:
        pgm_path = base / "map.pgm"
        if not pgm_path.exists():
            return None
        preview_path = base / "map_preview.png"
        try:
            return generate_map_preview(pgm_path, preview_path, max_size=int(self.config.preview_max_size))
        except Exception:
            LOGGER.exception("failed to generate map preview for %s", base)
            return None

    def _filter_map_outputs(self, base: Path) -> tuple[Path, dict]:
        """Create the navigation map from keyframe visibility evidence.

        The raw SLAM output stays untouched. This keeps the filter repeatable
        and makes the active local map identical to the map sent upstream.
        """
        if not self.config.visibility_filter_enabled:
            return base, {
                "enabled": False,
                "mode": "manual_cleanup",
                "source": str(base),
            }
        pcd_path = base / "map.pcd"
        source_bytes = pcd_path.stat().st_size if pcd_path.exists() else 0
        max_source_bytes = int(self.config.visibility_filter_max_source_bytes)
        if max_source_bytes > 0 and source_bytes > max_source_bytes:
            result = {
                "enabled": True,
                "skipped": "source_exceeds_memory_safe_visibility_limit",
                "source_bytes": source_bytes,
                "max_source_bytes": max_source_bytes,
                "active_filter": "disk_sharded_cpp_keyframe_filter",
            }
            LOGGER.warning(
                "Skipping whole-map visibility filter for %s bytes=%s limit=%s",
                base,
                source_bytes,
                max_source_bytes,
            )
            return base, result
        output = base / self.config.visibility_filter_output_suffix
        try:
            result = filter_with_keyframe_visibility(
                base,
                output,
                voxel_size_m=float(self.config.visibility_filter_voxel_size_m),
                min_free_observations=int(self.config.visibility_filter_min_free_observations),
                max_hit_observations=int(self.config.visibility_filter_max_hit_observations),
            )
        except Exception as exc:
            LOGGER.exception("Keyframe visibility map filter failed for %s", base)
            raise ProtocolError("MAPPING_FILTER_FAILED", f"keyframe visibility filter failed: {exc}") from exc
        return output, result

    def _refresh_current_map_links(self, base: Path, files: list[str]) -> None:
        for name in files:
            if name not in self.REQUIRED_FILES + self.OPTIONAL_FILES:
                continue
            source = base / name
            if not source.exists():
                continue
            target = self.map_dir / name
            if target.resolve() == source.resolve():
                continue
            tmp_link = self.map_dir / f".{name}.tmp-link"
            if tmp_link.exists() or tmp_link.is_symlink():
                tmp_link.unlink()
            tmp_link.symlink_to(source)
            tmp_link.replace(target)
        LOGGER.info("Current map links refreshed to %s with files=%s", base, files)

    def _set_state(self, state: str) -> None:
        if self.session:
            self.session.state = state
            self.session.updated_at = now_iso()

    def _stop_slam_process(self) -> None:
        if self._slam_process and self._slam_process.poll() is None:
            self._slam_process.terminate()
            try:
                self._slam_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._slam_process.kill()
        self._slam_process = None
        self._stop_orphan_slam_processes()
        if self._slam_log_handle:
            self._slam_log_handle.close()
            self._slam_log_handle = None

    def _find_slam_process_pids(self) -> list[int]:
        try:
            result = subprocess.run(
                ["ps", "-eo", "pid=,args="],
                capture_output=True,
                text=True,
                timeout=5,
            )
        except Exception:
            LOGGER.exception("failed to inspect SLAM processes")
            return []
        if result.returncode != 0:
            return []
        pids: list[int] = []
        current_pid = os.getpid()
        for line in result.stdout.splitlines():
            line = line.strip()
            if not line:
                continue
            pid_text, _, args = line.partition(" ")
            try:
                pid = int(pid_text)
            except ValueError:
                continue
            if pid == current_pid:
                continue
            if any(re.search(pattern, args) for pattern in self.SLAM_PROCESS_PATTERNS):
                pids.append(pid)
        return pids

    def _stop_orphan_slam_processes(self) -> None:
        pids = self._find_slam_process_pids()
        if not pids:
            return
        LOGGER.warning("Stopping orphan SLAM mapping processes: %s", pids)
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except PermissionError:
                LOGGER.warning("no permission to terminate SLAM process pid=%s", pid)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if not self._find_slam_process_pids():
                return
            time.sleep(0.5)
        for pid in self._find_slam_process_pids():
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except PermissionError:
                LOGGER.warning("no permission to kill SLAM process pid=%s", pid)

    def _shell_prefix(self) -> str:
        workspace_setup = self.config.workspace_setup
        return f"source {self.config.ros_setup} && source {workspace_setup} && "
