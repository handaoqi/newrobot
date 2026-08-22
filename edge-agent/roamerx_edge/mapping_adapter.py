from __future__ import annotations

import csv
import json
import logging
import math
import os
import re
import signal
import subprocess
import time
import uuid
import zipfile
from dataclasses import asdict, dataclass
from pathlib import Path

import yaml

LOGGER = logging.getLogger(__name__)

from .config import MappingConfig
from .keyframe_visibility_filter import filter_with_keyframe_visibility
from .map_coordinate import MapConstraintError, SCENE_SCOPES, normalize_text
from .map_package_finalize import finalize_map_package
from .map_preview import generate_map_preview
from .media_client import MediaClient
from .origin_lock import OriginLockMonitor, OriginSample
from .protocol import ProtocolError, now_iso


@dataclass
class MappingSession:
    session_id: str
    map_name: str
    route_hint: str
    state: str
    started_at: str
    updated_at: str
    scene_scope: str = "indoor"
    mapping_type: str = "indoor"
    heading_check_confirmed: bool = False
    mapping_capture_enabled: bool = False


class MappingAdapter:
    """Edge-side adapter for onsite SLAM mapping.

    The center platform owns the command lifecycle. This adapter only controls
    local ROS2/SLAM and uploads the generated map package back to the center.
    It intentionally does not control robot motion; onsite motion remains in
    Orche APP / local SDK control for lower latency and safety.
    """

    REQUIRED_FILES = ("map.yaml", "map.pgm")
    OPTIONAL_FILES = (
        "map.pcd",
        "map_raw.pcd",
        "map_preview.png",
        "preview.png",
        "map.txt",
        "gnss_origin.yaml",
        "mapping_trace.json",
        "map_manifest.json",
        "recording_manifest.yaml",
        "trajectory_raw.csv",
        "trajectory_optimized.csv",
    )
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
        self._record_rosbag = False
        self._rosbag_dir: str | None = None
        self._scene_scope = "indoor"
        self._mapping_type = "indoor"
        self._origin_file = Path(config.origin_file).expanduser() if config.origin_file else self.map_dir / "gnss_origin.yaml"
        self._origin_state_file = (
            Path(config.origin_state_file).expanduser()
            if config.origin_state_file
            else self.map_dir / "mapping_workflow.json"
        )
        self._origin_monitor = OriginLockMonitor(
            self._sample_origin_topics,
            str(self._origin_file),
            duration_seconds=config.origin_lock_duration_seconds,
            max_spread_m=config.origin_lock_max_spread_m,
            sample_interval_seconds=config.origin_lock_sample_interval_seconds,
            min_baseline_m=config.heading_min_baseline_m,
            max_heading_std_deg=config.heading_max_std_deg,
            max_age_seconds=config.heading_max_age_seconds,
            no_signal_timeout_seconds=config.origin_lock_no_signal_timeout_seconds,
        )
        self._restore_workflow_state()

    @property
    def _slam_process_alive(self) -> bool:
        return self._slam_process is not None and self._slam_process.poll() is None

    @property
    def _any_slam_process_alive(self) -> bool:
        return self._is_mapping_unit_active() or self._slam_process_alive or bool(self._find_slam_process_pids())

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
        scene_scope = normalize_text(command.get("scene_scope"), "indoor")
        if scene_scope not in SCENE_SCOPES:
            raise ProtocolError("MAP_CONSTRAINT_INVALID", "scene_scope must be indoor, transition, or outdoor")
        self._scene_scope = scene_scope
        mapping_type = normalize_text(command.get("mapping_type"), "outdoor" if scene_scope != "indoor" else "indoor")
        if mapping_type not in {"indoor", "outdoor"}:
            raise ProtocolError("MAPPING_TYPE_REQUIRED", "mapping_type must be indoor or outdoor")
        if mapping_type == "indoor" and scene_scope != "indoor":
            raise ProtocolError("MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN", "indoor mapping cannot use an outdoor or transition scene scope")
        if mapping_type == "outdoor" and scene_scope == "indoor":
            raise ProtocolError("MAP_LOCAL_ONLY_TRANSITION_FORBIDDEN", "outdoor mapping cannot use indoor scene scope")
        if mapping_type == "outdoor" and self._origin_monitor.status().get("origin_status") != "locked":
            raise ProtocolError("MAPPING_ORIGIN_REQUIRED", "outdoor mapping must lock the ENU origin before SLAM starts")
        self._mapping_type = mapping_type
        self.session = MappingSession(
            session_id=session_id,
            map_name=map_name,
            route_hint=command.get("route_hint", ""),
            state="starting",
            started_at=now_iso(),
            updated_at=now_iso(),
            scene_scope=scene_scope,
            mapping_type=mapping_type,
            mapping_capture_enabled=True,
        )
        self.map_dir.mkdir(parents=True, exist_ok=True)
        self._stop_conflicting_navigation_stack()
        self._ensure_mapping_sensors()
        self._record_rosbag = bool(command.get("record_rosbag", False))
        if self._record_rosbag:
            self._start_rosbag(map_name)
        try:
            self._ensure_slam_process()
            if mapping_type == "outdoor":
                self._call_map_state(self.config.warmup_data)
            self._call_map_state(self.config.start_data)
        except Exception:
            self._stop_rosbag()
            raise
        self._set_state("mapping")
        return self.status()

    def start_origin_lock(self, command: dict) -> dict:
        """Start outdoor sensors and the 60-second RTK anchor quality window."""
        if self._any_slam_process_alive:
            raise ProtocolError("MAPPING_ALREADY_ACTIVE", "stop SLAM before locking a new ENU origin")
        if self.session and self.session.state not in {"idle", "cancelled", "exited", "failed"}:
            raise ProtocolError("MAPPING_ALREADY_ACTIVE", "mapping workflow is already active")
        scene_scope = normalize_text(command.get("scene_scope"), "outdoor")
        if scene_scope not in {"transition", "outdoor"}:
            raise ProtocolError("MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN", "indoor mapping does not lock an ENU origin")
        self._mapping_type = "outdoor"
        self._scene_scope = scene_scope
        self.session = MappingSession(
            session_id=command.get("mapping_session_id") or str(uuid.uuid4()),
            map_name=command.get("map_name") or f"室外地图 {time.strftime('%Y%m%d-%H%M%S')}",
            route_hint=command.get("route_hint", ""),
            state="origin_starting",
            started_at=now_iso(),
            updated_at=now_iso(),
            scene_scope=scene_scope,
            mapping_type="outdoor",
        )
        self.map_dir.mkdir(parents=True, exist_ok=True)
        self._stop_conflicting_navigation_stack()
        self._ensure_mapping_sensors()
        self._origin_monitor.start()
        self._set_state("origin_waiting")
        return self.status()

    def cancel_origin_lock(self, command: dict) -> dict:
        del command
        origin = self._origin_monitor.cancel()
        if self.session and self.session.state in {"origin_starting", "origin_waiting", "origin_locked"}:
            self._set_state("cancelled")
        result = self.status()
        result["origin"] = origin
        return result

    def start_slam_warmup(self, command: dict) -> dict:
        """Start FAST-LIO-SAM estimator while keeping formal keyframe capture closed."""
        mapping_type = normalize_text(command.get("mapping_type"), self._mapping_type or "indoor")
        default_scene_scope = self.session.scene_scope if self.session else mapping_type
        scene_scope = normalize_text(command.get("scene_scope"), default_scene_scope)
        if mapping_type not in {"indoor", "outdoor"}:
            raise ProtocolError("MAPPING_TYPE_REQUIRED", "mapping_type must be indoor or outdoor")
        if mapping_type == "indoor" and scene_scope != "indoor":
            raise ProtocolError("MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN", "indoor mapping cannot use an outdoor or transition scene scope")
        if mapping_type == "outdoor" and scene_scope == "indoor":
            raise ProtocolError("MAP_LOCAL_ONLY_TRANSITION_FORBIDDEN", "outdoor mapping cannot use indoor scene scope")
        if self.session and self.session.state in {"cancelled", "exited", "failed", "idle"}:
            self.session = None
        if self.session and self.session.state not in {"origin_waiting", "origin_locked"}:
            raise ProtocolError("MAPPING_ALREADY_ACTIVE", "mapping workflow is already active")
        if self.session and self.session.mapping_type != mapping_type:
            raise ProtocolError("MAPPING_TYPE_REQUIRED", "mapping type cannot change inside an active workflow")
        origin_status = self._origin_monitor.status()
        if mapping_type == "outdoor" and origin_status.get("origin_status") != "locked":
            raise ProtocolError("MAPPING_ORIGIN_REQUIRED", "outdoor mapping requires a valid locked ENU origin")
        if mapping_type == "outdoor":
            locked_at = float((origin_status.get("origin") or {}).get("locked_at_unix") or 0)
            if locked_at and time.time() - locked_at > self.config.origin_lock_ttl_seconds:
                raise ProtocolError("MAPPING_ORIGIN_EXPIRED", "locked ENU origin has expired; lock it again")
            self._origin_monitor.resume()
        if not self.session:
            self.session = MappingSession(
                session_id=command.get("mapping_session_id") or str(uuid.uuid4()),
                map_name=command.get("map_name") or f"现场地图 {time.strftime('%Y%m%d-%H%M%S')}",
                route_hint=command.get("route_hint", ""),
                state="starting",
                started_at=now_iso(),
                updated_at=now_iso(),
                scene_scope=scene_scope,
                mapping_type=mapping_type,
            )
            self._stop_conflicting_navigation_stack()
            self._ensure_mapping_sensors()
        self._mapping_type = mapping_type
        self._scene_scope = scene_scope
        self.session.mapping_type = mapping_type
        self.session.scene_scope = scene_scope
        self._record_rosbag = bool(command.get("record_rosbag", False))
        if self._record_rosbag and not self._rosbag_status().get("running"):
            self._start_rosbag(self.session.map_name)
        self._set_state("slam_starting")
        try:
            self._ensure_slam_process()
            self._call_map_state(
                self.config.warmup_data if mapping_type == "outdoor" else self.config.indoor_warmup_data
            )
        except Exception:
            self._stop_rosbag()
            self._set_state("failed")
            raise
        self.session.mapping_capture_enabled = False
        self._set_state("slam_warmup")
        return self.status()

    def begin_mapping(self, command: dict) -> dict:
        if not self.session or self.session.state not in {"slam_warmup", "ready_to_map"}:
            raise ProtocolError("MAPPING_NOT_READY", "start SLAM warmup before formal mapping")
        status = self.status()
        readiness = status.get("readiness") or {}
        if not readiness.get("imu_initialized") or not readiness.get("slam_pose_ready"):
            raise ProtocolError("MAPPING_NOT_READY", str(readiness.get("message") or "IMU or SLAM pose is not ready"))
        if self.session.mapping_type == "outdoor":
            origin = status.get("origin") or {}
            if origin.get("origin_status") != "locked":
                raise ProtocolError("MAPPING_ORIGIN_REQUIRED", "outdoor ENU origin is no longer locked")
            if not origin.get("heading_stable"):
                raise ProtocolError("MAPPING_HEADING_NOT_CONFIRMED", "dual-antenna heading is not stable")
            if not bool(command.get("heading_check_confirmed")):
                raise ProtocolError("MAPPING_HEADING_NOT_CONFIRMED", "operator must confirm the heading check")
            self.session.heading_check_confirmed = True
        self._call_map_state(self.config.start_data)
        self.session.mapping_capture_enabled = True
        self._origin_monitor.stop()
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
                    scene_scope=normalize_text(command.get("scene_scope"), "indoor"),
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
                    scene_scope=normalize_text(command.get("scene_scope"), "indoor"),
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
            self._finalize_session_package(work_dir, command)
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
            self._set_state(self.session.state if self._any_slam_process_alive and self.session else "failed")
            raise ProtocolError("MAPPING_NOT_READY", readiness["message"])

        # The diagnostic bag captures sensor input while the robot is mapping;
        # stop it before the CPU- and disk-heavy map export begins.
        self._stop_rosbag()
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
        self._finalize_session_package(work_dir, command)
        should_upload = bool(command.get("upload", True))
        should_package = bool(command.get("package", should_upload))
        should_stop = bool(command.get("stop_process", True))
        result = self.status()
        if should_package:
            self._set_state("packaging")
            package_path, metadata = self._package_map(command, work_dir)
            result["package_path"] = str(package_path)
            if should_upload:
                self._set_state("uploading")
                result["upload_result"] = self.media_client.upload_map_package(str(package_path), metadata)
        if should_stop:
            self._set_state("stopping")
            self._stop_rosbag()
            self._stop_slam_process()
            self._set_state("exited")
        result.update(self.status())
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
        self._finalize_session_package(output_dir, rescue_command)
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
        self._stop_rosbag()
        self._stop_slam_process()
        if self.session and self.session.mapping_type == "outdoor":
            self._origin_monitor.cancel()
        else:
            self._origin_monitor.stop()
        self._mark_progress_cancelled(progress_dir)
        return self.status()

    def status(self) -> dict:
        process_alive = self._any_slam_process_alive
        origin = self._origin_monitor.status()
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
        rosbag = self._rosbag_status()
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
                "rosbag": rosbag,
                "origin": origin,
                "origin_status": origin.get("origin_status", "idle"),
                "mapping_type": self._mapping_type,
                "slam_process_alive": process_alive,
                "slam_warmup": False,
                "mapping_capture_enabled": False,
                "imu_initialized": bool(readiness.get("imu_initialized")),
                "slam_pose_ready": bool(readiness.get("slam_pose_ready")),
                "ready_for_mapping": bool(readiness.get("ready_for_mapping")),
            }
        state = self.session.state
        if state == "origin_waiting" and origin.get("origin_status") == "locked":
            self._set_state("origin_locked")
            state = "origin_locked"
        if state in {"origin_starting", "origin_waiting"} and origin.get("origin_status") == "failed":
            self._set_state("failed")
            state = "failed"
        if state == "slam_warmup" and readiness.get("ready_for_mapping"):
            self._set_state("ready_to_map")
            state = "ready_to_map"
        if (
            progress.get("error_code") == "SLAM_DIVERGED"
            or progress.get("slam_health", {}).get("state") == "diverged"
        ) and state in {"starting", "mapping", "saving"}:
            state = "failed"
        return {
            "mapping_session_id": self.session.session_id,
            "map_name": self.session.map_name,
            "route_hint": self.session.route_hint,
            "mapping_type": self.session.mapping_type,
            "scene_scope": self.session.scene_scope,
            "state": state,
            "started_at": self.session.started_at,
            "updated_at": self.session.updated_at,
            "map_dir": str(self.map_dir),
            "active_map_dir": str(latest_session_dir or self.map_dir),
            "latest_session_dir": str(latest_session_dir) if latest_session_dir else None,
            "process_alive": process_alive,
            "slam_process_alive": process_alive,
            "slam_warmup": state in {"slam_starting", "slam_warmup", "ready_to_map"},
            "heading_check_confirmed": self.session.heading_check_confirmed,
            "mapping_capture_enabled": self.session.mapping_capture_enabled,
            "imu_initialized": bool(readiness.get("imu_initialized")),
            "slam_pose_ready": bool(readiness.get("slam_pose_ready")),
            "slam_pids": self._find_slam_process_pids(),
            "slam_log_path": str(self._slam_log_path) if self._slam_log_path else None,
            "save_progress": progress,
            "readiness": readiness,
            "ready_for_motion": readiness["ready_for_motion"],
            "ready_for_save": readiness["ready_for_save"],
            "files": files,
            "rosbag": rosbag,
            "origin": origin,
            "origin_status": origin.get("origin_status", "idle"),
            "ready_for_mapping": bool(readiness.get("ready_for_mapping"))
            and (self.session.mapping_type == "indoor" or origin.get("heading_stable") is True),
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
        slam_pose_ready = bool(progress.get("slam_pose_ready") or health.get("slam_pose_ready") or keyframe_count > 0)
        capture_enabled = bool(
            progress.get("mapping_capture_enabled")
            if "mapping_capture_enabled" in progress
            else keyframe_count > 0
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
        elif not slam_pose_ready:
            state = "waiting_first_keyframe"
            message = "IMU 已初始化，正在建立首个有效 SLAM 位姿，请继续保持静止"
        elif not capture_enabled:
            state = "ready"
            message = "SLAM 预热检查通过，等待人工确认后开始正式采集关键帧"
        elif keyframe_count < 1 or stage == "waiting_first_keyframe":
            state = "waiting_first_keyframe"
            message = "正式采集已开启，正在建立首个关键帧"
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
            "slam_pose_ready": slam_pose_ready,
            "mapping_capture_enabled": capture_enabled,
            "keyframe_count": keyframe_count,
            "sample_age_seconds": sample_age_seconds,
            "ready_for_motion": state == "ready" and capture_enabled,
            "ready_for_mapping": imu_initialized and slam_pose_ready and not diverged,
            "ready_for_save": keyframe_count > 0 or diverged,
        }

    def wait_until_ready_for_motion(self, timeout_seconds: float = 90.0) -> dict:
        deadline = time.monotonic() + max(5.0, float(timeout_seconds))
        last = self.status()
        while time.monotonic() < deadline:
            last = self.status()
            readiness = last.get("readiness") or {}
            if readiness.get("ready_for_motion"):
                return last
            if readiness.get("state") in {"diverged", "offline"}:
                raise ProtocolError("MAPPING_NOT_READY", str(readiness.get("message") or "mapping is not ready"))
            time.sleep(1)
        raise ProtocolError(
            "MAPPING_NOT_READY",
            str((last.get("readiness") or {}).get("message") or "timed out waiting for IMU initialization and the first keyframe"),
        )

    def _cleanup(self) -> None:
        """Kill orphaned SLAM process and reset session state."""
        self._stop_rosbag()
        self._stop_slam_process()
        self._origin_monitor.stop()
        self.session = None

    def _restore_workflow_state(self) -> None:
        origin_restored = self._origin_monitor.restore(self.config.origin_lock_ttl_seconds)
        if origin_restored:
            self._mapping_type = "outdoor"
            self._scene_scope = "outdoor"
        if not origin_restored or not self._origin_state_file.is_file():
            return
        try:
            payload = json.loads(self._origin_state_file.read_text(encoding="utf-8"))
            raw = payload.get("session") or {}
        except (OSError, ValueError, TypeError):
            LOGGER.warning("Ignoring invalid mapping workflow state at %s", self._origin_state_file)
            return
        if raw.get("mapping_type") != "outdoor" or raw.get("state") not in {"origin_locked", "origin_waiting"}:
            return
        self.session = MappingSession(
            session_id=str(raw.get("session_id") or uuid.uuid4()),
            map_name=str(raw.get("map_name") or "恢复的室外地图"),
            route_hint=str(raw.get("route_hint") or ""),
            state="origin_locked",
            started_at=str(raw.get("started_at") or now_iso()),
            updated_at=now_iso(),
            scene_scope=str(raw.get("scene_scope") or "outdoor"),
            mapping_type="outdoor",
        )
        self._scene_scope = self.session.scene_scope

    def _persist_workflow_state(self) -> None:
        if not self.session:
            return
        self._origin_state_file.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema": "roamerx.mapping-workflow.v2",
            "session": asdict(self.session),
            "origin_status": self._origin_monitor.status().get("origin_status"),
            "updated_at": now_iso(),
        }
        temporary = self._origin_state_file.with_suffix(self._origin_state_file.suffix + ".tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(temporary, self._origin_state_file)

    def _rosbag_command(self, action: str, label: str = "") -> dict:
        script = Path(self.config.rosbag_script).expanduser()
        if not script.is_file():
            if action == "status":
                return {"running": False, "available": False, "error": f"script not found: {script}"}
            raise ProtocolError("ROSBAG_UNAVAILABLE", f"rosbag script not found: {script}")
        args = [str(script), action]
        if label:
            args.append(label)
        try:
            result = subprocess.run(
                args,
                capture_output=True,
                text=True,
                timeout=max(10, int(self.config.rosbag_stop_timeout_seconds)),
            )
        except subprocess.TimeoutExpired as exc:
            raise ProtocolError("ROSBAG_COMMAND_TIMEOUT", f"rosbag {action} timed out") from exc
        output = (result.stdout or "").strip()
        if result.returncode != 0:
            message = (result.stderr or output or f"rosbag {action} failed").strip()
            if action == "status":
                return {"running": False, "available": True, "error": message}
            raise ProtocolError("ROSBAG_COMMAND_FAILED", message)
        try:
            payload = json.loads(output.splitlines()[-1]) if output else {}
        except (ValueError, IndexError):
            payload = {"running": action == "start", "error": "invalid recorder status"}
        payload["available"] = True
        return payload

    def _start_rosbag(self, label: str) -> None:
        status = self._rosbag_command("start", label)
        self._rosbag_dir = status.get("bag_dir") or self._rosbag_dir

    def _stop_rosbag(self) -> None:
        status = self._rosbag_status()
        if status.get("bag_dir"):
            self._rosbag_dir = status.get("bag_dir")
        if not status.get("running"):
            return
        try:
            stopped = self._rosbag_command("stop")
            if stopped.get("bag_dir"):
                self._rosbag_dir = stopped.get("bag_dir")
        except ProtocolError:
            LOGGER.exception("failed to stop mapping rosbag recorder")

    def _rosbag_status(self) -> dict:
        return self._rosbag_command("status")

    def _ensure_slam_process(self) -> None:
        if self._any_slam_process_alive:
            return
        if self.config.mapping_unit:
            try:
                self._systemctl("start")
            except ProtocolError:
                LOGGER.warning("systemd start of %s failed; falling back to a direct mapping process", self.config.mapping_unit)
            if self._is_mapping_unit_active():
                self._wait_for_slam_services()
                return
            LOGGER.warning("mapping unit %s is not active; falling back to a direct mapping process", self.config.mapping_unit)
        self._start_slam_subprocess()
        self._wait_for_slam_services()

    def _start_slam_subprocess(self) -> None:
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
        time.sleep(2)
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

    def _wait_for_slam_services(self) -> None:
        deadline = time.monotonic() + max(20, int(self.config.command_timeout_seconds))
        last_error = "SLAM mapping services did not appear"
        while time.monotonic() < deadline:
            if not self._any_slam_process_alive:
                raise ProtocolError("MAPPING_SLAM_START_FAILED", "SLAM mapping process exited before services appeared")
            try:
                result = subprocess.run(
                    ["bash", "-lc", self._shell_prefix() + "ros2 service list"],
                    capture_output=True,
                    text=True,
                    timeout=8,
                )
            except subprocess.TimeoutExpired:
                last_error = "timed out listing ROS services"
                time.sleep(1)
                continue
            services = set((result.stdout or "").split())
            if self.config.start_service in services or self.config.service_name in services:
                return
            last_error = "waiting for /slam/start_mapping or /slam_state_service"
            time.sleep(1)
        raise ProtocolError("MAPPING_SLAM_START_FAILED", last_error)

    def _ensure_mapping_sensors(self) -> None:
        script = Path(self.config.sensor_start_script).expanduser()
        if not script.is_file():
            raise ProtocolError("MAPPING_SENSOR_SCRIPT_MISSING", f"mapping sensor script not found: {script}")
        try:
            result = subprocess.run(
                [str(script)],
                capture_output=True,
                text=True,
                timeout=max(10, int(self.config.sensor_start_timeout_seconds)),
            )
        except subprocess.TimeoutExpired as exc:
            raise ProtocolError("MAPPING_SENSOR_TIMEOUT", "timed out waiting for LiDAR/IMU data") from exc
        if result.returncode != 0:
            message = (result.stderr or result.stdout or "LiDAR/IMU startup failed").strip()
            raise ProtocolError("MAPPING_SENSOR_NOT_READY", message)

    def _echo_topic_once(self, topic: str, timeout_seconds: int = 4) -> dict:
        command = self._shell_prefix() + f"ros2 topic echo --once {topic}"
        try:
            result = subprocess.run(
                ["bash", "-lc", command],
                capture_output=True,
                text=True,
                timeout=timeout_seconds,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"{topic} 超时无数据") from exc
        if result.returncode != 0:
            raise RuntimeError((result.stderr or result.stdout or f"{topic} 读取失败").strip())
        body = (result.stdout or "").split("---", 1)[0].strip()
        payload = yaml.safe_load(body) if body else {}
        if not isinstance(payload, dict):
            raise RuntimeError(f"{topic} 消息格式无效")
        return payload

    def _sample_origin_topics(self) -> OriginSample:
        timeout = self.config.origin_topic_timeout_seconds
        fix = self._echo_topic_once(self.config.origin_fix_topic, timeout)
        pvh = self._echo_topic_once(self.config.origin_rtk_topic, timeout)
        ntrip_message = self._echo_topic_once(self.config.origin_ntrip_status_topic, timeout)
        ntrip_data = ntrip_message.get("data") or ""
        ntrip = yaml.safe_load(ntrip_data) if isinstance(ntrip_data, str) else ntrip_data
        if not isinstance(ntrip, dict):
            ntrip = {}
        bestnav = pvh.get("bestnav") or {}
        heading = pvh.get("heading") or {}
        fix_status = int(((fix.get("status") or {}).get("status")) or -1)
        position_type = int(bestnav.get("pos_type") or 0)
        ntrip_quality = str(ntrip.get("quality") or "")
        position_fixed = (
            fix_status >= 2
            and int(bestnav.get("p_sol_status", -1)) == 0
            and position_type in {48, 49, 50}
            and ntrip_quality == "rtk_fixed"
        )
        heading_fixed = int(heading.get("sol_status", -1)) == 0 and int(heading.get("heading_type") or 0) > 0
        lat_std = float(bestnav.get("lat_std") or ntrip.get("horizontal_std_m") or math.inf)
        lon_std = float(bestnav.get("lon_std") or ntrip.get("horizontal_std_m") or math.inf)
        header_stamp = pvh.get("header", {}).get("stamp", {})
        stamp_seconds = float(header_stamp.get("sec") or 0) + float(header_stamp.get("nanosec") or 0) / 1e9
        header_age = max(0.0, time.time() - stamp_seconds) if stamp_seconds > 1_000_000_000 else 0.0
        return OriginSample(
            latitude=float(bestnav.get("latitude_deg") or fix.get("latitude") or math.nan),
            longitude=float(bestnav.get("longitude_deg") or fix.get("longitude") or math.nan),
            altitude=float(bestnav.get("altitude_m") or fix.get("altitude") or 0.0),
            position_fixed=position_fixed,
            heading_fixed=heading_fixed,
            baseline_m=float(heading.get("base_line") or 0.0),
            heading_deg=float(heading.get("heading_deg") or 0.0),
            heading_std_deg=float(heading.get("heading_std") or math.inf),
            horizontal_std_m=max(lat_std, lon_std),
            age_seconds=max(float(ntrip.get("age_sec") or 0.0), header_age),
            ntrip_quality=ntrip_quality,
        )

    def _stop_conflicting_navigation_stack(self) -> None:
        """Stop localization/Nav2 so mapping owns the lidar, IMU, and map TF."""
        script = Path(self.config.navigation_script).expanduser()
        if not script.is_file():
            raise ProtocolError("MAPPING_STACK_STOP_FAILED", f"navigation script not found: {script}")
        result = subprocess.run(
            [str(script), "full-stop"],
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
        named = None
        if data == self.config.start_data and self.config.start_service:
            named = (self.config.start_service, self.config.start_service_type)
        elif data == self.config.save_data and self.config.save_service:
            named = (self.config.save_service, self.config.save_service_type)
        if named:
            try:
                return self._call_ros_service(named[0], named[1], "{}")
            except ProtocolError as exc:
                if exc.code != "MAPPING_SERVICE_FAILED":
                    raise
                LOGGER.warning("named mapping service %s unavailable; falling back to MapState", named[0])
        payload = "{data: %s}" % int(data)
        return self._call_ros_service(self.config.service_name, self.config.service_type, payload)

    def _call_ros_service(self, name: str, service_type: str, payload: str) -> str:
        command = (
            self._shell_prefix()
            + f"ros2 service call {name} {service_type} {payload!r}"
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

    @staticmethod
    def _read_json(path: Path) -> dict:
        if not path.exists():
            return {}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            return payload if isinstance(payload, dict) else {}
        except (OSError, ValueError):
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

    def _finalize_session_package(self, work_dir: Path, command: dict) -> dict:
        scene_scope = normalize_text(
            command.get("scene_scope")
            or (self.session.scene_scope if self.session else "")
            or self._scene_scope,
            "indoor",
        )
        self._merge_locked_origin_metadata(work_dir)
        try:
            manifest = finalize_map_package(
                work_dir,
                requested_scene_scope=scene_scope,
                bag_dir=self._rosbag_dir,
                raw_recording=str(self._rosbag_dir or ""),
            )
        except MapConstraintError as exc:
            raise ProtocolError(exc.code, exc.message) from exc
        # Scan-Context is generated after the SLAM save service returns. Feed
        # accepted candidates back into the still-running C++ node so the
        # authoritative GTSAM graph, rather than the Python SE2 fallback, owns
        # the final trajectory and map.pcd.
        if manifest.get("loop_status") == "accepted" and int(manifest.get("loop_closure_count") or 0) > 0:
            try:
                self._call_ros_service("/slam/global_optimize", "std_srvs/srv/Trigger", "{}")
                refreshed = self._read_json(work_dir / "map_manifest.json")
                if refreshed:
                    manifest = refreshed
            except ProtocolError as exc:
                # finalize_loop_closure has already rebuilt map.pcd from its
                # conservative SE2 result, so retain a usable offline fallback
                # while making the missing GTSAM handoff visible in logs.
                LOGGER.warning("C++ GTSAM global optimization handoff failed: %s", exc)
        return manifest

    def _merge_locked_origin_metadata(self, work_dir: Path) -> None:
        if not self.session or self.session.mapping_type != "outdoor":
            return
        exported_path = work_dir / "gnss_origin.yaml"
        if not self._origin_file.is_file() or not exported_path.is_file():
            raise ProtocolError("MAPPING_ORIGIN_REQUIRED", "outdoor map export is missing locked GNSS origin metadata")
        try:
            locked = yaml.safe_load(self._origin_file.read_text(encoding="utf-8")) or {}
            exported = yaml.safe_load(exported_path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as exc:
            raise ProtocolError("MAPPING_ORIGIN_LOCK_FAILED", f"cannot merge GNSS origin evidence: {exc}") from exc
        evidence_fields = {
            "schema", "origin_lock_session_id", "lock_duration_seconds", "position_spread_m",
            "sample_count", "heading_deg", "heading_std_deg", "baseline_m", "locked_at_unix", "enu_axis",
        }
        for key in evidence_fields:
            if key in locked:
                exported[key] = locked[key]
        # The lock-time coordinates are authoritative; SLAM adds ENU-map alignment fields.
        for key in ("datum", "origin_latitude", "origin_longitude", "origin_altitude"):
            if key in locked:
                exported[key] = locked[key]
        temporary = exported_path.with_suffix(exported_path.suffix + ".tmp")
        temporary.write_text(yaml.safe_dump(exported, allow_unicode=True, sort_keys=False), encoding="utf-8")
        os.replace(temporary, exported_path)

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
        trace_path = self._generate_mapping_trace(base, filtered_base)
        preview_path = self._generate_map_preview(filtered_base)
        version = time.strftime("%Y%m%d-%H%M%S")
        package_path = self.map_dir / f"map_package_{version}.zip"
        # Keep the ENU-to-map transform with the map package. Without this
        # file, an uploaded map cannot use RTK for initialization or fusion.
        upload_files = [
            "map.yaml",
            "map.pgm",
            "map.txt",
            "gnss_origin.yaml",
            "mapping_trace.json",
            "map_manifest.json",
            "recording_manifest.yaml",
            "trajectory_raw.csv",
            "trajectory_optimized.csv",
            "trajectory_covariance.json",
            "loop_closures.csv",
        ]
        if preview_path:
            upload_files.append(preview_path.name)
        if self.config.upload_point_cloud:
            upload_files.append("map.pcd")
        files = []
        extra_files = [
            "scan_context/index.json",
            "scan_context/loop_candidates.csv",
        ]
        with zipfile.ZipFile(package_path, "w", zipfile.ZIP_DEFLATED) as archive:
            for name in upload_files:
                path = filtered_base / name
                if path.exists():
                    archive.write(path, arcname=name)
                    files.append(name)
            for name in extra_files:
                path = filtered_base / name
                if path.exists():
                    archive.write(path, arcname=name)
                    files.append(name)
        # A rescue map must be inspected before it can replace the active
        # navigation map. Normal saves retain the existing auto-activation flow.
        if not is_rescue:
            self._refresh_current_map_links(filtered_base, list(self.REQUIRED_FILES + self.OPTIONAL_FILES))
        map_manifest = self._read_json(filtered_base / "map_manifest.json")
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
            "map_manifest": map_manifest,
            "coordinate_mode": map_manifest.get("coordinate_mode", ""),
            "scene_scope": map_manifest.get("scene_scope", ""),
            "localization_mode": map_manifest.get("localization_mode", ""),
            "origin_status": map_manifest.get("origin_status", ""),
        }
        return package_path, metadata

    def _generate_mapping_trace(self, source: Path, output: Path) -> Path | None:
        keyframe_csv = source / "keyframes" / "keyframes.csv"
        if not keyframe_csv.exists():
            return None
        gnss_metadata = {}
        gnss_path = source / "gnss_origin.yaml"
        if gnss_path.exists():
            gnss_metadata = yaml.safe_load(gnss_path.read_text(encoding="utf-8")) or {}
        alignment_locked = bool(int(gnss_metadata.get("alignment_locked") or 0))
        origin_lat = float(gnss_metadata.get("origin_latitude") or 0.0)
        origin_lon = float(gnss_metadata.get("origin_longitude") or 0.0)
        alignment_yaw = float(gnss_metadata.get("enu_to_map_yaw") or 0.0)
        offset_x = float(gnss_metadata.get("map_offset_x") or 0.0)
        offset_y = float(gnss_metadata.get("map_offset_y") or 0.0)
        earth_radius_m = 6378137.0
        samples = []
        def optional_number(value, digits):
            parsed = float(value or 0.0)
            return round(parsed, digits) if math.isfinite(parsed) else None

        with keyframe_csv.open(encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                try:
                    slam = {
                        "x": round(float(row.get("lidar_x") or row["x"]), 4),
                        "y": round(float(row.get("lidar_y") or row["y"]), 4),
                        "z": round(float(row.get("lidar_z") or row.get("z") or 0.0), 4),
                        "yaw": round(float(row.get("yaw") or 0.0), 5),
                        "qx": round(float(row.get("lidar_qx") or row.get("world_qx") or 0.0), 7),
                        "qy": round(float(row.get("lidar_qy") or row.get("world_qy") or 0.0), 7),
                        "qz": round(float(row.get("lidar_qz") or 0.0), 7),
                        "qw": round(float(row.get("lidar_qw") or 1.0), 7),
                    }
                    rtk_valid = row.get("rtk_valid") == "1"
                    latitude = float(row.get("rtk_latitude") or 0.0)
                    longitude = float(row.get("rtk_longitude") or 0.0)
                    rtk = {
                        "valid": rtk_valid,
                        "status": int(row.get("rtk_status") or -1),
                        "horizontal_std_m": optional_number(row.get("rtk_horizontal_std"), 3),
                        "age_s": optional_number(row.get("rtk_age_seconds"), 3),
                        "heading_valid": row.get("rtk_heading_valid") == "1",
                        "heading_deg": optional_number(row.get("rtk_heading_deg"), 3),
                        "heading_std_deg": optional_number(row.get("rtk_heading_std_deg"), 3),
                    }
                    if rtk_valid and alignment_locked and abs(latitude) > 1e-7 and abs(longitude) > 1e-7:
                        north = (latitude - origin_lat) * math.pi / 180.0 * earth_radius_m
                        east = (longitude - origin_lon) * math.pi / 180.0 * math.cos(origin_lat * math.pi / 180.0) * earth_radius_m
                        rtk["x"] = round(math.cos(alignment_yaw) * east - math.sin(alignment_yaw) * north + offset_x, 4)
                        rtk["y"] = round(math.sin(alignment_yaw) * east + math.cos(alignment_yaw) * north + offset_y, 4)
                        if rtk["heading_valid"]:
                            yaw_enu = math.pi / 2.0 - math.radians(rtk["heading_deg"])
                            rtk["yaw"] = round(math.atan2(math.sin(yaw_enu + alignment_yaw), math.cos(yaw_enu + alignment_yaw)), 5)
                    samples.append({
                        "index": int(row["index"]),
                        "stamp": round(float(row["stamp"]), 6),
                        "slam": slam,
                        "rtk": rtk,
                    })
                except (KeyError, TypeError, ValueError):
                    LOGGER.warning("Skipping malformed mapping trace row: %s", row)
        if not samples:
            return None
        output.mkdir(parents=True, exist_ok=True)
        trace_path = output / "mapping_trace.json"
        trace_path.write_text(json.dumps({
            "format": "roamerx.mapping-trace.v1",
            "frame_id": "map",
            "alignment_locked": alignment_locked,
            "samples": samples,
        }, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
        return trace_path

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
            self._persist_workflow_state()

    def _stop_slam_process(self) -> None:
        if self.config.mapping_unit:
            try:
                self._systemctl("stop")
            except ProtocolError:
                LOGGER.warning("systemd stop of %s failed; stopping leftover mapping processes", self.config.mapping_unit)
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

    def _systemctl(self, action: str) -> dict:
        completed = subprocess.run(
            ["sudo", "systemctl", action, self.config.mapping_unit],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=max(15, int(self.config.command_timeout_seconds)),
        )
        payload = {
            "action": action,
            "returncode": completed.returncode,
            "stdout": (completed.stdout or "")[-4000:],
            "stderr": (completed.stderr or "")[-4000:],
        }
        if completed.returncode != 0:
            raise ProtocolError("MAPPING_UNIT_FAILED", payload["stderr"] or payload["stdout"] or f"systemctl {action} failed")
        return payload

    def _is_mapping_unit_active(self) -> bool:
        if not self.config.mapping_unit:
            return False
        process = subprocess.run(
            ["systemctl", "is-active", "--quiet", self.config.mapping_unit],
            check=False,
            timeout=2,
        )
        return process.returncode == 0

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
