from __future__ import annotations

import contextlib
import csv
import hashlib
import json
import logging
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import tempfile
import threading
import time
import uuid
import zipfile
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

import yaml

LOGGER = logging.getLogger(__name__)

SCAN_CONTEXT_TOTAL_RE = re.compile(
    r"TOTAL:\s+(?P<answerable>\d+)/(?P<queries>\d+) answerable\s+"
    r"top1 (?P<top1>[0-9.]+)%\s+topk (?P<topk>[0-9.]+)%\s+"
    r"top1_err_med (?P<position>[0-9.]+) m\s+"
    r"yaw_err_med (?P<yaw_med>[0-9.]+) deg\s+"
    r"yaw_err_p90 (?P<yaw_p90>[0-9.]+) deg\s+"
    r"dist_hit_p90 (?P<hit_distance>[0-9.]+)\s+"
    r"dist_miss_med (?P<miss_distance>[0-9.]+)"
)


def _parse_scan_context_check(output: str) -> dict:
    match = SCAN_CONTEXT_TOTAL_RE.search(output or "")
    if not match:
        return {}
    values = match.groupdict()
    return {
        "queries": int(values["queries"]),
        "answerable": int(values["answerable"]),
        "top1_hit_percent": float(values["top1"]),
        "topk_hit_percent": float(values["topk"]),
        "top1_position_error_median_m": float(values["position"]),
        "yaw_error_median_deg": float(values["yaw_med"]),
        "yaw_error_p90_deg": float(values["yaw_p90"]),
        "descriptor_hit_p90": float(values["hit_distance"]),
        "descriptor_miss_median": float(values["miss_distance"]),
    }


def _parse_post_save_localization_check(output: str) -> dict:
    """Return the last structured checker record while tolerating ROS log lines."""
    for line in reversed((output or "").splitlines()):
        try:
            payload = json.loads(line)
        except (TypeError, json.JSONDecodeError):
            continue
        if isinstance(payload, dict) and payload.get("schema") == "roamerx.post-save-localization-check.v1":
            return payload
    return {}


def _atomic_write_text(path: Path, text: str) -> None:
    """Write then replace using a unique tmp name so concurrent persist cannot steal the file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_name, path)
    except Exception:
        try:
            os.unlink(tmp_name)
        except OSError:
            pass
        raise



from .config import MappingConfig
from . import __version__ as edge_agent_version
from .keyframe_visibility_filter import filter_with_keyframe_visibility
from .map_coordinate import MapConstraintError, SCENE_SCOPES, normalize_text
from .map_optimization_summary import build_optimization_summary, summary_without_corrections
from .map_loop_closure import optimize_reviewed_loop_closures
from .map_version_pointer import MapVersionPointerError, activate_map_version
from .map_package_finalize import finalize_map_package
from .map_preview import generate_map_preview
from .media_client import MediaClient
from .origin_lock import OriginLockMonitor, OriginSample
from .protocol import ProtocolError, now_iso
from .rtk_origin import build_origin_sample


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
    origin_session_id: str = ""
    origin_sha256: str = ""


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
        "map_raw.pgm",
        "map_raw.yaml",
        "map_raw.txt",
        "map_optimized_rejected.pcd",
        "map_optimized_rejected.pgm",
        "map_optimized_rejected.yaml",
        "map_optimized_rejected.txt",
        "map_preview.png",
        "preview.png",
        "map.txt",
        "gnss_origin.yaml",
        "mapping_trace.json",
        "map_manifest.json",
        "recording_manifest.yaml",
        "trajectory_raw.csv",
        "trajectory_optimized.csv",
        "optimization_summary.json",
        "localization_validation.json",
        "divergence_event.json",
        "rescue_metadata.json",
        "save_progress.json",
    )
    SAVE_OUTPUT_TIMEOUT_SECONDS = 7200
    SLAM_MAPPING_EXECUTABLE_RE = re.compile(
        r"^\S*/robot_slam/lib/robot_slam/mapping(?:\s|$)"
    )
    SLAM_LAUNCH_PROCESS_RE = re.compile(
        r"^(?:\S*/python(?:3(?:\.\d+)?)?\s+)?"
        r"(?:\S*/)?ros2\s+launch\s+robot_slam(?:\s|$)"
    )
    LIO_ODOMETRY_PROCESS_MARKERS = (
        "lio_odometry.launch",
        "__node:=lio_odometry",
        "frontend.odometry_only:=true",
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
        self._origin_payload_provider = None
        self._scene_scope = "indoor"
        self._mapping_type = "indoor"
        self._state_lock = threading.RLock()
        self._origin_file = Path(config.origin_file).expanduser() if config.origin_file else self.map_dir / "gnss_origin.yaml"
        self._global_enu_file = self._origin_file.parent / "global_enu.yaml"
        self._post_save_validation_file = self.map_dir / "post_save_validation.json"
        self._post_save_validation_lock = threading.RLock()
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
            heading_offset_deg=config.heading_offset_deg,
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
        if self.session and self.session.state in {"starting", "mapping", "saving", "optimizing", "packaging", "uploading"}:
            if self._slam_process_alive:
                raise ProtocolError("MAPPING_ALREADY_ACTIVE", "mapping session is already active")
            LOGGER.warning("Stale session '%s' (state=%s) detected with dead SLAM process — cleaning up", self.session.session_id, self.session.state)
            self._cleanup()
        if not self.session and self._any_slam_process_alive:
            # No session but a live mapping node: either a mapping run whose
            # session was lost across an edge-agent restart, or a leftover from a
            # failed export.  We cannot tell the two apart here, so refuse rather
            # than kill — but say how to clear it, since mapping.cancel is the
            # only path that unconditionally stops the node.
            raise ProtocolError(
                "MAPPING_ALREADY_ACTIVE",
                "mapping process is already running without a tracked session; "
                "send mapping.cancel (or `mapping_cli.py stop`) to stop it before starting a new run",
            )
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
        try:
            self._configure_mapping_recording(command, map_name)
            self._ensure_slam_process()
            if mapping_type == "outdoor":
                self._call_map_state(self.config.warmup_data)
            self._call_map_state(self.config.start_data)
        except Exception:
            self._cleanup()
            raise
        self._set_state("mapping")
        return self.status()

    def start_origin_lock(self, command: dict) -> dict:
        """Start outdoor sensors, optionally stopping before the RTK quality window."""
        if self._any_slam_process_alive:
            raise ProtocolError("MAPPING_ALREADY_ACTIVE", "stop SLAM before locking a new ENU origin")
        prepare_only = bool(command.get("prepare_only"))
        existing_origin_session = bool(self.session and self.session.state == "origin_waiting")
        if self.session and self.session.state not in {"idle", "cancelled", "exited", "failed", "origin_waiting"}:
            raise ProtocolError("MAPPING_ALREADY_ACTIVE", "mapping workflow is already active")
        if existing_origin_session and not prepare_only:
            origin_state = self._origin_monitor.status().get("origin_status")
            if origin_state in {"waiting_fix", "quality_holding"}:
                raise ProtocolError("MAPPING_ALREADY_ACTIVE", "ENU origin lock is already running")
            if origin_state == "locked":
                return self.status()
        scene_scope = normalize_text(command.get("scene_scope"), "outdoor")
        if scene_scope not in {"transition", "outdoor"}:
            raise ProtocolError("MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN", "indoor mapping does not lock an ENU origin")
        self._mapping_type = "outdoor"
        self._scene_scope = scene_scope
        if not existing_origin_session:
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
        try:
            if not existing_origin_session:
                self._stop_conflicting_navigation_stack(preserve_localization=True)
                self._ensure_mapping_sensors()
                self._configure_mapping_recording(command, self.session.map_name)
                self._ensure_origin_odometry()
            else:
                # The second request starts the actual quality window after a
                # prepare_only request. Preserve the checkbox state and ensure
                # the bag is already running before sampling the origin.
                self._configure_mapping_recording(command, self.session.map_name)
            if prepare_only:
                self._origin_monitor.prepare()
            else:
                self._origin_monitor.start()
        except Exception:
            self._stop_rosbag()
            if not existing_origin_session:
                self._cleanup()
            raise
        self._set_state("origin_waiting")
        return self.status()

    def cancel_origin_lock(self, command: dict) -> dict:
        del command
        origin = self._origin_monitor.cancel()
        if self.session and self.session.state in {"origin_starting", "origin_waiting", "origin_locked"}:
            self._set_state("cancelled")
        result = self.status()
        result["origin"] = origin
        self._cleanup()
        result.update(self.status())
        return result

    def extract_global_enu(self, command: dict) -> dict:
        """Persist the selected locked map origin as the robot-wide ENU config."""
        supplied = command.get("global_enu") or {}
        origin = supplied if isinstance(supplied, dict) else {}
        if not origin:
            origin = self._origin_monitor.status().get("origin") or {}
        if not origin.get("alignment_locked"):
            raise ProtocolError("MAPPING_ORIGIN_REQUIRED", "当前地图没有有效的锁定 ENU 原点")
        payload = {
            "schema": "roamerx.global-enu.v1",
            "source_map_id": command.get("source_map_id", ""),
            "source_map_name": command.get("source_map_name", ""),
            "extracted_at": now_iso(),
            **{key: origin.get(key) for key in (
                "origin_latitude", "origin_longitude", "origin_altitude",
                "enu_axis", "enu_to_map_yaw", "map_offset_x", "map_offset_y",
                "heading_deg", "heading_std_deg", "position_spread_m",
                "heading_confirmed", "heading_confirmation_source",
                "heading_confirmed_at_unix", "confirmed_heading_deg",
                "confirmed_receiver_heading_deg", "confirmed_enu_yaw_deg",
                "confirmed_heading_std_deg", "confirmed_baseline_m", "heading_offset_deg",
                "origin_lock_session_id", "locked_at_unix", "alignment_locked",
            ) if origin.get(key) is not None},
        }
        self._global_enu_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._global_enu_file.with_suffix(self._global_enu_file.suffix + ".tmp")
        temporary.write_text(yaml.safe_dump(payload, allow_unicode=True, sort_keys=False), encoding="utf-8")
        os.replace(temporary, self._global_enu_file)
        return {"global_enu": payload, "global_enu_file": str(self._global_enu_file)}

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
        self._set_state("slam_starting")
        try:
            self._configure_mapping_recording(command, self.session.map_name)
            if mapping_type == "outdoor":
                self._stop_localization_for_slam()
            self._ensure_slam_process()
            self._call_map_state(
                self.config.warmup_data if mapping_type == "outdoor" else self.config.indoor_warmup_data
            )
        except Exception:
            self._stop_rosbag()
            self._set_state("failed")
            self._cleanup()
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
            if not bool(command.get("heading_check_confirmed")):
                raise ProtocolError("MAPPING_HEADING_NOT_CONFIRMED", "operator must confirm the heading check")
        # Keep this as a defensive fallback for clients that call begin
        # directly. Normal outdoor/indoor workflows already started the bag
        # during origin preparation or SLAM warmup.
        try:
            self._configure_mapping_recording(command, self.session.map_name)
            self._call_map_state(self.config.start_data)
            if self.session.mapping_type == "outdoor":
                self._origin_monitor.confirm_heading_review()
                self.session.heading_check_confirmed = True
                alignment = self._wait_for_heading_alignment()
                if alignment.get("locked"):
                    LOGGER.info(
                        "ENU-map yaw locked from %s after heading confirmation",
                        alignment.get("source") or "unknown",
                    )
                else:
                    LOGGER.warning(
                        "ENU-map yaw is not locked yet; GNSS fusion waits for dual-antenna heading "
                        "or a 15 m trajectory fit"
                    )
        except Exception:
            self._stop_rosbag()
            raise
        self.session.mapping_capture_enabled = True
        # Keep the persistent /rtk_pvh-backed monitor alive during formal
        # mapping so ENU X/Y/Yaw and quality telemetry continue to refresh.
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
            result["mapping_metrics"] = metadata.get("mapping_metrics", {})
            result["upload_result"] = upload_result
            return result

        if not self._any_slam_process_alive:
            LOGGER.warning(
                "Mapping session %s has no SLAM process; restarting for persistent-keyframe recovery",
                self.session.session_id,
            )
            self._ensure_slam_process()
        return self._save_active_mapping(command)

    def optimize_historical_map(self, command: dict, source_dir: Path) -> dict:
        """Create, validate and upload an immutable manually optimized revision."""
        if self.session or self._any_slam_process_alive:
            raise ProtocolError("MAPPING_ACTIVE", "建图进程运行中，不能执行离线地图优化")
        source_dir = source_dir.resolve()
        required = ["map.pcd", "map.pgm", "map.yaml", "keyframes/keyframes.csv", "scan_context/loop_candidates.csv"]
        missing = [name for name in required if not (source_dir / name).is_file()]
        if missing:
            raise ProtocolError("MAP_OFFLINE_DATA_MISSING", f"离线优化缺少文件: {', '.join(missing)}")
        if not list((source_dir / "keyframes").glob("scan_*.pcd")):
            raise ProtocolError("MAP_KEYFRAME_CLOUDS_MISSING", "离线优化需要机器狗本地关键帧点云，云端精简包不能单独执行")
        stamp = time.strftime("%Y%m%d_%H%M%S")
        output_dir = self.map_dir / f"{stamp}_{int(time.time_ns() / 1_000_000) % 1000:03d}"
        if output_dir.exists():
            raise ProtocolError("MAP_VERSION_EXISTS", f"目标地图版本已存在: {output_dir.name}")
        try:
            shutil.copytree(source_dir, output_dir, symlinks=False)
            thresholds = dict(command.get("thresholds") or {})
            thresholds["source_map_id"] = str(command.get("source_map_id") or command.get("map_id") or "")
            summary = optimize_reviewed_loop_closures(
                output_dir,
                list(command.get("selected_candidates") or []),
                thresholds,
                grid_converter=self.config.pcd2grid_binary,
            )
            self._run_post_save_localization_validation(output_dir)
            package_command = dict(command)
            package_command.update(
                map_name=command.get("output_map_name") or f"{source_dir.name}-回环优化",
                _manual_optimization=True,
                parent_map_id=str(command.get("source_map_id") or command.get("map_id") or ""),
                activate_after_upload=bool(command.get("activate_after_upload", False)),
            )
            package_path, metadata = self._package_map(package_command, output_dir)
            metadata["parent_map_id"] = package_command["parent_map_id"]
            metadata["optimization_review"] = {
                "confirmed_at": command.get("review_confirmed_at"),
                "thresholds": command.get("thresholds") or {},
                "selected_candidate_ids": summary.get("selected_candidate_ids") or [],
            }
            upload_result = self.media_client.upload_map_package(str(package_path), metadata)
            return {
                "source_map_id": package_command["parent_map_id"],
                "source_dir": str(source_dir),
                "output_dir": str(output_dir),
                "package_path": str(package_path),
                "optimization": summary_without_corrections(summary),
                "upload_result": upload_result,
                "activated": bool(metadata.get("auto_activate")),
            }
        except ProtocolError:
            raise
        except Exception as exc:
            LOGGER.exception("offline map optimization failed for %s", source_dir)
            raise ProtocolError("MAP_OFFLINE_OPTIMIZATION_FAILED", str(exc)) from exc

    def _save_active_mapping(self, command: dict) -> dict:
        saved_mapping_type = (
            self.session.mapping_type
            if self.session and self.session.mapping_type in {"indoor", "outdoor"}
            else self._mapping_type
        )
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
        should_upload = bool(command.get("upload", True))
        should_package = bool(command.get("package", should_upload))
        should_stop = bool(command.get("stop_process", True))
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
        # Past this point the save service has returned, so the map is already on
        # disk and SLAM has nothing left to do.  Every step below can raise, and
        # letting one of them escape used to leak the mapping node: it kept a core
        # and ~230 MB busy, and the *next* start_mapping was then rejected with
        # MAPPING_ALREADY_ACTIVE until somebody thought to send mapping.cancel.
        # Export failures must not cost the operator the next mapping run.
        try:
            # SLAM writes yaml/pgm asynchronously after the save service returns.
            # Wait for an output touched after this save command instead of falling
            # back to an older complete map directory.
            work_dir = self._wait_for_complete_map_dir(save_started_at)
            self._validate_map_files(work_dir)
            self._finalize_session_package(work_dir, command)
            result = self.status()
            if should_package:
                self._set_state("packaging")
                package_path, metadata = self._package_map(command, work_dir)
                result["package_path"] = str(package_path)
                result["mapping_metrics"] = metadata.get("mapping_metrics", {})
                if should_upload:
                    self._set_state("uploading")
                    result["upload_result"] = self.media_client.upload_map_package(str(package_path), metadata)
        except Exception:
            if should_stop:
                self._set_state("stopping")
                # Suppressed so a failing teardown cannot mask the export error
                # that actually explains what went wrong.  Separate blocks: a
                # recorder that refuses to stop must not cost us the SLAM stop,
                # which is the whole point of this handler.
                with contextlib.suppress(Exception):
                    self._stop_rosbag()
                with contextlib.suppress(Exception):
                    self._stop_slam_process()
            self._set_state("failed")
            raise
        if should_stop:
            self._set_state("stopping")
            self._stop_rosbag()
            self._stop_slam_process()
            self._set_state("exited")
        result.update(self.status())
        # A completed export must not keep the previous mapping mode, session,
        # or ENU lock alive.  The packaged metadata remains in ``result`` while
        # the adapter is reset so the next run can choose indoor/outdoor again.
        if should_stop:
            self._cleanup()
            result.update(self.status())
        # Start the post-save static self-check only after the map is durable and
        # the mapping session has been torn down. It may seed localization but
        # never starts Nav2 or publishes a velocity command.
        self._start_post_save_validation(
            str(work_dir),
            str(command.get("mapping_type") or saved_mapping_type or "indoor"),
        )
        result.update(self.status())
        if should_stop:
            # The command result is the durable terminal snapshot consumed by
            # the platform.  Both cleanup and queued post-save validation merge
            # live status into the result, so write the terminal state last.
            result["state"] = "exited"
        return result

    def _read_post_save_validation(self) -> dict:
        value = self._read_json(self._post_save_validation_file)
        return value if isinstance(value, dict) else {
            "indoor": {"required": 10, "success": 0, "attempts": 0, "history": []},
            "outdoor": {"required": 5, "success": 0, "attempts": 0, "history": []},
            "state": "idle",
        }

    def _start_post_save_validation(self, map_dir: str, mapping_type: str) -> None:
        kind = "outdoor" if str(mapping_type).lower() == "outdoor" else "indoor"
        validation_id = str(uuid.uuid4())
        with self._post_save_validation_lock:
            payload = self._read_post_save_validation()
            payload.setdefault(kind, {"required": 5 if kind == "outdoor" else 10, "success": 0, "attempts": 0, "history": []})
            payload.update({
                "state": "queued",
                "mapping_type": kind,
                "map_dir": map_dir,
                "validation_id": validation_id,
                "detail": "",
                "result": {},
                "updated_at": now_iso(),
            })
            _atomic_write_text(self._post_save_validation_file, json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
        threading.Thread(
            target=self._run_post_save_validation,
            args=(map_dir, kind, validation_id),
            daemon=True,
        ).start()

    def _run_post_save_validation(
        self,
        map_dir: str,
        kind: str,
        validation_id: str = "",
    ) -> None:
        with self._post_save_validation_lock:
            payload = self._read_post_save_validation()
            if validation_id and payload.get("validation_id") != validation_id:
                return
            payload.update({"state": "running", "mapping_type": kind, "map_dir": map_dir, "updated_at": now_iso()})
            _atomic_write_text(self._post_save_validation_file, json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
        command = os.environ.get("ROAMERX_POST_SAVE_VALIDATION_CMD", "").strip()
        checker_argv = None
        if not command:
            checker = Path(__file__).resolve().parents[2] / "robot" / "script" / "robot" / "post_save_static_localization_check.py"
            checker_argv = [
                "python3",
                str(checker),
                "--mapping-type",
                kind,
                "--map-dir",
                map_dir,
            ]
        result_state, detail = ("unavailable", "未配置真实静止定位验证命令")
        structured_result = {}
        candidate_localization_started = False
        navigation_script = None
        prepare_environment = None
        if command or checker_argv:
            try:
                navigation_script = Path(self.config.navigation_script).expanduser()
                if not navigation_script.is_file():
                    raise RuntimeError(f"navigation script not found: {navigation_script}")
                map_path = Path(map_dir).expanduser().resolve()
                missing_files = [
                    name for name in ("map.pcd", "map.yaml", "map.txt", "map_manifest.json")
                    if not (map_path / name).is_file()
                ]
                if missing_files:
                    raise RuntimeError(
                        f"saved map is incomplete for localization check: {', '.join(missing_files)}"
                    )
                prepare_environment = os.environ.copy()
                prepare_environment.update({
                    "PCD_MAP": str(map_path / "map.pcd"),
                    "MAP_YAML": str(map_path / "map.yaml"),
                })
                prepared = subprocess.run(
                    [str(navigation_script), "restart-localization"],
                    capture_output=True,
                    text=True,
                    timeout=max(90, self.config.command_timeout_seconds),
                    env=prepare_environment,
                )
                if prepared.returncode != 0:
                    raise RuntimeError(
                        (prepared.stderr or prepared.stdout or "本次地图定位栈启动失败").strip()[-1000:]
                    )
                candidate_localization_started = True
                completed = subprocess.run(
                    checker_argv or command.format(
                        map_dir=shlex.quote(str(map_path)),
                        mapping_type=shlex.quote(kind),
                    ),
                    shell=checker_argv is None,
                    capture_output=True,
                    text=True,
                    timeout=45,
                )
                combined_output = "\n".join(part for part in (completed.stdout, completed.stderr) if part)
                structured_result = _parse_post_save_localization_check(combined_output)
                if structured_result:
                    declared_state = str(structured_result.get("state") or "")
                    declared_map_dir = str(structured_result.get("map_dir") or "")
                    declared_map_matches = bool(declared_map_dir) and (
                        Path(declared_map_dir).expanduser().resolve() == map_path
                    )
                    declared_type_matches = structured_result.get("mapping_type") == kind
                    passed = bool(
                        completed.returncode == 0
                        and declared_state == "passed"
                        and structured_result.get("accurate") is True
                        and declared_map_matches
                        and declared_type_matches
                    )
                    result_state = "passed" if passed else "failed"
                    if not declared_map_matches or not declared_type_matches:
                        structured_result = {
                            **structured_result,
                            "state": "failed",
                            "accurate": False,
                            "reason_code": "LOCALIZATION_CHECK_IDENTITY_MISMATCH",
                            "message": "自检输出的地图目录或建图类型与本次保存不一致",
                        }
                    detail = str(structured_result.get("message") or "")
                else:
                    result_state = "failed"
                    detail = "静止定位验证器未输出结构化结果"
                    structured_result = {
                        "schema": "roamerx.post-save-localization-check.v1",
                        "state": "failed",
                        "accurate": False,
                        "reason_code": "LOCALIZATION_CHECK_OUTPUT_INVALID",
                        "message": detail,
                        "mapping_type": kind,
                        "map_dir": str(map_path),
                        "frame_id": "map",
                        "raw_output": combined_output.strip()[-1000:],
                    }
            except Exception as exc:
                result_state, detail = "failed", str(exc)
                structured_result = {
                    "schema": "roamerx.post-save-localization-check.v1",
                    "state": "failed",
                    "accurate": False,
                    "reason_code": "LOCALIZATION_CHECK_START_FAILED",
                    "message": detail,
                    "mapping_type": kind,
                    "map_dir": map_dir,
                    "frame_id": "map",
                }
        if result_state != "passed" and candidate_localization_started and navigation_script:
            try:
                stopped = subprocess.run(
                    [str(navigation_script), "stop-localization"],
                    capture_output=True,
                    text=True,
                    timeout=45,
                    env=prepare_environment,
                )
                stop_error = (
                    (stopped.stderr or stopped.stdout or "候选定位栈停止失败").strip()[-500:]
                    if stopped.returncode != 0
                    else ""
                )
            except Exception as exc:
                stop_error = f"候选定位栈停止失败: {exc}"
            if stop_error:
                detail = f"{detail}；{stop_error}" if detail else stop_error
                structured_result["cleanup_error"] = stop_error
        with self._post_save_validation_lock:
            payload = self._read_post_save_validation()
            bucket = payload.setdefault(kind, {"required": 5 if kind == "outdoor" else 10, "success": 0, "attempts": 0, "history": []})
            bucket["attempts"] = int(bucket.get("attempts", 0)) + 1
            if result_state == "passed": bucket["success"] = int(bucket.get("success", 0)) + 1
            history_entry = {
                "at": now_iso(),
                "map_dir": map_dir,
                "state": result_state,
                "detail": detail,
                "validation_id": validation_id,
            }
            if structured_result:
                history_entry["result"] = structured_result
            bucket.setdefault("history", []).append(history_entry)
            bucket["history"] = bucket["history"][-20:]
            if not validation_id or payload.get("validation_id") == validation_id:
                payload.update({
                    "state": result_state,
                    "detail": detail,
                    "result": structured_result,
                    "mapping_type": kind,
                    "map_dir": map_dir,
                    "updated_at": now_iso(),
                })
            _atomic_write_text(self._post_save_validation_file, json.dumps(payload, ensure_ascii=False, indent=2) + "\n")

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
            mapping_metrics=metadata.get("mapping_metrics", {}),
            upload_result=upload_result,
        )
        self._cleanup()
        result.update(self.status())
        return result

    def auto_rescue_diverged_mapping(self, event: dict) -> dict:
        """Rescue a flushed SAFE_HOLD session without waiting for a cloud save command."""
        if not self.session:
            raise ProtocolError("MAPPING_RESCUE_FAILED", "no active mapping session for automatic rescue")
        if self.session.state in {"recovering", "packaging", "uploading", "exited"}:
            return self.status()
        source = Path(str(event.get("map_dir") or "")).expanduser()
        if not source.is_dir() or source.parent.resolve() != self.map_dir.resolve():
            raise ProtocolError("MAPPING_RESCUE_FAILED", "SAFE_HOLD map directory is outside mapping storage")
        progress = self._read_save_progress(source)
        if progress.get("error_code") != "SLAM_DIVERGED":
            raise ProtocolError("MAPPING_RESCUE_FAILED", "SAFE_HOLD progress does not contain SLAM_DIVERGED")
        # Keep a short post-trigger tail so the diagnostic bag contains both
        # the rejected frames and the sensor behaviour immediately afterward.
        post_record_seconds = max(0.0, float(self.config.divergence_post_record_seconds))
        if post_record_seconds:
            time.sleep(post_record_seconds)
        self._stop_rosbag()
        self._enrich_divergence_event(source, event, post_record_seconds)
        return self._rescue_diverged_mapping({"upload": True, "stop_process": True}, source)

    def _enrich_divergence_event(
        self, source: Path, event: dict, post_record_seconds: float
    ) -> None:
        path = source / "divergence_event.json"
        try:
            payload = self._read_json(path)
            if not isinstance(payload, dict):
                payload = {}
            payload.update({
                "map_dir": str(source.resolve()),
                "diagnostic_rosbag_dir": str(Path(self._rosbag_dir).expanduser())
                if self._rosbag_dir else "",
                "post_trigger_record_seconds": post_record_seconds,
                "writer_flushed": bool(event.get("writer_flushed")),
            })
            _atomic_write_text(
                path,
                json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            )
        except (OSError, TypeError, ValueError):
            LOGGER.exception("failed to enrich mapping divergence evidence at %s", path)

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
        result = self.status()
        self._cleanup()
        result.update(self.status())
        return result

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
        optimization = self._read_optimization_status(latest_session_dir)
        post_save_validation = self._read_post_save_validation()
        rosbag = self._rosbag_status()
        global_enu = self._read_global_enu()
        if not self.session:
            progress_stage = str(progress.get("stage") or "")
            manifest = self._read_json(latest_session_dir / "map_manifest.json") if latest_session_dir else {}
            output_complete = bool(
                progress_stage == "completed"
                or (
                    manifest.get("completeness") == "complete"
                    and progress_stage != "failed"
                    and not progress.get("error_code")
                )
            )
            save_stages = {
                "recovering",
                "flushing_keyframes",
                "filtering",
                "partitioning_filter",
                "writing_pcd",
                "building_grid",
                "writing_metadata",
            }
            recovered_state = (
                "idle"
                if output_complete and not process_alive
                else "saving"
                if progress_stage in save_stages
                else "mapping"
                if process_alive
                else "idle"
            )
            return {
                "state": recovered_state,
                "map_dir": str(self.map_dir),
                "active_map_dir": str(latest_session_dir or self.map_dir),
                "latest_session_dir": str(latest_session_dir) if latest_session_dir else None,
                "process_alive": process_alive,
                "slam_pids": self._find_slam_process_pids(),
                "slam_log_path": str(self._slam_log_path) if self._slam_log_path else None,
                "save_progress": progress,
                "rtk_alignment": progress.get("rtk_alignment") or {},
                "readiness": readiness,
                "ready_for_motion": readiness["ready_for_motion"],
                "ready_for_save": readiness["ready_for_save"],
                "files": files,
                "optimization": optimization,
                "post_save_validation": post_save_validation,
                "rosbag": rosbag,
                "origin": origin,
                "global_enu": global_enu,
                "origin_status": origin.get("origin_status", "idle"),
                "mapping_type": self._mapping_type,
                "slam_process_alive": process_alive,
                "slam_warmup": False,
                "mapping_capture_enabled": False,
                "imu_initialized": bool(readiness.get("imu_initialized")),
                "slam_pose_ready": bool(readiness.get("slam_pose_ready")),
                "ready_for_mapping": bool(readiness.get("ready_for_mapping")),
            }
        origin = self._origin_monitor.status()
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
            if self.session.mapping_type == "outdoor":
                origin = self._origin_monitor.begin_heading_review()
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
            "rtk_alignment": progress.get("rtk_alignment") or {},
            "readiness": readiness,
            "ready_for_motion": readiness["ready_for_motion"],
            "ready_for_save": readiness["ready_for_save"],
            "files": files,
            "optimization": optimization,
            "post_save_validation": post_save_validation,
            "rosbag": rosbag,
            "origin": origin,
            "global_enu": global_enu,
            "origin_status": origin.get("origin_status", "idle"),
            "ready_for_mapping": bool(readiness.get("ready_for_mapping")),
        }

    def _read_optimization_status(self, session_dir: Path | None) -> dict:
        if not session_dir:
            return {}
        summary = self._read_json(session_dir / "optimization_summary.json")
        if summary:
            return summary_without_corrections(summary)
        return self._read_json(session_dir / "optimization_status.json")

    def _read_global_enu(self) -> dict:
        if not self._global_enu_file.is_file():
            return {}
        try:
            value = yaml.safe_load(self._global_enu_file.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError):
            return {}
        return value if isinstance(value, dict) else {}

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
            alignment = progress.get("rtk_alignment") or {}
            if alignment.get("locked"):
                source = str(alignment.get("source") or "")
                if source == "heading":
                    message = "正式采集中，ENU-地图航向已用双天线锁定"
                elif source == "trajectory":
                    message = "正式采集中，ENU-地图航向已用轨迹拟合锁定"
                else:
                    message = "正式采集中，ENU-地图航向已锁定"
            elif alignment.get("fusion_enabled"):
                message = "正式采集中；GNSS 尚未锁定航向，直线行走约 15 米后将用轨迹拟合"
            else:
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
        # ``stop`` leaves a valid ENU origin on disk.  That is useful only
        # while the current workflow is active; clear it when the workflow is
        # terminal so a subsequent run starts with an explicit mode selection.
        self._origin_monitor.cancel()
        self.session = None
        self._mapping_type = "indoor"
        self._scene_scope = "indoor"
        self._record_rosbag = False
        self._rosbag_dir = None

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
        payload = {
            "schema": "roamerx.mapping-workflow.v2",
            "session": asdict(self.session),
            "origin_status": self._origin_monitor.status().get("origin_status"),
            "updated_at": now_iso(),
        }
        try:
            _atomic_write_text(
                self._origin_state_file,
                json.dumps(payload, ensure_ascii=False, indent=2),
            )
        except OSError:
            LOGGER.exception("failed to persist mapping workflow state to %s", self._origin_state_file)

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

    def _configure_mapping_recording(self, command: dict, label: str) -> None:
        """Apply the recording checkbox and start the fixed diagnostic bag.

        The bag must cover the complete workflow, including outdoor RTK
        preparation/origin locking and SLAM warmup. Repeated workflow commands
        are expected, so the recorder script remains the single idempotent
        owner of duplicate-start handling.
        """
        if "record_rosbag" in command:
            requested = bool(command.get("record_rosbag"))
            if not requested and self._record_rosbag:
                # An explicit uncheck must not leave a recorder from an
                # earlier prepare request running in the background.
                self._record_rosbag = False
                self._stop_rosbag()
            else:
                self._record_rosbag = requested
        if not self._record_rosbag:
            return
        status = self._rosbag_status()
        if not status.get("running"):
            self._start_rosbag(label)

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
        self._prepare_mapping_runtime()
        self._verify_mapping_deployment()
        if self._any_slam_process_alive:
            # Adopting a live node instead of restarting it.  systemd only reads
            # mapping-session.env at unit start, so the mapping type and ENU
            # origin _prepare_mapping_runtime() just wrote do not take effect —
            # the adopted process keeps the previous session's values.  Say so,
            # because otherwise this is silent and the map comes out referenced
            # to the wrong origin.
            LOGGER.warning(
                "Adopting a live SLAM mapping process (unit_active=%s, pids=%s); "
                "runtime params just written apply only to a fresh start",
                self._is_mapping_unit_active(),
                self._find_slam_process_pids() or "none",
            )
            return
        if self.config.mapping_unit:
            self._systemctl("start")
            if self._is_mapping_unit_active():
                self._wait_for_slam_services()
                self._verify_mapping_publishers()
                return
            raise ProtocolError(
                "MAPPING_SLAM_START_FAILED",
                f"mapping unit {self.config.mapping_unit} did not become active",
            )
        self._start_slam_subprocess()
        self._wait_for_slam_services()

    def _prepare_mapping_runtime(self) -> dict:
        """Write the mode-scoped launch environment and strict outdoor origin parameters."""
        mapping_type = self.session.mapping_type if self.session else self._mapping_type
        environment_path = Path(self.config.mapping_environment_file).expanduser()
        params_path = Path(self.config.mapping_session_params_file).expanduser()
        origin_sha256 = ""
        origin_session_id = ""
        if mapping_type == "outdoor":
            if not self._origin_file.is_file():
                raise ProtocolError("MAPPING_ORIGIN_REQUIRED", "locked gnss_origin.yaml is missing")
            try:
                origin = yaml.safe_load(self._origin_file.read_text(encoding="utf-8")) or {}
                lat0 = float(origin["origin_latitude"])
                lon0 = float(origin["origin_longitude"])
                alt0 = float(origin["origin_altitude"])
            except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
                raise ProtocolError("MAPPING_ORIGIN_LOCK_FAILED", f"invalid locked ENU origin: {exc}") from exc
            if (
                not bool(origin.get("alignment_locked"))
                or not all(math.isfinite(value) for value in (lat0, lon0, alt0))
                or not -90.0 <= lat0 <= 90.0
                or not -180.0 <= lon0 <= 180.0
                or (abs(lat0) < 1e-12 and abs(lon0) < 1e-12 and abs(alt0) < 1e-9)
            ):
                raise ProtocolError("MAPPING_ORIGIN_LOCK_FAILED", "locked ENU origin is unlocked, non-finite or out of range")
            origin_session_id = str(origin.get("origin_lock_session_id") or "")
            monitor_session_id = str(
                ((self._origin_monitor.status().get("origin") or {}).get("origin_lock_session_id")) or ""
            )
            if not origin_session_id or (monitor_session_id and monitor_session_id != origin_session_id):
                raise ProtocolError("MAPPING_ORIGIN_LOCK_FAILED", "locked ENU origin session does not match this workflow")
            origin_sha256 = self._sha256_file(self._origin_file)
            params = {
                "slam_enu_converter": {
                    "ros__parameters": {
                        "lat0": lat0,
                        "lon0": lon0,
                        "alt0": alt0,
                        "origin_file": str(self._origin_file.resolve()),
                        "origin_session_id": origin_session_id,
                        "origin_sha256": origin_sha256,
                        "input_topic": self.config.origin_fix_topic,
                        "output_topic": "/gnss/enu_odom",
                        "max_age_seconds": float(self.config.heading_max_age_seconds),
                        "min_status": 1,
                    }
                }
            }
            _atomic_write_text(params_path, yaml.safe_dump(params, allow_unicode=True, sort_keys=False))
        else:
            params_path.unlink(missing_ok=True)
        environment = (
            f"ROAMERX_MAPPING_TYPE={mapping_type}\n"
            "ROAMERX_AUTO_LOOP_OPTIMIZATION="
            f"{'true' if self.config.auto_loop_optimization_enabled else 'false'}\n"
            f"ROAMERX_SLAM_PARAMS={Path(self.config.slam_params_file).expanduser()}\n"
            f"ROAMERX_MAPPING_ORIGIN_PARAMS={params_path if mapping_type == 'outdoor' else '/dev/null'}\n"
        )
        _atomic_write_text(environment_path, environment)
        if self.session:
            self.session.origin_session_id = origin_session_id
            self.session.origin_sha256 = origin_sha256
            self._persist_workflow_state()
        return {
            "mapping_type": mapping_type,
            "origin_session_id": origin_session_id,
            "origin_sha256": origin_sha256,
            "environment_file": str(environment_path),
            "origin_params_file": str(params_path) if mapping_type == "outdoor" else "",
        }

    @staticmethod
    def _sha256_file(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        return digest.hexdigest()

    def _verify_mapping_deployment(self) -> dict:
        manifest_path = Path(self.config.deployment_manifest).expanduser()
        if not manifest_path.is_file():
            if self.config.deployment_manifest_required:
                raise ProtocolError(
                    "MAPPING_DEPLOYMENT_MISMATCH",
                    f"mapping deployment manifest is missing: {manifest_path}",
                )
            return {}
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ProtocolError(
                "MAPPING_DEPLOYMENT_MISMATCH", f"invalid mapping deployment manifest: {exc}"
            ) from exc
        if not isinstance(manifest, dict):
            raise ProtocolError("MAPPING_DEPLOYMENT_MISMATCH", "mapping deployment manifest must be an object")
        configuration = manifest.get("configuration") or {}
        expected_unit = str(configuration.get("mapping_unit") or "")
        expected_command = str(configuration.get("slam_command") or "")
        if expected_unit != self.config.mapping_unit or expected_command != self.config.slam_command:
            raise ProtocolError(
                "MAPPING_DEPLOYMENT_MISMATCH",
                "mapping unit or SLAM command differs from the deployment manifest",
            )
        artifacts = manifest.get("artifacts") or {}
        required = {
            "slam_binary": Path(self.config.slam_binary).expanduser(),
            "slam_params_file": Path(self.config.slam_params_file).expanduser(),
            "mapping_adapter": Path(__file__).resolve(),
        }
        if self.config.mapping_unit:
            required.update({
                "enu_binary": Path(self.config.enu_binary).expanduser(),
                "unified_launch_file": Path(self.config.unified_launch_file).expanduser(),
                "mapping_unit_file": Path(self.config.mapping_unit_file).expanduser(),
            })
            if str(manifest.get("edge_agent_version") or "") != edge_agent_version:
                raise ProtocolError(
                    "MAPPING_DEPLOYMENT_MISMATCH", "Edge Agent version differs from deployment manifest"
                )
        for name, configured_path in required.items():
            record = artifacts.get(name) or {}
            recorded_path = Path(str(record.get("path") or "")).expanduser()
            expected_hash = str(record.get("sha256") or "").lower()
            if not configured_path.is_file() or recorded_path.resolve() != configured_path.resolve():
                raise ProtocolError(
                    "MAPPING_DEPLOYMENT_MISMATCH", f"{name} path does not match deployed artifact"
                )
            actual_hash = self._sha256_file(configured_path)
            if not expected_hash or actual_hash != expected_hash:
                raise ProtocolError(
                    "MAPPING_DEPLOYMENT_MISMATCH", f"{name} SHA256 does not match deployment manifest"
                )
        return manifest

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

    def _ros_node_names(self) -> set[str]:
        result = subprocess.run(
            ["bash", "-lc", self._shell_prefix() + "ROS2CLI_DISABLE_DAEMON=1 ros2 node list"],
            capture_output=True,
            text=True,
            timeout=8,
        )
        if result.returncode != 0:
            raise ProtocolError("MAPPING_STACK_STOP_FAILED", (result.stderr or "cannot inspect ROS nodes").strip())
        return set((result.stdout or "").split())

    def _verify_navigation_stack_stopped(self) -> None:
        forbidden = {
            "/localization", "/planner_server", "/controller_server", "/bt_navigator",
            "/behavior_server", "/waypoint_follower", "/navigo_container",
            "/lio_odometry",
        }
        remaining = sorted(name for name in self._ros_node_names() if name in forbidden)
        if remaining or self._find_slam_process_pids():
            detail = ", ".join(remaining) if remaining else "old SLAM process"
            raise ProtocolError("MAPPING_STACK_STOP_FAILED", f"mapping-conflicting process remains: {detail}")

    def _verify_mapping_publishers(self) -> None:
        nodes = self._ros_node_names()
        if "/mapping" not in nodes:
            raise ProtocolError("MAPPING_SLAM_START_FAILED", "mapping node is absent after service startup")
        if self._mapping_type == "outdoor" and "/slam_enu_converter" not in nodes:
            raise ProtocolError("MAPPING_SLAM_START_FAILED", "outdoor ENU converter node is absent")
        result = subprocess.run(
            ["bash", "-lc", self._shell_prefix() + "ROS2CLI_DISABLE_DAEMON=1 ros2 topic info -v /odom/localization_odom"],
            capture_output=True,
            text=True,
            timeout=8,
        )
        output = result.stdout or ""
        match = re.search(r"Publisher count:\s*(\d+)", output)
        publisher_nodes = {
            name.lstrip("/") for name in re.findall(r"Node name:\s*([^\s]+)", output)
        }
        if result.returncode != 0 or not match or int(match.group(1)) != 1 or "mapping" not in publisher_nodes:
            raise ProtocolError(
                "MAPPING_ODOMETRY_PUBLISHER_CONFLICT",
                "expected /odom/localization_odom to have exactly one publisher from /mapping",
            )

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
        sampled_at = None
        if self._origin_payload_provider is not None:
            fix, pvh, ntrip_message, sampled_at = self._origin_payload_provider()
        else:
            timeout = self.config.origin_topic_timeout_seconds
            fix = self._echo_topic_once(self.config.origin_fix_topic, timeout)
            pvh = self._echo_topic_once(self.config.origin_rtk_topic, timeout)
            ntrip_message = self._echo_topic_once(self.config.origin_ntrip_status_topic, timeout)
        return build_origin_sample(fix, pvh, ntrip_message, sampled_at=sampled_at)

    def set_origin_payload_provider(self, provider) -> None:
        """Use the Edge ROS node's persistent RTK subscriptions for sampling."""
        self._origin_payload_provider = provider

    def _run_navigation_script(self, action: str, error_code: str, error_message: str) -> None:
        script = Path(self.config.navigation_script).expanduser()
        if not script.is_file():
            raise ProtocolError("MAPPING_STACK_STOP_FAILED", f"navigation script not found: {script}")
        result = subprocess.run(
            [str(script), action],
            capture_output=True,
            text=True,
            timeout=max(15, self.config.command_timeout_seconds),
        )
        if result.returncode != 0:
            raise ProtocolError(
                error_code,
                (result.stderr or result.stdout or error_message).strip(),
            )

    def _stop_conflicting_navigation_stack(self, *, preserve_localization: bool = False) -> None:
        """Stop Nav2 and optionally preserve localization during RTK origin checks."""
        self._run_navigation_script(
            "stop" if preserve_localization else "full-stop",
            "MAPPING_STACK_STOP_FAILED",
            "failed to stop navigation stack",
        )
        if not preserve_localization:
            self._verify_navigation_stack_stopped()

    def _ensure_origin_odometry(self) -> None:
        self._run_navigation_script(
            "ensure-localization-odom",
            "MAPPING_ODOMETRY_NOT_READY",
            "localization odometry did not become ready for origin checks",
        )

    def _stop_localization_for_slam(self) -> None:
        self._run_navigation_script(
            "stop-localization",
            "MAPPING_LOCALIZATION_STOP_FAILED",
            "failed to release localization before starting SLAM",
        )
        self._verify_navigation_stack_stopped()

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

    def _wait_for_heading_alignment(self, timeout_seconds: float = 2.5) -> dict:
        deadline = time.monotonic() + max(0.0, float(timeout_seconds))
        alignment: dict = {}
        while True:
            progress = self._read_save_progress(
                self._find_latest_session_dir() or self._find_latest_progress_dir()
            )
            alignment = (progress or {}).get("rtk_alignment") or {}
            if alignment.get("locked") or time.monotonic() >= deadline:
                return alignment
            time.sleep(0.15)

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
            error_code="",
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

    def _mark_progress_completed(self, base: Path | None) -> None:
        """Close a validated export after optional post-save optimization."""
        progress = self._read_save_progress(base)
        if not base or not progress:
            return
        progress.update(
            stage="completed",
            progress_percent=100.0,
            recoverable=False,
            updated_at_unix=int(time.time()),
            error="",
        )
        path = base / "save_progress.json"
        try:
            _atomic_write_text(path, json.dumps(progress, ensure_ascii=False, indent=2) + "\n")
        except OSError:
            LOGGER.exception("failed to mark mapping progress completed: %s", path)

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
        self._write_optimization_status(work_dir, {
            "stage": "detecting",
            "success": None,
            "trigger_source": "automatic_save",
            "started_at_unix": round(time.time(), 3),
        })
        try:
            manifest = finalize_map_package(
                work_dir,
                requested_scene_scope=scene_scope,
                auto_loop_optimization_enabled=bool(
                    self.config.auto_loop_optimization_enabled
                ),
                bag_dir=self._rosbag_dir,
                raw_recording=str(self._rosbag_dir or ""),
            )
        except MapConstraintError as exc:
            raise ProtocolError(exc.code, exc.message) from exc
        # The C++ optimizer rewrites the complete navigation product set. Keep
        # the pre-optimization set so a quality-rejected loop can be rolled
        # back without mixing a raw cloud with an optimized grid/trajectory.
        self._snapshot_raw_map_products(work_dir)
        # Scan Context always writes descriptors and candidates. Loop factors
        # enter the graph only when the explicit automatic policy is enabled;
        # the default manual-review policy writes an empty loop_closures.csv.
        # Always invoke the C++ graph because LIO+IMU smoothing remains useful
        # independently of loop closure policy.
        optimization_started = time.monotonic()
        fallback_error = ""
        if int(manifest.get("keyframe_count") or 0) > 1:
            self._write_optimization_status(work_dir, {
                "stage": "optimizing",
                "success": None,
                "trigger_source": "automatic_save",
                "candidate_count": int(manifest.get("loop_candidate_count") or 0),
                "accepted_loop_count": int(manifest.get("loop_closure_count") or 0),
            })
            if self.session:
                self._set_state("optimizing")
            try:
                self._call_ros_service("/slam/global_optimize", "std_srvs/srv/Trigger", "{}")
                refreshed = self._read_json(work_dir / "map_manifest.json")
                if refreshed:
                    manifest = refreshed
            except ProtocolError as exc:
                fallback_error = str(exc)
                if scene_scope != "indoor":
                    raise ProtocolError(
                        "MAP_OPTIMIZATION_FAILED",
                        "outdoor GTSAM must apply RTK XY to the saved map: "
                        + str(exc),
                    ) from exc
                LOGGER.warning("C++ GTSAM global optimization handoff failed; keeping raw LIO map: %s", exc)
        optimization_duration = round(time.monotonic() - optimization_started, 3)
        timing = {
            "loop_detection_seconds": float(manifest.get("loop_detection_duration_seconds") or 0.0),
            "optimization_and_rebuild_seconds": optimization_duration,
        }
        summary = build_optimization_summary(
            work_dir,
            mapping_type="indoor" if scene_scope == "indoor" else "outdoor",
            timing=timing,
            fallback_error=fallback_error,
        )
        if scene_scope != "indoor":
            rtk_count = int((summary.get("factors") or {}).get("rtk_position") or 0)
            if not summary.get("applied") or rtk_count <= 0:
                raise ProtocolError(
                    "MAP_OPTIMIZATION_FAILED",
                    "outdoor GTSAM must apply RTK XY to the saved map: "
                    + str(
                        summary.get("fallback_error")
                        or f"applied={summary.get('applied')} rtk_position={rtk_count}"
                    ),
                )
        if summary.get("candidate_applied") and not summary.get("applied"):
            selection = self._restore_raw_map_products(work_dir)
            summary["map_output_selection"] = selection
            if not selection["restored"]:
                summary["stage"] = "fallback"
                summary["success"] = False
                summary["auto_activation_allowed"] = False
                missing = ", ".join(selection["missing"])
                summary["fallback_error"] = (
                    f"{summary.get('fallback_error')}; raw map rollback incomplete: {missing}"
                ).strip("; ")
            _atomic_write_text(
                work_dir / "optimization_summary.json",
                json.dumps(summary, ensure_ascii=False, indent=2),
            )
        compact_summary = summary_without_corrections(summary)
        manifest["optimization"] = compact_summary
        manifest["trajectory_source"] = summary.get("trajectory_source") or manifest.get("trajectory_source") or "raw"
        manifest["use_gps"] = bool(summary.get("use_gps"))
        # Persist the selected trajectory before the offline checker loads the
        # Scan Context database; optimized maps must return optimized seeds.
        _atomic_write_text(
            work_dir / "map_manifest.json",
            json.dumps(manifest, ensure_ascii=False, indent=2),
        )
        validation = self._run_post_save_localization_validation(work_dir)
        manifest["localization_validation"] = validation
        _atomic_write_text(
            work_dir / "map_manifest.json",
            json.dumps(manifest, ensure_ascii=False, indent=2),
        )
        self._write_optimization_status(work_dir, compact_summary)
        if manifest.get("completeness") == "complete":
            self._mark_progress_completed(work_dir)
        return manifest

    def _run_post_save_localization_validation(self, work_dir: Path) -> dict:
        """Measure offline relocalization readiness without auto-accepting it.

        Threshold decisions intentionally remain manual. This check verifies
        descriptor retrieval and optimized seed coordinates; it does not claim
        to be a live NDT initialization test because keyframe scans are already
        part of the exported map.
        """
        binary = Path(self.config.localization_scan_context_check_binary).expanduser()
        payload = {
            "schema": "roamerx.localization-validation.v1",
            "test_type": "offline_scan_context_leave_one_out",
            "threshold_policy": "manual_review",
            "live_ndt_initialization_tested": False,
            "measured_at_unix": round(time.time(), 3),
        }
        if not binary.is_file():
            payload.update(status="unavailable", error=f"checker not found: {binary}")
        else:
            try:
                completed = subprocess.run(
                    [
                        str(binary), "--quiet", "--top-k", "5", "--radius", "2.0",
                        "--min-gap", "1", str(work_dir),
                    ],
                    check=False,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    timeout=300,
                )
                metrics = _parse_scan_context_check(completed.stdout)
                payload.update(
                    status="measured" if completed.returncode == 0 and metrics else "failed",
                    returncode=completed.returncode,
                    metrics=metrics,
                    output=(completed.stdout or "")[-4000:],
                    error=(completed.stderr or "")[-2000:],
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                payload.update(status="failed", error=str(exc))
        _atomic_write_text(
            work_dir / "localization_validation.json",
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        )
        return payload

    @staticmethod
    def _snapshot_raw_map_products(work_dir: Path) -> dict:
        copied = []
        for active_name, raw_name in (
            ("map.pcd", "map_raw.pcd"),
            ("map.pgm", "map_raw.pgm"),
            ("map.yaml", "map_raw.yaml"),
            ("map.txt", "map_raw.txt"),
        ):
            source = work_dir / active_name
            target = work_dir / raw_name
            if source.is_file() and not target.exists():
                shutil.copy2(source, target)
                copied.append(raw_name)
        return {"copied": copied}

    @staticmethod
    def _restore_raw_map_products(work_dir: Path) -> dict:
        restored = []
        missing = []
        for active_name, raw_name, rejected_name in (
            ("map.pcd", "map_raw.pcd", "map_optimized_rejected.pcd"),
            ("map.pgm", "map_raw.pgm", "map_optimized_rejected.pgm"),
            ("map.yaml", "map_raw.yaml", "map_optimized_rejected.yaml"),
            ("map.txt", "map_raw.txt", "map_optimized_rejected.txt"),
        ):
            active = work_dir / active_name
            raw = work_dir / raw_name
            rejected = work_dir / rejected_name
            if not raw.is_file():
                missing.append(raw_name)
                continue
            if active.is_file() and not rejected.exists():
                shutil.copy2(active, rejected)
            shutil.copy2(raw, active)
            restored.append(active_name)
        return {
            "selected": "raw",
            "restored": not missing,
            "restored_files": restored,
            "missing": missing,
            "rejected_candidate_preserved": all(
                (work_dir / name).is_file()
                for name in (
                    "map_optimized_rejected.pcd",
                    "map_optimized_rejected.pgm",
                    "map_optimized_rejected.yaml",
                    "map_optimized_rejected.txt",
                )
            ),
        }

    @staticmethod
    def _write_optimization_status(work_dir: Path, payload: dict) -> None:
        value = dict(payload)
        value["updated_at_unix"] = round(time.time(), 3)
        _atomic_write_text(
            work_dir / "optimization_status.json",
            json.dumps(value, ensure_ascii=False, indent=2),
        )

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
            "heading_confirmed", "heading_confirmation_source", "heading_confirmed_at_unix",
            "confirmed_heading_deg", "confirmed_receiver_heading_deg", "confirmed_enu_yaw_deg",
            "confirmed_heading_std_deg", "confirmed_baseline_m", "heading_offset_deg",
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

    @staticmethod
    def _directory_size(base: Path | None) -> int:
        """Return the regular-file bytes in a map session directory."""
        if not base or not base.is_dir():
            return 0
        total = 0
        try:
            for path in base.rglob("*"):
                if path.is_file() and not path.is_symlink():
                    total += path.stat().st_size
        except OSError:
            LOGGER.warning("failed to calculate map directory size for %s", base, exc_info=True)
        return total

    @staticmethod
    def _count_keyframes(base: Path, manifest: dict, progress: dict) -> int:
        count = int(manifest.get("keyframe_count") or progress.get("keyframe_count") or progress.get("written_keyframes") or 0)
        if count > 0:
            return count
        csv_path = base / "keyframes" / "keyframes.csv"
        if not csv_path.is_file():
            return 0
        try:
            with csv_path.open("r", encoding="utf-8", newline="") as handle:
                return sum(1 for row in csv.DictReader(handle) if row)
        except (OSError, csv.Error):
            LOGGER.warning("failed to count keyframes in %s", csv_path, exc_info=True)
            return 0

    @staticmethod
    def _trajectory_meters(base: Path, progress: dict) -> float:
        reported = float(progress.get("trajectory_m") or 0)
        if reported > 0:
            return round(reported, 3)
        csv_path = base / "keyframes" / "keyframes.csv"
        if not csv_path.is_file():
            return 0.0
        distance = 0.0
        previous = None
        try:
            with csv_path.open("r", encoding="utf-8", newline="") as handle:
                for row in csv.DictReader(handle):
                    try:
                        x = float(row.get("world_x") or row.get("x") or 0)
                        y = float(row.get("world_y") or row.get("y") or 0)
                    except (TypeError, ValueError):
                        continue
                    if previous is not None:
                        distance += math.hypot(x - previous[0], y - previous[1])
                    previous = (x, y)
        except (OSError, csv.Error):
            LOGGER.warning("failed to calculate trajectory length from %s", csv_path, exc_info=True)
        return round(distance, 3)

    @staticmethod
    def _mapping_duration_seconds(started_at: str | None, progress: dict) -> float:
        recorded = float(progress.get("mapping_duration_seconds") or 0)
        if recorded > 0:
            return round(recorded, 1)
        if not started_at:
            return 0.0
        try:
            value = str(started_at).replace("Z", "+00:00")
            started = datetime.fromisoformat(value)
            if started.tzinfo is None:
                started = started.replace(tzinfo=timezone.utc)
            return round(max(0.0, time.time() - started.timestamp()), 1)
        except (TypeError, ValueError, OverflowError):
            return 0.0

    def _build_mapping_metrics(
        self,
        base: Path,
        progress: dict,
        manifest: dict,
        package_path: Path | None = None,
        started_at: str | None = None,
        diagnostic_paths: list[str] | None = None,
    ) -> dict:
        diagnostic_paths = diagnostic_paths or [
            "recording_manifest.yaml",
            "mapping_trace.json",
            "trajectory_raw.csv",
            "trajectory_optimized.csv",
            "trajectory_covariance.json",
            "optimization_summary.json",
            "localization_validation.json",
            "divergence_event.json",
            "loop_closures.csv",
            "scan_context/index.json",
            "scan_context/loop_candidates.csv",
            "save_progress.json",
        ]
        diagnostic_size = 0
        diagnostic_count = 0
        if package_path and package_path.is_file():
            try:
                with zipfile.ZipFile(package_path) as archive:
                    diagnostic_names = set(diagnostic_paths)
                    for info in archive.infolist():
                        if (
                            info.filename in diagnostic_names
                            or info.filename.startswith("diagnostics/rosbag/")
                            or info.filename.startswith("imu_preintegration/")
                        ):
                            diagnostic_size += info.file_size
                            diagnostic_count += 1
            except (OSError, zipfile.BadZipFile):
                LOGGER.warning("failed to calculate diagnostic package size for %s", package_path, exc_info=True)
        else:
            for relative in diagnostic_paths:
                path = base / relative
                if path.is_file():
                    diagnostic_size += path.stat().st_size
                    diagnostic_count += 1
        return {
            "schema": "roamerx.mapping-metrics.v1",
            "package_size_bytes": package_path.stat().st_size if package_path and package_path.is_file() else 0,
            "robot_directory_size_bytes": self._directory_size(base),
            "keyframe_count": self._count_keyframes(base, manifest, progress),
            "trajectory_m": self._trajectory_meters(base, progress),
            "mapping_duration_seconds": self._mapping_duration_seconds(started_at, progress),
            "diagnostic_data_size_bytes": diagnostic_size,
            "diagnostic_file_count": diagnostic_count,
        }

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
            "map.pcd",
            "map_raw.pcd",
            "map_raw.pgm",
            "map_raw.yaml",
            "map_raw.txt",
            "map_optimized_rejected.pcd",
            "map_optimized_rejected.pgm",
            "map_optimized_rejected.yaml",
            "map_optimized_rejected.txt",
            "map.txt",
            "gnss_origin.yaml",
            "mapping_trace.json",
            "map_manifest.json",
            "recording_manifest.yaml",
            "trajectory_raw.csv",
            "trajectory_optimized.csv",
            "trajectory_covariance.json",
            "optimization_summary.json",
            "localization_validation.json",
            "divergence_event.json",
            "rescue_metadata.json",
            "save_progress.json",
            "loop_closures.csv",
            "keyframes/keyframes.csv",
            "scan_context/index.json",
            "scan_context/loop_candidates.csv",
        ]
        if preview_path:
            upload_files.append(preview_path.name)
        files = []
        bag_dir = Path(self._rosbag_dir).expanduser() if self._rosbag_dir else None
        with zipfile.ZipFile(package_path, "w", zipfile.ZIP_DEFLATED) as archive:
            for name in upload_files:
                path = filtered_base / name
                if not path.exists() and filtered_base != base:
                    # Visibility-filtered outputs contain only navigation
                    # assets. Keep the complete session metadata from the raw
                    # session in the cloud package.
                    path = base / name
                if path.exists():
                    archive.write(path, arcname=name)
                    files.append(name)
            # Raw interval IMU samples and their bias linearization points are
            # required to audit or replay standard preintegration in the cloud.
            preintegration_root = base / "imu_preintegration"
            if preintegration_root.is_dir():
                for path in sorted(preintegration_root.glob("preint_*.json")):
                    name = path.relative_to(base).as_posix()
                    archive.write(path, arcname=name)
                    files.append(name)
        if bag_dir and bag_dir.is_dir():
            LOGGER.info(
                "Keeping mapping rosbag on robot at %s; not embedding it in the upload package",
                bag_dir,
            )
        map_manifest = self._read_json(filtered_base / "map_manifest.json")
        if not map_manifest and filtered_base != base:
            map_manifest = self._read_json(base / "map_manifest.json")
        optimization_summary = self._read_json(filtered_base / "optimization_summary.json")
        if not optimization_summary and filtered_base != base:
            optimization_summary = self._read_json(base / "optimization_summary.json")
        optimization = summary_without_corrections(optimization_summary) if optimization_summary else {}
        local_activation_allowed = bool(
            not is_rescue and optimization.get("auto_activation_allowed", True)
        )
        manual_revision = bool(command.get("_manual_optimization"))
        activate_manual_revision = bool(command.get("activate_after_upload", False))
        if local_activation_allowed and (not manual_revision or activate_manual_revision):
            self._refresh_current_map_links(
                filtered_base, list(self.REQUIRED_FILES + self.OPTIONAL_FILES)
            )
        else:
            LOGGER.warning(
                "Map %s saved but not selected as the local navigation map: stage=%s reasons=%s",
                filtered_base,
                optimization.get("stage"),
                (optimization.get("inertial_smoothing_guard") or {}).get("reasons"),
            )
        mapping_metrics = self._build_mapping_metrics(
            base,
            progress,
            map_manifest,
            package_path=package_path,
            started_at=self.session.started_at if self.session else None,
        )
        metadata = {
            "robot_code": self.media_client.robot_id,
            "mapping_session_id": self.session.session_id if self.session else str(uuid.uuid4()),
            # 地图名称使用 session 目录的时间戳，与 .jszr/map/<timestamp> 一致
            "map_name": command.get("map_name") or (work_dir.name if work_dir and work_dir != self.map_dir else (self.session.map_name if self.session else "untitled")),
            "map_version": version,
            "source_map_dir": str(filtered_base),
            "raw_map_dir": str(base),
            "auto_activate": bool(
                local_activation_allowed
                and (
                    activate_manual_revision
                    if manual_revision
                    else self.config.auto_activate_uploaded_map
                )
            ),
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
            "mapping_metrics": mapping_metrics,
            "optimization": optimization,
            "parent_map_id": str(command.get("parent_map_id") or ""),
            "local_rosbag_dir": str(bag_dir) if bag_dir and bag_dir.is_dir() else "",
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
        optimization_payload = self._read_json(source / "optimization_summary.json")
        correction_by_index = {
            int(item.get("index")): item
            for item in optimization_payload.get("corrections", [])
            if isinstance(item, dict) and item.get("index") is not None
        }
        alignment_locked = bool(int(gnss_metadata.get("alignment_locked") or 0))
        origin_lat = float(gnss_metadata.get("origin_latitude") or 0.0)
        origin_lon = float(gnss_metadata.get("origin_longitude") or 0.0)
        alignment_yaw = float(gnss_metadata.get("enu_to_map_yaw") or 0.0)
        heading_offset = math.radians(float(gnss_metadata.get("heading_offset_deg") or 0.0))
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
                            rtk["base_heading_deg"] = round(
                                (rtk["heading_deg"] - math.degrees(heading_offset)) % 360.0,
                                3,
                            )
                            map_yaw = yaw_enu + alignment_yaw + heading_offset
                            rtk["yaw"] = round(math.atan2(math.sin(map_yaw), math.cos(map_yaw)), 5)
                    correction = correction_by_index.get(int(row["index"]))
                    if correction:
                        raw_pose = correction.get("raw") or {}
                        optimized_pose = correction.get("optimized") or {}
                        if optimization_payload.get("applied") and optimized_pose:
                            slam = dict(optimized_pose)
                    sample = {
                        "index": int(row["index"]),
                        "stamp": round(float(row["stamp"]), 6),
                        "slam": slam,
                        "rtk": rtk,
                    }
                    if correction:
                        sample.update({
                            "raw": raw_pose,
                            "optimized": optimized_pose,
                            "correction": correction.get("delta") or {},
                            "correction_significant": bool(correction.get("significant")),
                        })
                    samples.append(sample)
                except (KeyError, TypeError, ValueError):
                    LOGGER.warning("Skipping malformed mapping trace row: %s", row)
        if not samples:
            return None
        output.mkdir(parents=True, exist_ok=True)
        trace_path = output / "mapping_trace.json"
        trace_path.write_text(json.dumps({
            "format": "roamerx.mapping-trace.v2",
            "frame_id": "map",
            "alignment_locked": alignment_locked,
            "heading_offset_deg": round(math.degrees(heading_offset), 3),
            "optimization": summary_without_corrections(optimization_payload) if optimization_payload else {},
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
        selected = tuple(
            name for name in self.OPTIONAL_FILES
            if name in files
        )
        try:
            activate_map_version(
                self.map_dir,
                base,
                required_files=self.REQUIRED_FILES,
                optional_files=selected,
            )
        except MapVersionPointerError as exc:
            raise ProtocolError("MAP_ACTIVATION_POINTER_FAILED", str(exc)) from exc
        LOGGER.info("Current map links refreshed to %s with files=%s", base, files)

    def _set_state(self, state: str) -> None:
        if not self.session:
            return
        with self._state_lock:
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
            # -ww is load-bearing, not cosmetic.  Without it ps truncates args to
            # $COLUMNS, and navigation's LIO node is the *same executable* as the
            # mapping node - only "-r __node:=lio_odometry" tells them apart, and
            # that sits ~70 chars in.  At COLUMNS=80 the line truncates to
            # ".../lib/robot_slam/mapping --ros-arg": the marker is gone, so
            # _is_lio_odometry_process() says False, "lib/robot_slam/mapping"
            # matches, and _stop_orphan_slam_processes() SIGKILLs the running
            # navigation odometry.  Reproduced from a terminal-launched
            # mapping_cli.py; the systemd service escapes only because it happens
            # to leave COLUMNS unset.
            result = subprocess.run(
                ["ps", "-ww", "-eo", "pid=,args="],
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
            if self._is_slam_mapping_process(args):
                pids.append(pid)
        return pids

    @classmethod
    def _is_slam_mapping_process(cls, args: str) -> bool:
        """Match the process itself, never a shell merely mentioning it."""
        if cls._is_lio_odometry_process(args):
            return False
        return bool(
            cls.SLAM_MAPPING_EXECUTABLE_RE.search(args)
            or cls.SLAM_LAUNCH_PROCESS_RE.search(args)
        )

    @staticmethod
    def _is_lio_odometry_process(args: str) -> bool:
        return any(marker in args for marker in MappingAdapter.LIO_ODOMETRY_PROCESS_MARKERS)

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
