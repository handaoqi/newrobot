from __future__ import annotations

import logging
import threading
import time
from datetime import datetime, timezone
from typing import Callable

from .local_store import LocalStore
from .protocol import MessageEnvelope, ProtocolError, build_ack, build_progress, build_result, decode_message, now_iso
from .safety_policy import SafetyPolicy
from .task_executor import TaskExecutor
from .teleop_skill_executor import TeleopSkillExecutor

LOGGER = logging.getLogger(__name__)


STARTUP_STAGE_LABELS = {
    "navigation_prepare": "准备地图、安全与碰撞监控",
    "fast_lio_readiness": "等待 FAST-LIO + IMU 局部收敛",
    "map_transfer": "应用任务地图",
    "localization_bootstrap": "准备定位节点",
    "last_trusted": "验证可信位姿",
    "mapping_origin_bounded": "搜索建图原点及周边候选",
    "mapping_origin": "搜索建图原点及周边候选",
    "route_waypoints": "搜索路线航点候选",
    "route_waypoint": "搜索路线航点候选",
    "keyframe_global_match": "执行全局关键帧匹配",
    "quick_initialization": "执行快速定位搜索",
    "operator_initial_pose": "验证手选初始位姿",
    "best_candidate_commit": "提交最优定位结果",
    "fast_lio_imu_handoff": "确认 FAST-LIO + IMU 接管",
    "final_localization_gate": "验收连续定位与锚点证据",
    "navigation_execution_activate": "激活导航执行组",
    "secondary_correction": "执行二次定位校正",
    "rtk_fixed": "验证 RTK 固定解",
    "trusted_rtk_fixed": "使用 RTK 固定解缩小 NDT 搜索范围",
    "rtk_correction": "执行 RTK 定位校正",
    "ukf_correction": "执行 UKF 融合校正",
    "ndt_secondary_correction": "执行 NDT 二次校正",
    "navigation_start": "应用航段策略并下发首航点",
}


def _startup_progress_detail(result: dict) -> dict:
    """Build a stable operator summary for task.start progress records."""
    localization = result.get("localization_attempts")
    localization = localization if isinstance(localization, dict) else {}
    phase = str(result.get("selected_stage") or localization.get("selected_stage") or "localization_bootstrap")
    phase_label = STARTUP_STAGE_LABELS.get(phase, phase.replace("_", " "))
    navigation_start = result.get("navigation_start")
    navigation_start = navigation_start if isinstance(navigation_start, dict) else {}
    navigation_status = str(navigation_start.get("status") or result.get("state") or "running").lower()
    at_navigation_start = phase == "navigation_start"
    navigation_done = at_navigation_start and navigation_status in {"accepted", "succeeded", "ready"}
    navigation_failed = at_navigation_start and navigation_status in {"failed", "rejected", "error"}
    localization_status = "completed" if at_navigation_start else "in_progress"
    dispatch_status = (
        "failed" if navigation_failed else "completed" if navigation_done else
        "in_progress" if at_navigation_start else "waiting"
    )
    if navigation_done:
        current_action = "首航点已下发，Nav2 开始执行"
        next_action = "持续上报路径跟踪与航点进度"
    elif navigation_failed:
        current_action = "启动导航失败"
        next_action = "等待安全处理或重新启动"
    elif at_navigation_start:
        current_action = phase_label
        next_action = "确认 Nav2 接受首航点"
    else:
        current_action = f"智能初始化定位：{phase_label}"
        next_action = "定位接管后应用航段策略并下发首航点"
    return {
        "phase": phase,
        "phase_label": phase_label,
        "status": "failed" if navigation_failed else "completed" if navigation_done else "running",
        "current_action": current_action,
        "next_action": next_action,
        "actions": [
            {"key": "navigation_stack_check", "label": "检查导航栈与安全状态", "status": "completed"},
            {"key": "route_validation", "label": "校验地图、边界与路线", "status": "completed"},
            {"key": "localization_initialization", "label": "初始化定位并确认主定位源", "status": localization_status},
            {"key": "navigation_dispatch", "label": "应用航段策略并下发首航点", "status": dispatch_status},
        ],
    }


class CommandProcessor:
    def __init__(
        self,
        *,
        robot_id: str,
        store: LocalStore,
        safety: SafetyPolicy,
        task_executor: TaskExecutor,
        publish_ack: Callable[[str, dict], None],
        publish_result: Callable[[str, dict], None],
        publish_progress: Callable[[str, dict], None] | None = None,
        mapping_adapter=None,
        map_activation_adapter=None,
        navigation_stack_adapter=None,
        localization_adapter=None,
        teleop_control_adapter=None,
        person_follow_controller=None,
        sensor_control_adapter=None,
        charge_control_adapter=None,
        audio_control_adapter=None,
        structured_logs=None,
        navigation_boundary=None,
        temporary_fusion_release_callback: Callable[[str], bool] | None = None,
    ) -> None:
        self.robot_id = robot_id
        self.store = store
        self.safety = safety
        self.task_executor = task_executor
        self.mapping_adapter = mapping_adapter
        self.map_activation_adapter = map_activation_adapter
        self.navigation_stack_adapter = navigation_stack_adapter
        self.localization_adapter = localization_adapter
        self.teleop_control_adapter = teleop_control_adapter
        self.person_follow_controller = person_follow_controller
        self.sensor_control_adapter = sensor_control_adapter
        self.charge_control_adapter = charge_control_adapter
        self.audio_control_adapter = audio_control_adapter
        self.structured_logs = structured_logs
        self.navigation_boundary = navigation_boundary
        self.temporary_fusion_release_callback = temporary_fusion_release_callback
        self.publish_ack = publish_ack
        self.publish_result = publish_result
        self.publish_progress = publish_progress or (lambda *_args, **_kwargs: None)
        self._navigation_command_lock = threading.Lock()
        # Pose commands bypass the navigation-stack lock so they can seed a
        # nav.start that is waiting for localization. They still need their
        # own lock: two operator requests must never supersede one another.
        self._localization_command_lock = threading.Lock()
        self.skill_executor = TeleopSkillExecutor(localization_adapter) if localization_adapter else None

    def _release_temporary_fusion_for_manual_control(self, action: str) -> None:
        callback = self.temporary_fusion_release_callback
        if not callable(callback):
            return
        try:
            callback(f"manual_control_{action}")
        except Exception:
            # Manual takeover must remain available even if localization
            # profile cleanup has to fall back to its in-node TTL.
            LOGGER.exception("failed to release temporary fusion profile for %s", action)

    def handle_command(self, raw) -> tuple[dict, dict | None]:
        envelope = decode_message(raw)
        LOGGER.info("handle_command: type=%s id=%s", envelope.message_type, envelope.payload.get("command_id", "?"))
        if envelope.robot_id != self.robot_id:
            raise ProtocolError("ROBOT_ID_MISMATCH", "command robot_id does not match this device")
        command_id = envelope.payload["command_id"]
        duplicate = self.store.get_processed_command(command_id)
        if duplicate:
            if duplicate["ack"]:
                self.publish_ack(command_id, duplicate["ack"])
            if duplicate["result"]:
                self.publish_result(command_id, duplicate["result"])
            return duplicate["ack"] or {}, duplicate["result"]

        started_at = now_iso()
        ack = None
        try:
            self._structured(
                "INFO", self._module_for_command(envelope.message_type),
                f"{envelope.message_type}.received", f"收到命令 {envelope.message_type}",
                envelope=envelope,
            )
            if datetime.now(timezone.utc) >= datetime.fromisoformat(
                envelope.payload["expires_at"].replace("Z", "+00:00")
            ):
                raise ProtocolError("COMMAND_EXPIRED", "command has expired")
            self._validate_expected_state(envelope)
            prepared_task_start = envelope.message_type == "task.start"
            if prepared_task_start:
                self._ensure_navigation_stack_for_task()
                docking = ((envelope.payload.get("command") or {}).get("docking") or {})
                if docking.get("enabled"):
                    # A return-to-charge route may intentionally use a
                    # dedicated map. Activate it before the final map and
                    # localization checks; validating the old patrol map first
                    # rejects every legitimate cross-map docking task.
                    self._prepare_docking_map(envelope.payload["command"])
                if self.navigation_boundary:
                    self.navigation_boundary.validate_route(envelope.payload["command"]["route_snapshot"])
                route_snapshot = envelope.payload["command"]["route_snapshot"]
                route_waypoints = route_snapshot.get("waypoints") or []
                self._structured(
                    "DEBUG", "planner", "planner.route_input", "巡检路线规划输入",
                    data={
                        "route_id": route_snapshot.get("route_id"),
                        "waypoint_count": len(route_waypoints),
                        "global_controller": route_snapshot.get("global_controller"),
                        "boundary_revision": route_snapshot.get("boundary_revision"),
                        "first_waypoint": route_waypoints[0] if route_waypoints else None,
                        "last_waypoint": route_waypoints[-1] if route_waypoints else None,
                    },
                    envelope=envelope,
                )
                # A deliberate navigation start is an explicit request to hand
                # motion authority back to autonomy.  Validate every hard
                # interlock first; only then release an existing operator
                # session, so a rejected task cannot unexpectedly cancel it.
                self.safety.validate_task_start(
                    envelope,
                    self.task_executor.has_active_task(),
                    allow_manual_takeover_release=True,
                    require_localization=not bool(
                        (envelope.payload.get("command") or {}).get("smart_initialize", True)
                    ),
                    require_navigation_ready=not bool(
                        (envelope.payload.get("command") or {}).get("smart_initialize", True)
                    ),
                )
                self._release_manual_control_for_task()
                # The acknowledgement must carry the same task version sequence
                # as task.started/task.progress.  Previously it reused a
                # completed task's stale version, causing progress events to be
                # discarded by the center as out-of-order.
                self.task_executor.prepare_task_start(envelope)
            ack = build_ack(
                envelope,
                accepted=True,
                edge_state_version=self._state_version(),
            )
            self.store.save_command_ack(command_id, ack)
            self.publish_ack(command_id, ack)
            result = None if prepared_task_start else self._execute(envelope, started_at)
            if prepared_task_start:
                # Initialization can actively relocalize and may take up to 90s.
                # Persist and publish the acceptance first so an Edge restart in
                # that window cannot leave the center stuck in `dispatching`.
                prepared_at = now_iso()
                self._emit_command_progress(
                    envelope,
                    started_at,
                    {
                        "state": "running",
                        "selected_stage": "navigation_prepare",
                        "stages": [{
                            "stage": "navigation_prepare",
                            "status": "accepted",
                            "started_at": started_at,
                            "finished_at": prepared_at,
                        }],
                        "navigation_lifecycle": {"status": "prepared"},
                    },
                )
                set_progress = getattr(
                    self.localization_adapter, "set_attempt_progress_callback", None
                )
                initialization_result = {}
                try:
                    if callable(set_progress):
                        set_progress(
                            lambda payload: self._emit_command_progress(
                                envelope, started_at, payload
                            )
                        )
                    self._emit_command_progress(
                        envelope, started_at,
                        {
                            "state": "running",
                            "selected_stage": "fast_lio_readiness",
                            "navigation_lifecycle": {"status": "prepared"},
                        },
                    )
                    if bool((envelope.payload.get("command") or {}).get("smart_initialize", True)):
                        initialization_result = dict(
                            self.task_executor.initialize_before_navigation() or {}
                        )
                    self.safety.wait_until_localization_stable()
                    self._emit_command_progress(
                        envelope, started_at,
                        {**initialization_result, "state": "running",
                         "selected_stage": "navigation_execution_activate",
                         "navigation_lifecycle": {"status": "activating"}},
                    )
                    activation_evidence = self._activate_navigation_execution_for_task()
                    self._emit_command_progress(
                        envelope, started_at,
                        {
                            **initialization_result,
                            "state": "running",
                            "selected_stage": "navigation_execution_activate",
                            "navigation_lifecycle": {
                                "status": "active",
                                **dict(activation_evidence or {}),
                            },
                        },
                    )
                    validate_admission = getattr(self.safety, "validate_navigation_admission", None)
                    if callable(validate_admission):
                        validate_admission()
                finally:
                    if callable(set_progress):
                        set_progress(None)
                navigation_started_at = now_iso()
                self._emit_command_progress(
                    envelope,
                    started_at,
                    {
                        **initialization_result,
                        "state": "running",
                        "selected_stage": "navigation_start",
                        "navigation_start": {
                            "status": "starting",
                            "started_at": navigation_started_at,
                            "updated_at": navigation_started_at,
                        },
                    },
                )
                try:
                    self.task_executor.launch_prepared_task()
                except Exception as exc:
                    navigation_failed_at = now_iso()
                    self._emit_command_progress(
                        envelope,
                        started_at,
                        {
                            "state": "failed",
                            "selected_stage": "navigation_start",
                            "navigation_start": {
                                "status": "failed",
                                "started_at": navigation_started_at,
                                "updated_at": navigation_failed_at,
                                "finished_at": navigation_failed_at,
                                "error_code": str(getattr(exc, "code", "") or "NAVIGATION_START_FAILED"),
                                "error_message": str(getattr(exc, "message", "") or exc),
                            },
                        },
                    )
                    raise
                navigation_finished_at = now_iso()
                self._emit_command_progress(
                    envelope,
                    started_at,
                    {
                        "state": "accepted",
                        "selected_stage": "navigation_start",
                        "navigation_start": {
                            "status": "accepted",
                            "started_at": navigation_started_at,
                            "updated_at": navigation_finished_at,
                            "finished_at": navigation_finished_at,
                        },
                    },
                )
            if result:
                self.store.save_command_result(command_id, result)
                self.publish_result(command_id, result)
            return ack, result
        except ProtocolError as exc:
            self._structured(
                "WARNING" if ack is None else "ERROR",
                self._module_for_command(envelope.message_type),
                f"{envelope.message_type}.failed", f"命令 {envelope.message_type} 失败：{exc.message}",
                data={"error_code": exc.code}, envelope=envelope,
            )
            if ack is None:
                ack = build_ack(
                    envelope,
                    accepted=False,
                    edge_state_version=self._state_version(),
                    code=exc.code,
                    message=exc.message,
                )
                self.store.save_command_ack(command_id, ack)
                self.publish_ack(command_id, ack)
                return ack, None
            # Once accepted, failures must be reported as a command result.
            # A task.start is acknowledged before launch so the platform sees a
            # strictly ordered lifecycle.  If launch then fails (for example,
            # Nav2's costmap nodes are still appearing), do not retain the
            # accepted context: it would reject every later task as ROBOT_BUSY.
            if prepared_task_start:
                try:
                    self.task_executor.force_exit(
                        envelope.payload["task_execution_id"],
                        report_start_result=False,
                    )
                except Exception:
                    LOGGER.exception("failed to clear task after launch error")
            result = build_result(
                envelope,
                status="failed",
                result=exc.details if isinstance(exc.details, dict) else {},
                started_at=started_at,
                error_code=exc.code,
                error_message=exc.message,
            )
            self.store.save_command_result(command_id, result)
            self.publish_result(command_id, result)
            return ack, result
        except Exception as exc:
            LOGGER.exception("unexpected error executing command %s", command_id)
            self._structured(
                "ERROR", self._module_for_command(envelope.message_type),
                f"{envelope.message_type}.internal_error", f"命令 {envelope.message_type} 发生内部错误",
                data={"error": str(exc)}, envelope=envelope,
            )
            if prepared_task_start and ack is not None:
                try:
                    self.task_executor.force_exit(
                        envelope.payload["task_execution_id"],
                        report_start_result=False,
                    )
                except Exception:
                    LOGGER.exception("failed to clear task after unexpected launch error")
            result = build_result(
                envelope,
                status="failed",
                result={},
                started_at=started_at,
                error_code="INTERNAL_ERROR",
                error_message=str(exc),
            )
            self.store.save_command_result(command_id, result)
            self.publish_result(command_id, result)
            return ack if "ack" in dir() else {}, result

    def _ensure_navigation_stack_for_task(self) -> None:
        """Repair an unexpectedly stopped Nav2 stack before task validation.

        Nav2 and localization stay warm between tasks. Starting them for every
        route adds latency and resets useful state, but a task should still be
        able to recover when the resident stack was stopped or crashed.
        """
        if not self.navigation_stack_adapter:
            return
        prepare = getattr(self.navigation_stack_adapter, "prepare", None)
        if callable(prepare):
            prepare({"reason": "task_start"})
        elif not self.safety.state.nav_ready:
            self.navigation_stack_adapter.start({"reason": "task_start"})
        wait_until_prepared = getattr(self.task_executor.navigation, "wait_until_prepared", None)
        if callable(wait_until_prepared) and not wait_until_prepared(timeout_seconds=45.0):
            raise ProtocolError("NAV_STACK_PREPARE_FAILED", "navigation map/safety group did not become prepared")
        self.safety.state.nav_ready = False

    def _activate_navigation_execution_for_task(self) -> dict:
        if not self.navigation_stack_adapter:
            return {}
        started_at = time.monotonic()
        activate = getattr(self.navigation_stack_adapter, "activate_execution", None)
        activation = {}
        if callable(activate):
            activation = activate() or {}
        activation_finished_at = time.monotonic()
        self._await_navigation_stack_ready(
            timeout_seconds=45.0,
            message="navigation execution group did not become ready after localization",
        )
        return {
            "activation": activation,
            "activation_seconds": round(activation_finished_at - started_at, 3),
            "ready_check_seconds": round(time.monotonic() - activation_finished_at, 3),
            "total_seconds": round(time.monotonic() - started_at, 3),
        }

    def _await_navigation_stack_ready(self, timeout_seconds: float, message: str) -> None:
        wait_until_ready = getattr(self.task_executor.navigation, "wait_until_ready", None)
        if not callable(wait_until_ready) or not wait_until_ready(timeout_seconds=timeout_seconds):
            raise ProtocolError("NAV_STACK_NOT_READY", message)
        self.safety.state.nav_ready = True

    def _wait_for_command_fast_lio_readiness(self, timeout_seconds: float = 90.0) -> dict:
        """Read the same local FAST-LIO gate used by task startup.

        Direct ``nav.start/restart/recover`` commands used to bypass task
        initialization entirely.  Keep optional legacy adapters compatible,
        but require production ROS adapters to provide three fresh, stopped
        local frames before any NDT seed is evaluated.
        """
        readiness = getattr(self.localization_adapter, "lio_readiness", None)
        if not callable(readiness):
            return {"status": "unavailable", "reason": "adapter_compatibility"}
        stopped = getattr(self.localization_adapter, "is_robot_stopped", None)
        deadline = time.monotonic() + max(0.0, float(timeout_seconds))
        last = {}
        while time.monotonic() < deadline:
            last = readiness(max_age_seconds=0.50, required_frames=3) or {}
            if bool(last.get("ready")) and (not callable(stopped) or bool(stopped())):
                return {**last, "status": "ready"}
            time.sleep(0.2)
        raise ProtocolError(
            "FAST_LIO_NOT_READY",
            "FAST-LIO + IMU did not provide three fresh local odometry frames while stopped",
            details={"fast_lio": last},
        )

    def _wait_for_command_final_localization_gate(self, timeout_seconds: float = 15.0) -> dict:
        waiter = getattr(self.localization_adapter, "wait_for_final_localization_gate", None)
        if not callable(waiter):
            return {"status": "unavailable", "reason": "adapter_compatibility"}
        result = waiter(timeout_seconds=timeout_seconds, required_samples=3) or {}
        if bool(result.get("accepted")):
            return {**result, "status": "accepted"}
        raise ProtocolError(
            "FINAL_LOCALIZATION_GATE_FAILED",
            "NDT correction completed but FAST-LIO final acceptance evidence is incomplete",
            details={"final_localization_gate": result},
        )

    def _command_mapping_origin(self) -> dict | None:
        if self.map_activation_adapter is None:
            return None
        try:
            return self.map_activation_adapter.mapping_start_pose()
        except ProtocolError as exc:
            return {
                "unavailable_error_code": exc.code,
                "unavailable_error_message": exc.message,
            }

    def _effective_command_localization_scope(self, command: dict) -> tuple[str, str]:
        """Resolve scene/coordinates from the active map for all relocalize entries."""
        scene = str(command.get("scene_scope") or "").strip().lower()
        coordinate = str(command.get("coordinate_mode") or "").strip().lower()
        adapter = self.map_activation_adapter
        reader = getattr(adapter, "status", None) if adapter is not None else None
        if not callable(reader):
            return scene, coordinate
        try:
            active = reader() or {}
        except Exception:
            return scene, coordinate
        if not isinstance(active, dict):
            return scene, coordinate
        command_map_id = str(command.get("map_id") or "").strip()
        active_map_id = str(active.get("map_id") or "").strip()
        if command_map_id and active_map_id and command_map_id != active_map_id:
            return scene, coordinate
        constraints = active.get("map_constraints") if isinstance(active.get("map_constraints"), dict) else {}
        active_coordinate = str(
            active.get("coordinate_mode") or constraints.get("coordinate_mode") or ""
        ).strip().lower()
        active_scene = str(
            active.get("scene_scope") or constraints.get("scene_scope") or ""
        ).strip().lower()
        if active_coordinate in {"rtk_fixed", "local_only"}:
            coordinate = active_coordinate
        if active_scene in {"indoor", "transition", "outdoor"}:
            scene = active_scene
        return scene, coordinate

    def _command_scene_uses_rtk_seed(self, command: dict) -> bool:
        scene, coordinate = self._effective_command_localization_scope(command)
        return scene in {"outdoor", "transition"} and coordinate == "rtk_fixed"

    def _trusted_rtk_seed_for_command(self, command: dict) -> tuple[dict | None, dict | None]:
        """Bounded fixed-RTK evidence for an outdoor NDT candidate list."""
        scene, coordinate = self._effective_command_localization_scope(command)
        if not (scene in {"outdoor", "transition"} and coordinate == "rtk_fixed"):
            return None, None
        seed_reader = getattr(self.localization_adapter, "trusted_rtk_search_seed", None)
        if not callable(seed_reader):
            return None, None
        evidence = seed_reader(timeout_seconds=0.8) or {}
        if bool(evidence.get("accepted")) and isinstance(evidence.get("seed_pose"), dict):
            return dict(evidence["seed_pose"]), evidence
        return None, evidence

    def _run_navigation_command_localization(self, envelope: MessageEnvelope, command: dict) -> dict:
        """NDT-first initialization for direct navigation lifecycle commands.

        There is intentionally no RTK-direct branch here: a verified fixed
        RTK may be supplied as a candidate to NDT, then the normal NDT commit,
        LIO handoff, optional secondary correction and final gate still apply.
        """
        if self.localization_adapter is None:
            raise ProtocolError("LOCALIZATION_UNAVAILABLE", "localization adapter is not configured")
        lio_readiness = self._wait_for_command_fast_lio_readiness()
        self._emit_command_progress(
            envelope,
            now_iso(),
            {
                "state": "running",
                "selected_stage": "fast_lio_readiness",
                "fast_lio_readiness": lio_readiness,
            },
        )
        origin = self._command_mapping_origin()
        waypoints = list(command.get("waypoints") or [])
        route = command.get("route_snapshot") if isinstance(command.get("route_snapshot"), dict) else {}
        if not waypoints:
            waypoints = list(route.get("waypoints") or [])
        effective_scene, effective_coordinate = self._effective_command_localization_scope(command)
        if effective_scene:
            command = {**command, "scene_scope": effective_scene}
        if effective_coordinate:
            command = {**command, "coordinate_mode": effective_coordinate}
        trusted_seed, trusted_evidence = self._trusted_rtk_seed_for_command(command)
        trusted_stage = trusted_evidence or {
            "status": "skipped",
            "accepted": False,
            "reason": (
                "indoor_or_local_only"
                if not self._command_scene_uses_rtk_seed(command)
                else "fixed_rtk_not_accepted"
            ),
        }
        self._emit_command_progress(
            envelope,
            now_iso(),
            {
                "state": "running",
                "selected_stage": "trusted_rtk_fixed",
                "trusted_rtk_seed": trusted_stage,
                "scene_scope": command.get("scene_scope"),
            },
        )
        progressive = getattr(self.localization_adapter, "progressive_relocalize", None)
        if not callable(progressive):
            raise ProtocolError("PROGRESSIVE_RELOCALIZATION_UNAVAILABLE", "NDT progressive relocalization is unavailable")
        initial_ndt_commit = progressive(
            origin=origin,
            waypoints=waypoints,
            trusted_seed=trusted_seed,
            wait_seconds=float(command.get("wait_seconds", 180.0)),
        ) or {}
        raw_mode = str(command.get("localization_mode") or "").strip().lower()
        if not raw_mode and command.get("map_activation_requires_waypoint_mode"):
            secondary = {
                "status": "skipped",
                "reason": "first_waypoint_localization_mode_unavailable",
                "message": "地图已完成 NDT/LIO 初始化，但未提供首航点定位校正配置",
                "mode": None,
            }
            self._emit_command_progress(
                envelope,
                now_iso(),
                {
                    "state": "running",
                    "selected_stage": "secondary_correction",
                    "secondary_correction": secondary,
                    "trusted_rtk_seed": trusted_evidence,
                },
            )
            final_gate = self._wait_for_command_final_localization_gate()
            return {
                "initial_ndt_commit": initial_ndt_commit,
                "secondary_correction": secondary,
                "final_localization_gate": final_gate,
                "continuous_source": "lio_imu",
                "trusted_rtk_seed": trusted_evidence,
                "selected_stage": "final_localization_gate",
            }
        mode = raw_mode or "ndt"
        # rtk_ndt describes a map capability, not a waypoint correction mode.
        # A map activation without a first waypoint mode is explicitly
        # reported above instead of silently converting it to RTK.
        if mode == "rtk_ndt":
            mode = "ndt"
        if mode not in {"ndt", "rtk", "ukf"}:
            mode = "ndt"
        secondary = self._run_external_secondary_correction(
            envelope, mode, prefix="navigation_lifecycle"
        )
        self._emit_command_progress(
            envelope,
            now_iso(),
            {
                "state": "running",
                "selected_stage": "secondary_correction",
                "secondary_correction": secondary,
                "trusted_rtk_seed": trusted_evidence,
            },
        )
        final_gate = self._wait_for_command_final_localization_gate()
        self._emit_command_progress(
            envelope,
            now_iso(),
            {
                "state": "running",
                "selected_stage": "final_localization_gate",
                "secondary_correction": secondary,
                "final_localization_gate": final_gate,
                "trusted_rtk_seed": trusted_evidence,
            },
        )
        return {
            "initial_ndt_commit": initial_ndt_commit,
            "secondary_correction": secondary,
            "final_localization_gate": final_gate,
            "continuous_source": "lio_imu",
            "trusted_rtk_seed": trusted_evidence,
            "selected_stage": "final_localization_gate",
        }

    def _run_external_secondary_correction(
        self, envelope: MessageEnvelope, mode: str, *, prefix: str
    ) -> dict:
        """Run and verify the stationary post-NDT correction transaction."""
        correction = getattr(self.localization_adapter, "control_localization_correction", None)
        if not callable(correction):
            return {"status": "skipped", "reason": "interface_unavailable", "mode": mode}
        transaction_id = f"{prefix}:{envelope.message_id}:secondary:{mode}"
        secondary = correction(transaction_id, mode, "start") or {}
        secondary = {**secondary, "mode": mode, "transaction_id": transaction_id, "started_at": now_iso()}
        if not secondary.get("accepted"):
            if mode == "ndt" and str(secondary.get("status") or "") == "unavailable":
                return {**secondary, "status": "skipped", "finished_at": now_iso()}
            raise ProtocolError(
                "LOCALIZATION_SECONDARY_CORRECTION_REJECTED",
                str(secondary.get("message") or secondary.get("status") or "secondary correction rejected"),
                details={"secondary_correction": secondary},
            )
        decision_reader = getattr(self.localization_adapter, "localization_decision", None)
        if not callable(decision_reader):
            raise ProtocolError(
                "LOCALIZATION_SECONDARY_CORRECTION_UNVERIFIABLE",
                "secondary correction was accepted but no decision reader is available",
                details={"secondary_correction": secondary},
            )
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            decision = decision_reader() or {}
            one_shot = decision.get("one_shot_correction") if isinstance(decision, dict) else {}
            if isinstance(one_shot, dict) and str(one_shot.get("transaction_id") or "") == transaction_id:
                status = str(one_shot.get("status") or "")
                completed = {**secondary, **one_shot, "finished_at": now_iso()}
                if status == "completed":
                    return completed
                if status in {"failed", "cancelled", "rejected"}:
                    raise ProtocolError(
                        "LOCALIZATION_SECONDARY_CORRECTION_FAILED",
                        str(completed.get("reason") or f"secondary {mode} correction failed"),
                        details={"secondary_correction": completed},
                    )
            time.sleep(0.2)
        try:
            correction(transaction_id, mode, "cancel")
        except Exception:
            LOGGER.exception("failed to cancel timed-out secondary correction %s", transaction_id)
        timed_out = {**secondary, "status": "timed_out", "finished_at": now_iso()}
        raise ProtocolError(
            "LOCALIZATION_SECONDARY_CORRECTION_TIMEOUT",
            f"secondary {mode} correction timed out",
            details={"secondary_correction": timed_out},
        )

    def _wait_for_initial_pose_subscriber(self, timeout_seconds: float) -> bool:
        """Probe the concrete localization seed receiver when the adapter supports it."""
        wait_for_subscriber = getattr(
            self.localization_adapter,
            "wait_for_initial_pose_subscriber",
            None,
        )
        # Test adapters and older optional integrations do not expose the ROS
        # publisher. Their own pose call remains the authoritative check.
        if not callable(wait_for_subscriber):
            return True
        return bool(wait_for_subscriber(timeout_seconds=timeout_seconds))

    def _ensure_initial_pose_subscriber(self) -> dict | None:
        """Start localization without waiting for the pose it is about to receive.

        ``nav.start`` intentionally waits for localization status=3 before it
        starts Nav2. Therefore it cannot be used to prepare a cold
        ``nav.initial_pose`` command: status=3 itself depends on that seed.
        ``restart_localization`` stops after the node and map service are ready,
        which breaks that dependency cycle.
        """
        if self._wait_for_initial_pose_subscriber(1.0):
            return None
        if not self.navigation_stack_adapter:
            raise ProtocolError(
                "LOCALIZATION_UNAVAILABLE",
                "/initialpose has no localization subscriber and the localization stack cannot be started",
            )

        acquired = self._navigation_command_lock.acquire(blocking=False)
        if not acquired:
            # A concurrent nav.start/restart owns the stack lock. It may still
            # be bringing up localization, so let it create the subscriber
            # while this pose command remains free to seed it.
            if self._wait_for_initial_pose_subscriber(45.0):
                return None
            raise ProtocolError(
                "LOCALIZATION_UNAVAILABLE",
                "/initialpose subscriber did not appear while the navigation stack was starting",
            )
        try:
            if self._wait_for_initial_pose_subscriber(0.0):
                return None
            bootstrap = self.navigation_stack_adapter.restart_localization()
        finally:
            self._navigation_command_lock.release()

        if not self._wait_for_initial_pose_subscriber(30.0):
            raise ProtocolError(
                "LOCALIZATION_UNAVAILABLE",
                "localization was started but /initialpose still has no subscriber",
                details={"localization_bootstrap": bootstrap},
            )
        return bootstrap

    def _start_navigation_after_localization(self) -> dict:
        if not self.navigation_stack_adapter:
            raise ProtocolError("NAVIGATION_STACK_UNAVAILABLE", "navigation stack adapter is not configured")
        if not self._navigation_command_lock.acquire(blocking=True, timeout=45.0):
            raise ProtocolError("NAV_COMMAND_BUSY", "another navigation command is still running")
        try:
            prepare = getattr(self.navigation_stack_adapter, "prepare", None)
            if callable(prepare):
                result = prepare({"reason": "initial_pose_bootstrap"})
                self._activate_navigation_execution_for_task()
            else:
                result = self.navigation_stack_adapter.start({"reason": "initial_pose_bootstrap"})
                self._await_navigation_stack_ready(
                    timeout_seconds=45.0,
                    message="Nav2 did not become ready after localization initialization",
                )
            return {**result, "ready": True}
        finally:
            self._navigation_command_lock.release()

    def _pause_active_task_for_operator_localization(self) -> dict | None:
        """Pause only a running task; preserve a deliberate operator pause.

        The saved waypoint index belongs to TaskExecutor and is restored only
        after the new localization has passed the final LIO gate.  A failure
        leaves the task safely paused with Nav2 execution inactive.
        """
        context = getattr(self.task_executor, "context", None)
        if context is None or str(getattr(context, "state", "")) != "running":
            return None
        execution_id = str(getattr(context, "task_execution_id", "") or "")
        if not execution_id:
            return None
        paused = self.task_executor.pause_task(execution_id)
        deactivate = getattr(self.navigation_stack_adapter, "deactivate_execution", None)
        if callable(deactivate):
            deactivate()
        self.safety.state.nav_ready = False
        return {
            "execution_id": execution_id,
            "resume_index": int(paused.get("resume_from_waypoint_index", 0)),
            "paused": paused,
        }

    def _resume_task_after_operator_localization(self, transaction: dict | None) -> dict | None:
        if not transaction:
            return None
        self._activate_navigation_execution_for_task()
        return self.task_executor.resume_task(
            transaction["execution_id"], int(transaction["resume_index"])
        )

    def _prepare_docking_map(self, command: dict) -> None:
        if not self.map_activation_adapter or not self.navigation_stack_adapter:
            raise ProtocolError("DOCKING_UNAVAILABLE", "map activation or navigation stack is unavailable")
        map_payload = dict(command.get("map") or {})
        map_payload.setdefault("map_name", map_payload.get("map_version", ""))
        if not map_payload.get("map_id") or not map_payload.get("map_version"):
            raise ProtocolError("DOCKING_MAP_INVALID", "docking task has no map identity")
        if map_payload.get("local_map_dir"):
            map_payload["local_map_dir"] = map_payload["local_map_dir"]
        activated = self.map_activation_adapter.activate(map_payload)
        if self.navigation_boundary:
            self.navigation_boundary.activate_map(map_payload["map_id"], map_payload["map_version"])
        self.navigation_stack_adapter.switch_map()
        navigation = self.task_executor.navigation
        if not navigation.wait_until_ready(timeout_seconds=45):
            raise ProtocolError(
                "NAV_STACK_NOT_READY",
                "Nav2 did not become active after switching the docking map",
            )
        LOGGER.info("docking map activated: %s", activated.get("map_id"))

    def _validate_expected_state(self, envelope: MessageEnvelope) -> None:
        # Task controls express an operator's current intent.  They must remain
        # usable while the centre and the edge are briefly out of sync (for
        # example while a navigation result is still being reconciled).
        if envelope.message_type in {"task.pause", "task.resume", "task.force_exit", "task.recover.v1"}:
            return
        expected = envelope.payload.get("expected_robot_state_version")
        if expected is None:
            return
        current = self._state_version()
        if int(expected) < current:
            raise ProtocolError("STALE_ROBOT_STATE", f"expected={expected}, current={current}")

    def _execute(self, envelope: MessageEnvelope, started_at: str) -> dict | None:
        if envelope.message_type == "task.start":
            raise ProtocolError("INVALID_COMMAND_FLOW", "task.start must be prepared before acknowledgement")
        if envelope.message_type.startswith("mapping."):
            return self._execute_mapping(envelope, started_at)
        if envelope.message_type.startswith("map."):
            return self._execute_map(envelope, started_at)
        if envelope.message_type == "diagnostics.nav_rosbag_stop":
            command = envelope.payload.get("command") or {}
            result_payload = self.task_executor.stop_loop_rosbag(
                command.get("loop_session_id")
            )
            return build_result(
                envelope,
                status="succeeded",
                result=result_payload,
                started_at=started_at,
            )
        if envelope.message_type == "diagnostics.log_config":
            if not self.structured_logs:
                raise ProtocolError("STRUCTURED_LOGS_UNAVAILABLE", "structured logging is not configured")
            command = envelope.payload.get("command") or {}
            result_payload = self.structured_logs.configure_debug(
                enabled=command.get("enabled", False),
                modules=command.get("modules"),
                sample_hz=command.get("sample_hz", 1.0),
                expires_at=command.get("expires_at"),
            )
            self._structured(
                "INFO", "system", "debug.configuration_applied",
                "DEBUG 日志配置已应用", data=result_payload, envelope=envelope,
            )
            if self.structured_logs:
                self.structured_logs.flush()
            return build_result(envelope, status="succeeded", result=result_payload, started_at=started_at)
        if envelope.message_type.startswith("nav."):
            return self._execute_navigation(envelope, started_at)
        if envelope.message_type.startswith("teleop."):
            return self._execute_teleop(envelope, started_at)
        if envelope.message_type.startswith("sensor."):
            return self._execute_sensor(envelope, started_at)
        if envelope.message_type.startswith("charge."):
            return self._execute_charge(envelope, started_at)
        if envelope.message_type.startswith("motion."):
            return self._execute_motion_control(envelope, started_at)
        if envelope.message_type.startswith("audio."):
            return self._execute_audio(envelope, started_at)
        execution_id = envelope.payload["task_execution_id"]
        if envelope.message_type == "task.force_exit":
            result_payload = self.task_executor.force_exit(envelope.payload["task_execution_id"])
            return build_result(envelope, status="succeeded", result=result_payload, started_at=started_at)
        if not self.task_executor.context:
            raise ProtocolError("TASK_CONTEXT_MISMATCH", "no active task context")
        if envelope.message_type == "task.pause":
            self.safety.validate_pause(self.task_executor.context.state)
            result_payload = self.task_executor.pause_task(execution_id)
        elif envelope.message_type == "task.resume":
            self.safety.validate_resume(self.task_executor.context.state)
            resume_index = int(envelope.payload["command"].get("resume_from_waypoint_index", -1))
            result_payload = self.task_executor.resume_task(execution_id, resume_index)
        elif envelope.message_type == "task.resume_forward":
            result_payload = self.task_executor.resume_forward(execution_id)
        elif envelope.message_type == "task.recover.v1":
            command = envelope.payload.get("command") or {}
            state = self.safety.state
            if state.emergency_stop:
                raise ProtocolError("EMERGENCY_STOP_ACTIVE", "emergency stop is active")
            if state.control_mode == "manual_takeover":
                raise ProtocolError("MANUAL_TAKEOVER_ACTIVE", "manual takeover is active")
            if (
                state.power_available
                and state.battery_percent is not None
                and state.battery_percent < self.safety.config.low_battery_percent
            ):
                raise ProtocolError("LOW_BATTERY", f"battery={state.battery_percent}")
            self.safety.validate_resume(self.task_executor.context.state)
            if not state.nav_ready:
                if not self.navigation_stack_adapter:
                    raise ProtocolError(
                        "NAVIGATION_STACK_UNAVAILABLE",
                        "navigation stack adapter is not configured",
                    )
                self.navigation_stack_adapter.recover(
                    {"reason": "center_loop_recovery", "attempt": int(command.get("attempt") or 0)}
                )
                self._await_navigation_stack_ready(
                    timeout_seconds=45.0,
                    message="Nav2 did not become ready during task recovery",
                )
            if (
                str(command.get("trigger_reason_code") or "")
                == "ARRIVAL_POSE_CONVERGENCE_FAILED"
                and self.navigation_boundary
            ):
                pose = self.task_executor.navigation.latest_pose()
                if pose is None:
                    raise ProtocolError("LOCALIZATION_NOT_READY", "current pose is unavailable")
                route = self.task_executor.context.route_snapshot or {}
                self.navigation_boundary.validate_point(
                    str(state.current_map_id or ""),
                    float(pose.x),
                    float(pose.y),
                    expected_revision=route.get("boundary_revision"),
                )
            result_payload = self.task_executor.recover_task(
                execution_id,
                trigger_reason_code=str(command.get("trigger_reason_code") or ""),
                recovery_episode_id=str(command.get("recovery_episode_id") or ""),
                attempt=int(command.get("attempt") or 0),
            )
        else:
            self.safety.validate_cancel(self.task_executor.context.state)
            result_payload = self.task_executor.cancel_task(execution_id)
        return build_result(
            envelope,
            status="succeeded",
            result=result_payload,
            started_at=started_at,
        )

    def _execute_navigation(self, envelope: MessageEnvelope, started_at: str) -> dict:
        command = envelope.payload.get("command") or {}
        if envelope.message_type == "nav.single_goal":
            navigation = getattr(self.task_executor, "navigation", None)
            if navigation is None:
                raise ProtocolError("NAVIGATION_STACK_UNAVAILABLE", "navigation adapter is not configured")
            global_controller = str(command.get("global_controller") or "theta_star")
            global_setter = getattr(navigation, "set_global_controller", None)
            if callable(global_setter):
                global_setter(global_controller)
            goal = {
                "x": float(command["x"]),
                "y": float(command["y"]),
                "yaw": float(command["yaw"]),
                "require_yaw": bool(command.get("require_yaw", True)),
                "global_controller": global_controller,
            }
            if self.navigation_boundary:
                self.navigation_boundary.validate_point(
                    str(command.get("map_id") or ""), goal["x"], goal["y"],
                    expected_revision=command.get("boundary_revision"),
                )
            self._structured(
                "DEBUG", "planner", "planner.single_goal_input", "单点规划输入",
                data=goal, envelope=envelope, pose={"x": goal["x"], "y": goal["y"], "yaw": goal["yaw"]},
            )
            accepted = navigation.send_waypoints([goal], lambda *_: None, lambda *_: None)
            if not accepted:
                raise ProtocolError("NAV_GOAL_REJECTED", "Nav2 rejected the single-point goal")
            result_payload = {"accepted": True, "goal": goal}
        elif envelope.message_type == "nav.status":
            if not self.navigation_stack_adapter:
                raise ProtocolError("NAVIGATION_STACK_UNAVAILABLE", "navigation stack adapter is not configured")
            result_payload = self.navigation_stack_adapter.status()
        elif envelope.message_type in {"nav.initial_pose", "nav.relocalize"}:
            # Pose commands must remain usable while nav.start/restart is
            # waiting for localization; sharing the stack-management lock
            # creates a deadlock where localization can never be initialized.
            if not self.localization_adapter:
                raise ProtocolError("LOCALIZATION_UNAVAILABLE", "localization adapter is not configured")
            if not self._localization_command_lock.acquire(blocking=True, timeout=1.0):
                raise ProtocolError(
                    "LOCALIZATION_COMMAND_BUSY",
                    "another operator localization command is still running",
                )
            begin_operator = getattr(
                self.localization_adapter,
                "begin_operator_localization",
                None,
            )
            end_operator = getattr(
                self.localization_adapter,
                "end_operator_localization",
                None,
            )
            operator_scope_started = False
            task_pause_transaction = None
            set_progress = getattr(
                self.localization_adapter, "set_attempt_progress_callback", None
            )
            try:
                self._structured(
                    "DEBUG", "relocalization" if envelope.message_type == "nav.relocalize" else "localization",
                    f"{envelope.message_type}.input", "定位算法输入",
                    data={
                        "seed_source": command.get("seed_source"),
                        "wait_seconds": command.get("wait_seconds"),
                        "candidate_waypoint_count": len(command.get("waypoints") or []),
                        "coordinate_mode": command.get("coordinate_mode"),
                        "scene_scope": command.get("scene_scope"),
                        "localization_mode": command.get("localization_mode"),
                    }, envelope=envelope,
                )
                if callable(begin_operator):
                    begin_operator()
                    operator_scope_started = True
                task_pause_transaction = self._pause_active_task_for_operator_localization()
                if callable(set_progress):
                    set_progress(
                        lambda payload: self._emit_command_progress(envelope, started_at, payload)
                    )
                bootstrap_started_at = now_iso()
                localization_bootstrap = self._ensure_initial_pose_subscriber()
                if localization_bootstrap is not None:
                    localization_bootstrap = {
                        **localization_bootstrap,
                        "started_at": localization_bootstrap.get("started_at") or bootstrap_started_at,
                        "finished_at": localization_bootstrap.get("finished_at") or now_iso(),
                    }
                if envelope.message_type == "nav.initial_pose":
                    # A hand-selected pose and the legacy ``seed_source=rtk``
                    # are hypotheses, never a direct map anchor.  Run the
                    # common NDT order (mapping origin → supplied candidate →
                    # global fallback); RTK remains a secondary correction
                    # after the best NDT result has handed off to FAST-LIO.
                    try:
                        origin = (
                            self.map_activation_adapter.mapping_start_pose()
                            if self.map_activation_adapter else None
                        )
                    except ProtocolError as exc:
                        origin = {
                            "unavailable_error_code": exc.code,
                            "unavailable_error_message": exc.message,
                        }
                    waypoints = list(command.get("waypoints") or [])
                    if str(command.get("seed_source") or "") != "rtk":
                        pose = self._resolve_localization_seed(command)
                        waypoints.insert(0, {
                            key: pose[key] for key in ("x", "y", "z", "yaw")
                            if pose.get(key) is not None
                        })
                    trusted_seed, trusted_evidence = self._trusted_rtk_seed_for_command(command)
                    result_payload = self.localization_adapter.progressive_relocalize(
                        origin=origin,
                        waypoints=waypoints,
                        trusted_seed=trusted_seed,
                        wait_seconds=float(command.get("wait_seconds", 180.0)),
                    )
                    if trusted_evidence is not None and isinstance(result_payload, dict):
                        result_payload["trusted_rtk_seed"] = trusted_evidence
                else:
                    seed_source = str(command.get("seed_source") or "last_trusted")
                    if seed_source == "progressive":
                        try:
                            origin = (
                                self.map_activation_adapter.mapping_start_pose()
                                if self.map_activation_adapter else None
                            )
                        except ProtocolError as exc:
                            origin = {
                                "unavailable_error_code": exc.code,
                                "unavailable_error_message": exc.message,
                            }
                        trusted_seed, trusted_evidence = self._trusted_rtk_seed_for_command(command)
                        result_payload = self.localization_adapter.progressive_relocalize(
                            origin=origin,
                            waypoints=list(command.get("waypoints") or []),
                            trusted_seed=trusted_seed,
                            wait_seconds=float(command.get("wait_seconds", 180.0)),
                        )
                        if trusted_evidence is not None and isinstance(result_payload, dict):
                            result_payload["trusted_rtk_seed"] = trusted_evidence
                    elif seed_source in {"global", "quick_then_global"}:
                        try:
                            origin = (
                                self.map_activation_adapter.mapping_start_pose()
                                if self.map_activation_adapter else None
                            )
                        except ProtocolError as exc:
                            origin = {
                                "unavailable_error_code": exc.code,
                                "unavailable_error_message": exc.message,
                            }
                        supplied = all(
                            command.get(field) is not None for field in ("x", "y", "yaw")
                        )
                        manual_seed = None
                        if supplied:
                            manual_seed = {
                                field: float(command.get(field, 0.0))
                                for field in ("x", "y", "z", "yaw")
                            }
                        result_payload = self.localization_adapter.quick_then_global_relocalize(
                            origin=origin,
                            manual_seed=manual_seed,
                            scene_scope=str(command.get("scene_scope") or "indoor"),
                            coordinate_mode=str(command.get("coordinate_mode") or "local_only"),
                            wait_seconds=float(command.get("wait_seconds", 120.0)),
                        )
                    else:
                        seed = self._resolve_localization_seed(command)
                        result_payload = self.localization_adapter.active_relocalize(seed)
                # Every externally initiated localization command follows the
                # same post-NDT correction contract.  This closes the gap for
                # nav.relocalize/nav.initial_pose callers that do not go
                # through TaskExecutor startup or self-healing.
                correction = getattr(self.localization_adapter, "control_localization_correction", None)
                if callable(correction) and isinstance(result_payload, dict):
                    mode = str(command.get("localization_mode") or "ndt").strip().lower()
                    if mode not in {"ndt", "rtk", "ukf"}:
                        mode = "ndt"
                    transaction_id = f"{envelope.message_type}:{envelope.message_id}:secondary:{mode}"
                    secondary_started_at = now_iso()
                    secondary = correction(transaction_id, mode, "start") or {}
                    secondary = {
                        **secondary,
                        "mode": mode,
                        "transaction_id": transaction_id,
                        "started_at": secondary_started_at,
                    }
                    decision_reader = getattr(self.localization_adapter, "localization_decision", None)
                    if not secondary.get("accepted"):
                        if mode == "ndt" and str(secondary.get("status") or "") == "unavailable":
                            secondary.update({"status": "skipped", "finished_at": now_iso()})
                        else:
                            raise ProtocolError(
                                "LOCALIZATION_SECONDARY_CORRECTION_REJECTED",
                                str(secondary.get("message") or secondary.get("status") or "secondary correction rejected"),
                                details={"secondary_correction": secondary},
                            )
                    elif callable(decision_reader):
                        deadline = time.monotonic() + 30.0
                        terminal = False
                        while time.monotonic() < deadline:
                            decision = decision_reader()
                            one_shot = decision.get("one_shot_correction") if isinstance(decision, dict) else {}
                            if isinstance(one_shot, dict) and str(one_shot.get("transaction_id") or "") == transaction_id:
                                status = str(one_shot.get("status") or "")
                                if status in {"completed", "failed", "cancelled", "rejected"}:
                                    secondary = {**secondary, **one_shot, "finished_at": now_iso()}
                                    terminal = True
                                    break
                            time.sleep(0.2)
                        if not terminal:
                            try:
                                correction(transaction_id, mode, "cancel")
                            except Exception:
                                LOGGER.exception(
                                    "failed to cancel timed-out secondary correction %s",
                                    transaction_id,
                                )
                            secondary.update({"status": "timed_out", "finished_at": now_iso()})
                            raise ProtocolError(
                                "LOCALIZATION_SECONDARY_CORRECTION_TIMEOUT",
                                f"secondary {mode} correction timed out",
                                details={"secondary_correction": secondary},
                            )
                        if str(secondary.get("status") or "") != "completed":
                            raise ProtocolError(
                                "LOCALIZATION_SECONDARY_CORRECTION_FAILED",
                                str(secondary.get("reason") or f"secondary {mode} correction failed"),
                                details={"secondary_correction": secondary},
                            )
                    else:
                        secondary.update({"status": "unverifiable", "finished_at": now_iso()})
                        raise ProtocolError(
                            "LOCALIZATION_SECONDARY_CORRECTION_UNVERIFIABLE",
                            "secondary correction was accepted but no decision reader is available",
                            details={"secondary_correction": secondary},
                        )
                    result_payload["secondary_correction"] = secondary
                # Every localization snapshot must identify its map.  The
                # route planner uses this identity to reject terminal results
                # from a previously selected map.
                result_payload = dict(result_payload or {})
                route_map = (command.get("route_snapshot") or {}).get("map") or {}
                result_payload.setdefault("map_id", command.get("map_id") or route_map.get("map_id"))
                result_payload.setdefault("map_version", command.get("map_version") or route_map.get("map_version"))
                if localization_bootstrap is not None:
                    result_payload["localization_bootstrap"] = localization_bootstrap
                progressive_initialization = (
                    envelope.message_type == "nav.relocalize"
                    and str(command.get("seed_source") or "")
                    in {"progressive", "quick_then_global"}
                )
                # An operator-triggered relocalization is a recovery
                # transaction, not a pose-only probe.  Once NDT, FAST-LIO
                # handoff and the secondary correction complete, always
                # reactivate the prepared execution group.  Previously the
                # common case (an already-present /initialpose subscriber)
                # defaulted to False here, leaving Nav2 configured/inactive
                # and showing "navigation stack not ready" after a successful
                # localization result.
                should_start_navigation = (
                    (envelope.message_type == "nav.relocalize" and self.navigation_stack_adapter is not None)
                    or localization_bootstrap is not None
                    or bool(command.get("start_navigation", progressive_initialization))
                )
                if should_start_navigation:
                    result_payload = dict(result_payload or {})
                    navigation_started_at = now_iso()
                    try:
                        navigation_start = self._start_navigation_after_localization()
                    except ProtocolError as exc:
                        # The NDT/LIO transaction may already be accepted
                        # when the independent Nav2 execution activation
                        # fails.  Preserve that completed localization result
                        # in the terminal command rather than replacing it
                        # with an opaque NAV_COMMAND_FAILED.
                        localization_snapshot = result_payload.get("localization_attempts")
                        if not isinstance(localization_snapshot, dict):
                            localization_snapshot = {
                                **result_payload,
                                "state": "accepted",
                            }
                        raise ProtocolError(
                            exc.code,
                            exc.message,
                            details={
                                **result_payload,
                                "localization_attempts": localization_snapshot,
                                "navigation_start": {
                                    "status": "failed",
                                    "started_at": navigation_started_at,
                                    "finished_at": now_iso(),
                                    "error_code": exc.code,
                                    "error_message": exc.message,
                                    "details": dict(exc.details or {}),
                                },
                            },
                        ) from exc
                    result_payload["navigation_start"] = {
                        **navigation_start,
                        "started_at": navigation_start.get("started_at") or navigation_started_at,
                        "finished_at": navigation_start.get("finished_at") or now_iso(),
                    }
                    # The localization adapter's last progress frame is
                    # emitted before Nav2 starts. Publish this terminal
                    # transition as progress as well, otherwise a page that
                    # is rendering the live command snapshot keeps the
                    # navigation stage at "待执行" despite Nav2 being ready.
                    self._emit_command_progress(envelope, started_at, result_payload)
                task_resume = self._resume_task_after_operator_localization(
                    task_pause_transaction
                )
                if task_resume is not None:
                    result_payload["task_resume"] = task_resume
                self._structured(
                    "DEBUG", "relocalization" if envelope.message_type == "nav.relocalize" else "localization",
                    f"{envelope.message_type}.output", "定位算法输出",
                    data=result_payload, envelope=envelope,
                )
            finally:
                try:
                    if callable(set_progress):
                        set_progress(None)
                finally:
                    try:
                        if operator_scope_started and callable(end_operator):
                            end_operator()
                    finally:
                        self._localization_command_lock.release()
        else:
            wait_seconds = 90.0 if envelope.message_type in {
                "nav.start", "nav.restart", "nav.recover",
            } else 5.0
            if not self._navigation_command_lock.acquire(blocking=True, timeout=wait_seconds):
                raise ProtocolError("NAV_COMMAND_BUSY", "another navigation command is still running")
            try:
                if not self.navigation_stack_adapter:
                    raise ProtocolError("NAVIGATION_STACK_UNAVAILABLE", "navigation stack adapter is not configured")
                if envelope.message_type in {"nav.start", "nav.restart", "nav.recover"}:
                    prepare = getattr(self.navigation_stack_adapter, "prepare", None)
                    # Older optional adapters retain the historical stack-only
                    # command.  Production uses prepare→NDT→handoff→activate
                    # so these three entry points cannot bypass localization.
                    if not callable(prepare) or self.localization_adapter is None:
                        fallback = getattr(
                            self.navigation_stack_adapter,
                            "start" if envelope.message_type == "nav.start" else envelope.message_type.split(".", 1)[1],
                        )
                        result_payload = fallback(command)
                        self._await_navigation_stack_ready(
                            timeout_seconds=45.0,
                            message=f"Nav2 did not become ready after {envelope.message_type}",
                        )
                    else:
                        if envelope.message_type in {"nav.restart", "nav.recover"}:
                            readiness = getattr(self.localization_adapter, "lio_readiness", None)
                            local_readiness = (readiness() or {}) if callable(readiness) else {}
                            local_ready = bool(local_readiness.get("ready")) if callable(readiness) else True
                            if not local_ready:
                                restart_lio = getattr(self.navigation_stack_adapter, "restart_localization", None)
                                if callable(restart_lio):
                                    restart_lio()
                        stack_prepare = prepare({"reason": envelope.message_type})
                        wait_prepared = getattr(self.localization_adapter, "wait_until_prepared", None)
                        if callable(wait_prepared) and not wait_prepared(timeout_seconds=45.0):
                            raise ProtocolError(
                                "NAV_STACK_PREPARE_FAILED",
                                "navigation map/safety group did not become prepared",
                            )
                        self.safety.state.nav_ready = False
                        localization = self._run_navigation_command_localization(envelope, command)
                        activate = getattr(self.navigation_stack_adapter, "activate_execution", None)
                        if not callable(activate):
                            raise ProtocolError(
                                "NAVIGATION_EXECUTION_UNAVAILABLE",
                                "prepared navigation stack cannot activate its execution group",
                            )
                        activation = activate()
                        self._await_navigation_stack_ready(
                            timeout_seconds=45.0,
                            message=f"Nav2 did not become ready after {envelope.message_type}",
                        )
                        result_payload = {
                            "action": envelope.message_type,
                            "stack_prepare": stack_prepare,
                            "localization": localization,
                            "execution_activation": activation,
                            "navigation_allowed": True,
                        }
                elif envelope.message_type == "nav.stop":
                    deactivate = getattr(self.navigation_stack_adapter, "deactivate_execution", None)
                    if callable(deactivate):
                        deactivate()
                    result_payload = self.navigation_stack_adapter.stop(command)
                    self.safety.state.nav_ready = False
                else:
                    raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
            finally:
                self._navigation_command_lock.release()
        return build_result(
            envelope,
            status="succeeded",
            result=result_payload,
            started_at=started_at,
        )

    def _resolve_localization_seed(self, command: dict) -> dict:
        seed = dict(command)
        if all(seed.get(field) is not None for field in ("x", "y", "yaw")):
            return seed
        map_id = str(command.get("map_id") or self.safety.state.current_map_id or "")
        map_version = str(command.get("map_version") or self.safety.state.current_map_version or "")
        seed_source = str(command.get("seed_source") or "last_trusted")
        if seed_source == "mapping_start":
            resolved = self.map_activation_adapter.mapping_start_pose() if self.map_activation_adapter else {}
        else:
            resolved = self.store.load_last_trusted_pose(map_id, map_version) or {}
        # Preserve operator search controls while taking x/y/yaw from the
        # map-scoped trusted seed.  Previously wait_seconds/max_attempts were
        # silently lost here, forcing every stored-pose search back to its
        # internal defaults.
        seed = {**resolved, **{
            key: command[key]
            for key in ("wait_seconds", "max_attempts", "candidate_wait_seconds", "source")
            if command.get(key) is not None
        }}
        if not all(seed.get(field) is not None for field in ("x", "y", "yaw")):
            raise ProtocolError(
                "RELOCALIZATION_SEED_UNAVAILABLE",
                "no trusted pose exists for this map; select an approximate pose on the map first",
            )
        return seed

    def _emit_command_progress(self, envelope: MessageEnvelope, started_at: str, result: dict) -> None:
        try:
            progress_result = dict(result) if isinstance(result, dict) else {}
            if envelope.message_type == "task.start":
                progress_result["startup_progress"] = _startup_progress_detail(progress_result)
            progress = build_progress(
                envelope,
                result=progress_result,
                started_at=started_at,
            )
            self.publish_progress(envelope.payload["command_id"], progress)
        except Exception:
            LOGGER.exception(
                "failed to publish localization command progress for %s",
                envelope.payload.get("command_id"),
            )

    def _execute_teleop(self, envelope: MessageEnvelope, started_at: str) -> dict:
        teleop_adapter = self.localization_adapter
        if not teleop_adapter:
            raise ProtocolError("TELEOP_UNAVAILABLE", "teleop adapter is not configured")
        command = envelope.payload.get("command") or {}
        action = envelope.message_type.removeprefix("teleop.")
        bridge_status = None
        bridge_free_actions = {
            "person_follow_status", "person_follow_stop",
            "skill_list", "skill_status", "skill_cancel",
        }
        if self.teleop_control_adapter and action not in bridge_free_actions:
            velocity_actions = {
                "move_forward", "move_backward", "move_left", "move_right",
                "turn_left", "turn_right", "move_velocity", "move_stop",
            }
            bridge_status = (
                self.teleop_control_adapter.recent_ready_status()
                if action in velocity_actions
                else self.teleop_control_adapter.ensure_ready()
            )
        if self.person_follow_controller and action in {
            "takeover_enter", "takeover_exit", "move_forward", "move_backward",
            "move_left", "move_right", "turn_left", "turn_right", "move_velocity", "skill",
            "shake_hand", "two_leg_stand",
        }:
            self.person_follow_controller.stop("manual_teleop_override")
        assist_action = bool(command.get("assist", False))
        if assist_action and not self.task_executor.has_active_task():
            raise ProtocolError(
                "MANUAL_ASSIST_REQUIRES_ACTIVE_TASK",
                "manual assist actions require an active navigation task",
            )
        if assist_action:
            self.safety.validate_manual_assist()
        if action == "takeover_enter":
            if bool(command.get("assist", False)):
                if not self.task_executor.has_active_task():
                    raise ProtocolError(
                        "MANUAL_ASSIST_REQUIRES_ACTIVE_TASK",
                        "manual assist requires an active navigation task",
                    )
                self.safety.validate_manual_assist()
                self.safety.state.control_mode = "manual_assist"
                result_payload = {
                    "mode": "manual_assist",
                    "motion_topic": "/cmd_vel_assist",
                    "detail": "Nav2 remains active; assist is collision-monitored and time-bounded",
                }
            else:
                result_payload = teleop_adapter.confirmed_remote_teleop_action(
                    "stand_up", {"standing_up", "standing"}, {"stand_up_retrying"}
                )
                self.safety.state.control_mode = "manual_takeover"
            self._release_temporary_fusion_for_manual_control(action)
        elif action == "takeover_exit":
            if self.safety.state.control_mode == "manual_assist":
                result_payload = self._manual_assist_velocity(teleop_adapter)
                result_payload["mode"] = "autonomous"
            else:
                teleop_adapter.teleop_velocity(0.0, 0.0, 0.0)
                result_payload = teleop_adapter.release_to_remote_control()
            self.safety.state.control_mode = "autonomous"
        elif action == "stand_up":
            result_payload = teleop_adapter.confirmed_remote_teleop_action(
                "stand_up", {"standing_up", "standing"}, {"stand_up_retrying"}
            )
            if assist_action:
                self.safety.validate_manual_assist()
                self.safety.state.control_mode = "manual_assist"
                result_payload["mode"] = "manual_assist"
            else:
                self.safety.state.control_mode = "manual_takeover"
                self._release_temporary_fusion_for_manual_control(action)
        elif action == "lie_down":
            if self.person_follow_controller:
                self.person_follow_controller.stop("lie_down")
            teleop_adapter.teleop_velocity(0.0, 0.0, 0.0)
            result_payload = teleop_adapter.remote_teleop_action("lie_down")
            if assist_action:
                self.safety.validate_manual_assist()
                self.safety.state.control_mode = "manual_assist"
                result_payload["mode"] = "manual_assist"
            else:
                self.safety.state.control_mode = "autonomous"
        elif action == "shake_hand":
            result_payload = teleop_adapter.confirmed_remote_teleop_action(
                "shake_hand", {"greeting"}
            )
            if assist_action:
                self.safety.validate_manual_assist()
                self.safety.state.control_mode = "manual_assist"
                result_payload["mode"] = "manual_assist"
            else:
                self.safety.state.control_mode = "manual_takeover"
                self._release_temporary_fusion_for_manual_control(action)
        elif action == "two_leg_stand":
            result_payload = teleop_adapter.confirmed_remote_teleop_action(
                "two_leg_stand", {"two_leg_standing"}
            )
            if assist_action:
                self.safety.validate_manual_assist()
                self.safety.state.control_mode = "manual_assist"
                result_payload["mode"] = "manual_assist"
            else:
                self.safety.state.control_mode = "manual_takeover"
                self._release_temporary_fusion_for_manual_control(action)
        elif action == "move_stop":
            if self.person_follow_controller:
                self.person_follow_controller.stop("move_stop")
            if self.safety.state.control_mode == "manual_assist":
                result_payload = self._manual_assist_velocity(teleop_adapter)
            else:
                result_payload = teleop_adapter.teleop_velocity(0.0, 0.0, 0.0)
        elif action == "person_follow_start":
            if not self.person_follow_controller:
                raise ProtocolError("PERSON_FOLLOW_UNAVAILABLE", "person follow controller is not configured")
            if self.task_executor.has_active_task():
                raise ProtocolError("PERSON_FOLLOW_UNAVAILABLE", "cannot start person follow while a navigation task is active")
            stand = teleop_adapter.confirmed_remote_teleop_action(
                "stand_up", {"standing_up", "standing"}, {"stand_up_retrying"}
            )
            result_payload = {
                "follow": self.person_follow_controller.start(str(command.get("track_id") or "")),
                "stand": stand,
            }
            self.safety.state.control_mode = "manual_takeover"
            self._release_temporary_fusion_for_manual_control(action)
        elif action == "person_follow_stop":
            if not self.person_follow_controller:
                raise ProtocolError("PERSON_FOLLOW_UNAVAILABLE", "person follow controller is not configured")
            result_payload = self.person_follow_controller.stop("operator_stop")
        elif action == "person_follow_status":
            if not self.person_follow_controller:
                raise ProtocolError("PERSON_FOLLOW_UNAVAILABLE", "person follow controller is not configured")
            result_payload = self.person_follow_controller.status()
        elif action == "skill":
            if not self.skill_executor:
                raise ProtocolError("TELEOP_UNAVAILABLE", "skill executor is not configured")
            run = self.skill_executor.start(
                envelope.payload["command_id"],
                command,
                lambda outcome: self._complete_skill(envelope, started_at, outcome),
            )
            return None
        elif action == "skill_list":
            if not self.skill_executor:
                raise ProtocolError("TELEOP_UNAVAILABLE", "skill executor is not configured")
            result_payload = {"presets": self.skill_executor.list_presets()}
        elif action == "skill_status":
            if not self.skill_executor:
                raise ProtocolError("TELEOP_UNAVAILABLE", "skill executor is not configured")
            result_payload = self.skill_executor.status(str(command.get("command_id") or ""))
        elif action == "skill_cancel":
            if not self.skill_executor:
                raise ProtocolError("TELEOP_UNAVAILABLE", "skill executor is not configured")
            result_payload = self.skill_executor.cancel(str(command.get("command_id") or ""))
        elif action == "move_forward":
            result_payload = self._manual_assist_velocity(
                teleop_adapter, vx=float(command.get("vx", 0.35)))
        elif action == "move_backward":
            result_payload = self._manual_assist_velocity(
                teleop_adapter, vx=float(command.get("vx", -0.35)))
        elif action == "move_left":
            result_payload = self._manual_assist_velocity(
                teleop_adapter, vy=float(command.get("vy", 0.25)))
        elif action == "move_right":
            result_payload = self._manual_assist_velocity(
                teleop_adapter, vy=float(command.get("vy", -0.25)))
        elif action == "turn_left":
            result_payload = self._manual_assist_velocity(
                teleop_adapter, yaw_rate=float(command.get("yaw_rate", 0.45)))
        elif action == "turn_right":
            result_payload = self._manual_assist_velocity(
                teleop_adapter, yaw_rate=float(command.get("yaw_rate", -0.45)))
        elif action == "move_velocity":
            vx = max(-0.5, min(0.5, float(command.get("vx", 0.0))))
            vy = max(-0.5, min(0.5, float(command.get("vy", 0.0))))
            yaw_rate = max(-0.5, min(0.5, float(command.get("yaw_rate", 0.0))))
            result_payload = self._manual_assist_velocity(teleop_adapter, vx=vx, vy=vy, yaw_rate=yaw_rate)
        elif action in {"speed_micro", "speed_slow", "speed_normal", "speed_fast"}:
            result_payload = teleop_adapter.remote_teleop_action(action)
        elif action == "passive":
            if self.person_follow_controller:
                self.person_follow_controller.stop("passive")
            result_payload = teleop_adapter.confirmed_remote_teleop_action(
                "passive", {"passive"}, {"passive_failed"}
            )
            # The bridge remains resident. Its remote passive handler clears
            # the held stick command before issuing the vendor emergency stop.
            self.safety.state.control_mode = "autonomous"
        else:
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        if bridge_status is not None:
            result_payload["teleop_bridge"] = bridge_status
        return build_result(
            envelope,
            status="succeeded",
            result=result_payload,
            started_at=started_at,
        )

    def _manual_assist_velocity(self, adapter, vx: float = 0.0, vy: float = 0.0, yaw_rate: float = 0.0) -> dict:
        """Route assist through the Nav2 safety pipeline while a task remains active."""
        if self.safety.state.control_mode != "manual_assist":
            return adapter.teleop_velocity(vx=vx, vy=vy, yaw_rate=yaw_rate)
        if not self.task_executor.has_active_task():
            self.safety.state.control_mode = "autonomous"
            return adapter.teleop_velocity(vx=vx, vy=vy, yaw_rate=yaw_rate)
        self.safety.validate_manual_assist()
        # A second clamp at Edge bounds a malformed remote payload before the
        # velocity optimizer applies its independent final limits.
        vx = max(-0.10, min(0.10, float(vx)))
        vy = max(-0.10, min(0.10, float(vy)))
        yaw_rate = max(-0.25, min(0.25, float(yaw_rate)))
        result = adapter.manual_assist_velocity(vx=vx, vy=vy, yaw_rate=yaw_rate)
        result["mode"] = "manual_assist"
        return result

    def _release_manual_control_for_task(self) -> None:
        """Make a task.start an explicit, safe hand-back to autonomous motion.

        This is intentionally invoked only after all task-start interlocks
        have passed.  `manual_takeover` needs a direct zero command followed by
        a confirmed SDK release.  `manual_assist` has no SDK ownership, but a
        zero assist sample prevents a stale short-lived assist command from
        carrying into the new route.
        """
        mode = self.safety.state.control_mode
        if mode not in {"manual_assist", "manual_takeover"}:
            return
        adapter = self.localization_adapter
        if not adapter:
            raise ProtocolError("TELEOP_UNAVAILABLE", "teleop adapter is not configured")
        if mode == "manual_assist":
            adapter.manual_assist_velocity(0.0, 0.0, 0.0)
        else:
            adapter.teleop_velocity(0.0, 0.0, 0.0)
            adapter.release_to_remote_control()
        self.safety.state.control_mode = "autonomous"
        LOGGER.info("released %s for explicit navigation task start", mode)

    def _complete_skill(self, envelope: MessageEnvelope, started_at: str, outcome: dict) -> None:
        """Publish the terminal result after the asynchronous local skill ends."""
        status = outcome.pop("status", "failed")
        result = build_result(
            envelope,
            status=status,
            result=outcome,
            started_at=started_at,
            error_code=outcome.get("error_code", ""),
            error_message=outcome.get("error_message", ""),
        )
        command_id = envelope.payload["command_id"]
        self.store.save_command_result(command_id, result)
        self.publish_result(command_id, result)

    def _execute_sensor(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if not self.sensor_control_adapter:
            raise ProtocolError("SENSOR_CONTROL_UNAVAILABLE", "sensor control adapter is not configured")
        if envelope.message_type != "sensor.restart":
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        result_payload = self.sensor_control_adapter.restart(
            (envelope.payload.get("command") or {}).get("sensor")
        )
        return build_result(
            envelope,
            status="succeeded",
            result=result_payload,
            started_at=started_at,
        )

    def _execute_charge(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if not self.charge_control_adapter:
            raise ProtocolError("CHARGE_CONTROL_UNAVAILABLE", "charge control adapter is not configured")
        if envelope.message_type == "charge.start":
            result_payload = self.charge_control_adapter.start()
        elif envelope.message_type == "charge.stop":
            result_payload = self.charge_control_adapter.stop()
        else:
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        return build_result(envelope, status="succeeded", result=result_payload, started_at=started_at)

    def _execute_motion_control(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if not self.charge_control_adapter:
            raise ProtocolError("MOTION_CONTROL_UNAVAILABLE", "charge control adapter is not configured")
        if self.task_executor.has_active_task() and envelope.message_type == "motion.stop":
            raise ProtocolError("TASK_ACTIVE", "cannot stop motion control while a navigation task is active")
        if envelope.message_type == "motion.start":
            result_payload = self.charge_control_adapter.start_motion_control()
        elif envelope.message_type == "motion.stop":
            result_payload = self.charge_control_adapter.stop_motion_control()
        else:
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        return build_result(envelope, status="succeeded", result=result_payload, started_at=started_at)

    def _execute_audio(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if not self.audio_control_adapter:
            raise ProtocolError("AUDIO_CONTROL_UNAVAILABLE", "audio control adapter is not configured")
        if envelope.message_type != "audio.volume":
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        command = envelope.payload.get("command") or {}
        result_payload = self.audio_control_adapter.set_volume(
            str(command.get("target") or ""),
            int(command.get("volume", 0)),
        )
        return build_result(envelope, status="succeeded", result=result_payload, started_at=started_at)

    def _execute_mapping(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if not self.mapping_adapter:
            raise ProtocolError("MAPPING_UNAVAILABLE", "mapping adapter is not configured")
        command = envelope.payload.get("command") or {}
        if envelope.message_type == "mapping.start":
            result_payload = self.mapping_adapter.start_mapping(command)
        elif envelope.message_type == "mapping.origin_start":
            result_payload = self.mapping_adapter.start_origin_lock(command)
        elif envelope.message_type == "mapping.origin_cancel":
            result_payload = self.mapping_adapter.cancel_origin_lock(command)
        elif envelope.message_type == "mapping.origin_extract_global":
            result_payload = self.mapping_adapter.extract_global_enu(command)
        elif envelope.message_type == "mapping.slam_start":
            result_payload = self.mapping_adapter.start_slam_warmup(command)
        elif envelope.message_type == "mapping.begin":
            result_payload = self.mapping_adapter.begin_mapping(command)
        elif envelope.message_type == "mapping.save":
            result_payload = self.mapping_adapter.save_mapping(command)
        elif envelope.message_type == "mapping.cancel":
            result_payload = self.mapping_adapter.cancel_mapping(command)
        elif envelope.message_type == "mapping.status":
            result_payload = self.mapping_adapter.status()
        elif envelope.message_type == "mapping.scene_semantics":
            result_payload = self.mapping_adapter.build_scene_semantics(command)
        elif envelope.message_type == "mapping.scene_semantics_status":
            result_payload = self.mapping_adapter.scene_semantics_status()
        else:
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        return build_result(
            envelope,
            status="succeeded",
            result=result_payload,
            started_at=started_at,
        )

    def _execute_map(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if envelope.message_type == "map.boundary_apply":
            if not self.navigation_boundary:
                raise ProtocolError("BOUNDARY_UNAVAILABLE", "navigation boundary manager is not configured")
            command = envelope.payload.get("command") or {}
            current_map_id = str(self.safety.state.current_map_id or "")
            if current_map_id and current_map_id != str(command.get("map_id") or ""):
                raise ProtocolError(
                    "BOUNDARY_MAP_MISMATCH",
                    f"导航边界地图 {command.get('map_id')} 与当前地图 {current_map_id} 不一致",
                )
            result_payload = self.navigation_boundary.apply(command)
            boundary_reloader = getattr(self.navigation_stack_adapter, "reload_boundary_filter", None)
            if callable(boundary_reloader):
                result_payload["nav2_filter_reload"] = boundary_reloader()
            return build_result(envelope, status="succeeded", result=result_payload, started_at=started_at)
        if not self.map_activation_adapter:
            raise ProtocolError("MAP_ACTIVATION_UNAVAILABLE", "map activation adapter is not configured")
        command = envelope.payload.get("command") or {}
        if envelope.message_type == "map.activate":
            context = self.task_executor.context
            if context and context.state not in self.task_executor.TERMINAL_STATES:
                raise ProtocolError(
                    "ROBOT_BUSY",
                    "机器人正在执行任务，不能切换活动地图",
                )
            result_payload = self.map_activation_adapter.activate(command)
            if self.navigation_boundary:
                self.navigation_boundary.activate_map(command.get("map_id"), command.get("map_version"))
            if not self.navigation_stack_adapter:
                raise ProtocolError("MAP_RELOAD_UNAVAILABLE", "navigation stack adapter is not configured")
            deactivate = getattr(self.navigation_stack_adapter, "deactivate_execution", None)
            if callable(deactivate):
                deactivate()
            self.safety.state.nav_ready = False
            active_files = result_payload["current_map"]["active_files"]
            reload_if_running = getattr(self.navigation_stack_adapter, "reload_map_if_running", None)
            if callable(reload_if_running):
                result_payload["map_reload"] = reload_if_running(
                    active_files["map.pcd"],
                    active_files["map.yaml"],
                )
            else:
                result_payload["map_reload"] = self.navigation_stack_adapter.reload_map(
                    active_files["map.pcd"],
                    active_files["map.yaml"],
                )
            boundary_reloader = getattr(self.navigation_stack_adapter, "reload_boundary_filter", None)
            if not result_payload["map_reload"].get("deferred") and callable(boundary_reloader):
                result_payload["boundary_filter_reload"] = boundary_reloader()
            # A map-local pose cannot be carried across maps.  Production map
            # activation owns the complete map-specific localization flow; a
            # legacy adapter without ROS localization support retains the old
            # explicit-reseed result for backwards compatibility.
            self.safety.state.localization_status = "initializing"
            self.safety.state.localization_normal_since_monotonic = 0.0
            prepare = getattr(self.navigation_stack_adapter, "prepare", None)
            if not callable(prepare) or self.localization_adapter is None:
                result_payload["localization_reset_required"] = True
            else:
                active_map = result_payload.get("current_map") or {}
                # The activated package is authoritative for the map scene
                # contract. A stale/missing cloud field must not turn an
                # outdoor RTK-fixed map into indoor/local_only behavior.
                active_scene = str(active_map.get("scene_scope") or "").strip().lower()
                active_coordinate = str(active_map.get("coordinate_mode") or "").strip().lower()
                requested_scene = str(command.get("scene_scope") or "").strip().lower()
                requested_coordinate = str(command.get("coordinate_mode") or "").strip().lower()
                effective_scene = active_scene
                effective_coordinate = active_coordinate
                # A route explicitly selecting outdoor/transition is allowed
                # to supply the scene for older RTK-fixed packages whose
                # manifest only recorded the coordinate mode. Never let an
                # outdoor request override an active local_only package.
                if requested_scene in {"outdoor", "transition"} and active_coordinate == "rtk_fixed":
                    effective_scene = requested_scene
                if effective_coordinate not in {"rtk_fixed", "local_only"}:
                    effective_coordinate = requested_coordinate
                localization_command = {
                    **command,
                    "scene_scope": (effective_scene or requested_scene or "indoor").lower(),
                    "coordinate_mode": (effective_coordinate or requested_coordinate or "local_only").lower(),
                    # This is the first-waypoint policy only. The map's
                    # map_localization_mode (e.g. rtk_ndt) is never used here.
                    "localization_mode": str(command.get("localization_mode") or "").lower(),
                    "waypoints": list(command.get("waypoints") or []),
                }
                set_progress = getattr(
                    self.localization_adapter, "set_attempt_progress_callback", None
                )
                if callable(set_progress):
                    set_progress(
                        lambda payload: self._emit_command_progress(
                            envelope, started_at, payload
                        )
                    )
                try:
                    self._emit_command_progress(
                        envelope,
                        started_at,
                        {
                            "state": "running",
                            "selected_stage": "navigation_prepare",
                            "navigation_lifecycle": {"status": "preparing"},
                        },
                    )
                    prepared = prepare({"reason": "map.activate"})
                    wait_prepared = getattr(self.localization_adapter, "wait_until_prepared", None)
                    if callable(wait_prepared) and not wait_prepared(timeout_seconds=45.0):
                        raise ProtocolError(
                            "NAV_STACK_PREPARE_FAILED",
                            "navigation map/safety group did not become prepared after map activation",
                        )
                    self._emit_command_progress(
                        envelope,
                        started_at,
                        {"state": "running", "selected_stage": "fast_lio_readiness"},
                    )
                    localization = self._run_navigation_command_localization(
                        envelope, localization_command
                    )
                    self._emit_command_progress(
                        envelope,
                        started_at,
                        {
                            **localization,
                            "state": "running",
                            "selected_stage": "navigation_execution_activate",
                        },
                    )
                    activation = getattr(self.navigation_stack_adapter, "activate_execution", None)
                    if not callable(activation):
                        raise ProtocolError(
                            "NAVIGATION_EXECUTION_UNAVAILABLE",
                            "prepared navigation stack cannot activate its execution group",
                        )
                    result_payload["navigation_prepare"] = prepared
                    result_payload["localization"] = localization
                    result_payload["execution_activation"] = activation()
                    self._await_navigation_stack_ready(
                        timeout_seconds=45.0,
                        message="navigation execution group did not become ready after map activation",
                    )
                    result_payload["localization_reset_required"] = False
                    result_payload["navigation_allowed"] = True
                except Exception:
                    if callable(deactivate):
                        try:
                            deactivate()
                        except Exception:
                            LOGGER.exception("failed to deactivate execution group after map activation failure")
                    self.safety.state.nav_ready = False
                    raise
                finally:
                    if callable(set_progress):
                        set_progress(None)
        elif envelope.message_type == "map.optimize":
            if not self.mapping_adapter:
                raise ProtocolError("MAPPING_UNAVAILABLE", "mapping adapter is not configured")
            context = self.task_executor.context
            if context and context.state not in self.task_executor.TERMINAL_STATES:
                raise ProtocolError("ROBOT_BUSY", "机器人正在执行任务，不能离线优化地图")
            source_dir = self.map_activation_adapter.resolve_source_dir(command)
            result_payload = self.mapping_adapter.optimize_historical_map(command, source_dir)
        else:
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        return build_result(
            envelope,
            status="succeeded",
            result=result_payload,
            started_at=started_at,
        )

    @staticmethod
    def _module_for_command(command_type: str) -> str:
        if command_type == "nav.relocalize":
            return "relocalization"
        if command_type == "nav.initial_pose":
            return "localization"
        if command_type == "nav.single_goal":
            return "planner"
        if command_type.startswith("nav."):
            return "navigation"
        if command_type == "map.boundary_apply":
            return "boundary"
        return "system"

    def _structured(self, level, module, event_code, message, *, data=None, envelope=None, pose=None):
        if not self.structured_logs:
            return
        context = {}
        if envelope:
            command = envelope.payload.get("command") or {}
            route_map = ((command.get("route_snapshot") or {}).get("map") or {})
            context = {
                "trace_id": envelope.trace_id,
                "command_id": envelope.payload.get("command_id"),
                "task_execution_id": envelope.payload.get("task_execution_id"),
                "map_id": command.get("map_id") or route_map.get("map_id"),
            }
        if pose:
            context["pose"] = pose
        self.structured_logs.emit(level, module, event_code, message, data=data, **context)

    def _state_version(self) -> int:
        context = self.task_executor.context
        if not context or context.state in self.task_executor.TERMINAL_STATES:
            return 0
        return context.state_version
