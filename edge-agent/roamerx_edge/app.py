from __future__ import annotations

import argparse
import logging
import shutil
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path

from .alert_bridge import AlertBridge
from .audio_control_adapter import AudioControlAdapter
from .charge_control_adapter import ChargeControlAdapter
from .command_processor import CommandProcessor
from .config import EdgeConfig
from .local_store import LocalStore
from .localization_recovery import planar_distance_m, select_recovery_seed
from .map_activation_adapter import MapActivationAdapter
from .map_set_coordinator import MapSetCoordinator
from .mapping_adapter import MappingAdapter
from .media_client import MediaClient
from .mqtt_client import EdgeMqttClient
from .structured_logging import StructuredLogEmitter
from .navigation_stack_adapter import NavigationStackAdapter
from .navigation_boundary import NavigationBoundaryManager
from .protocol import ProtocolError, build_envelope, now_iso
from .power_mode_controller import PowerModeController
from .person_follow_controller import PersonFollowController
from .ros_adapter import ROS_AVAILABLE, RosAdapter, RosRuntime, rclpy
from .rosbag_recorder import RosbagRecorder
from .safety_policy import RuntimeSafetyState, SafetyPolicy
from .sensor_control_adapter import SensorControlAdapter
from .self_healing import (
    FaultDiagnoser,
    SelfHealingCoordinator,
    SelfHealingPolicyError,
)
from .task_executor import TaskExecutor
from .telemetry_collector import TelemetryCollector
from .teleop_control_adapter import TeleopControlAdapter
from .trajectory_buffer import TrajectoryBuffer
from .system_telemetry import SystemTelemetryProbe

LOGGER = logging.getLogger(__name__)


class EdgeAgentApplication:
    def __init__(self, config: EdgeConfig, navigation=None, config_path: str = "config.yaml") -> None:
        self.config = config
        self.store = LocalStore(
            config.storage.sqlite_path,
            trajectory_outbox_limit=config.storage.trajectory_outbox_limit,
            system_log_outbox_limit=config.storage.system_log_outbox_limit,
        )
        reconciled = self.store.reconcile_unfinished_self_healing()
        pruned = self.store.prune_self_healing()
        if reconciled["episodes"] or reconciled["actions"]:
            LOGGER.warning("reconciled unfinished self-healing records: %s", reconciled)
        if pruned["episodes"] or pruned["actions"]:
            LOGGER.info("pruned self-healing detail after daily aggregation: %s", pruned)
        self.stop_event = threading.Event()
        self.safety_state = RuntimeSafetyState(
            control_mode="autonomous",
            current_map_id=config.robot.current_map_id,
            current_map_version=config.robot.current_map_version,
        )
        self.telemetry = TelemetryCollector(config.robot, self.safety_state)
        self.telemetry.configure_system_probe_staleness(
            config.telemetry.system_probe_stale_seconds
        )
        self.system_telemetry = SystemTelemetryProbe(
            config.telemetry,
            self.telemetry,
            config.charge_control,
        )
        self.mqtt = EdgeMqttClient(config, self.store)
        self._latest_task_state_event: dict = {}
        self.structured_logs = StructuredLogEmitter(self.mqtt.publish_system_logs)
        self.navigation_boundary = NavigationBoundaryManager(
            self.store,
            self.structured_logs,
            mask_dir=config.navigation_stack.boundary_filter_dir,
            map_yaml_path=str(Path(config.mapping.map_dir) / "map.yaml"),
        )
        self.media_client = MediaClient(config.media, config.robot.id)
        self.ros_runtime = None
        if navigation is None:
            if not ROS_AVAILABLE:
                raise RuntimeError("ROS2 is required unless a navigation adapter is injected")
            rclpy.init(args=None)
            navigation = RosAdapter(
                config.ros,
                config.safety,
                self.telemetry,
                self.safety_state,
                config.mapping,
                config.imu_cross_check,
                self.structured_logs,
                config.ros_callback_optimization,
            )
            self.ros_runtime = RosRuntime(navigation)
        self.navigation = navigation
        self.map_activation_adapter = MapActivationAdapter(config, self.safety_state, config_path)
        self.navigation_stack_adapter = NavigationStackAdapter(config.navigation_stack)
        self._localization_recovery_lock = threading.Lock()
        self._fusion_profile_lock = threading.RLock()
        self._active_fusion_profile_generation = 0
        self.map_set_coordinator = MapSetCoordinator(self.map_activation_adapter, self.navigation_stack_adapter)
        self.navigation_rosbag = RosbagRecorder(
            config.navigation_stack.rosbag_script,
            config.navigation_stack.rosbag_stop_timeout_seconds,
        )
        self.task_executor = TaskExecutor(
            self.store,
            navigation,
            event_callback=self._publish_task_event,
            start_result_callback=self._publish_start_result,
            final_waypoint_tolerance_m=config.safety.final_waypoint_tolerance_m,
            arrival_degraded_tolerance_m=config.safety.arrival_degraded_tolerance_m,
            docking_goal_tolerance_m=config.safety.docking_goal_tolerance_m,
            docking_goal_yaw_tolerance_rad=config.safety.docking_goal_yaw_tolerance_rad,
            arrival_adjust_max_distance_m=config.safety.arrival_adjust_max_distance_m,
            arrival_adjust_clearance_lookahead_m=config.safety.arrival_adjust_clearance_lookahead_m,
            arrival_adjust_speed_mps=config.safety.arrival_adjust_speed_mps,
            arrival_adjust_yaw_rate_rps=config.safety.arrival_adjust_yaw_rate_rps,
            arrival_adjust_timeout_seconds=config.safety.arrival_adjust_timeout_seconds,
            arrival_adjust_scan_max_age_seconds=config.safety.arrival_adjust_scan_max_age_seconds,
            arrival_adjust_safety_grace_seconds=config.safety.arrival_adjust_safety_grace_seconds,
            arrival_micro_adjust_mode=config.safety.arrival_micro_adjust_mode,
            arrival_micro_adjust_max_initial_error_m=config.safety.arrival_micro_adjust_max_initial_error_m,
            arrival_micro_adjust_total_budget_m=config.safety.arrival_micro_adjust_total_budget_m,
            arrival_micro_adjust_step_m=config.safety.arrival_micro_adjust_step_m,
            arrival_micro_adjust_max_steps=config.safety.arrival_micro_adjust_max_steps,
            arrival_nav2_reapproach_max_error_m=config.safety.arrival_nav2_reapproach_max_error_m,
            arrival_precision_recovery_retry_seconds=config.safety.arrival_precision_recovery_retry_seconds,
            arrival_micro_goal_tolerance_m=config.safety.arrival_micro_goal_tolerance_m,
            arrival_ndt_max_fitness_score=config.safety.arrival_ndt_max_fitness_score,
            arrival_convergence_samples=config.safety.arrival_convergence_samples,
            standup_confirmation_timeout_seconds=config.safety.standup_confirmation_timeout_seconds,
            stop_confirmation_seconds=config.safety.stop_confirmation_seconds,
            map_set_coordinator=self.map_set_coordinator,
            obstacle_speech=config.obstacle_speech,
            waypoint_speech=config.waypoint_speech,
            rosbag_recorder=self.navigation_rosbag,
            localization_recovery_callback=self._handle_task_localization_loss,
        )
        self.self_healing = SelfHealingCoordinator(
            store=self.store,
            event_callback=self._publish_task_event,
            ndt_failure_score=config.safety.ndt_failure_score,
            rtk_required_samples=config.safety.localization_rtk_required_samples,
        )
        set_log_context_provider = getattr(navigation, "set_log_context_provider", None)
        if callable(set_log_context_provider):
            set_log_context_provider(lambda: {
                "trace_id": self.task_executor.context.trace_id if self.task_executor.context else None,
                "task_execution_id": self.task_executor.context.task_execution_id if self.task_executor.context else None,
                "map_id": config.robot.current_map_id,
            })
        set_localization_failure_callback = getattr(
            navigation, "set_localization_failure_callback", None
        )
        if callable(set_localization_failure_callback):
            set_localization_failure_callback(self._handle_task_localization_loss)
        set_localization_recovery_callback = getattr(
            navigation, "set_localization_recovery_callback", None
        )
        if callable(set_localization_recovery_callback):
            set_localization_recovery_callback(self._handle_task_localization_recovered)
        set_recovery_lease_callbacks = getattr(navigation, "set_recovery_lease_callbacks", None)
        if callable(set_recovery_lease_callbacks):
            set_recovery_lease_callbacks(
                self.task_executor.acquire_recovery,
                self.task_executor.release_recovery,
                self.task_executor.recovery_snapshot,
            )
        set_self_healing_callback = getattr(navigation, "set_self_healing_callback", None)
        if callable(set_self_healing_callback):
            set_self_healing_callback(self._handle_navigation_self_healing)
        # Deduplicates localization alerts the same way _mapping_divergence_notified
        # does for SLAM divergence: one alert per episode, re-armed on recovery.
        self._localization_alert_notified = False
        # Separate latch from the localization one: a disagreeing gyro is a
        # hardware finding that outlives any single localization episode.
        self._imu_mismatch_notified = False
        set_imu_cross_check_callback = getattr(navigation, "set_imu_cross_check_callback", None)
        if callable(set_imu_cross_check_callback):
            set_imu_cross_check_callback(self._handle_imu_cross_check_report)
        set_trusted_pose_callback = getattr(navigation, "set_trusted_pose_callback", None)
        if callable(set_trusted_pose_callback):
            set_trusted_pose_callback(self._persist_last_trusted_pose)
        self.mapping_adapter = MappingAdapter(config.mapping, self.media_client, config.scene_semantics)
        self._mapping_divergence_notified = False
        self._mapping_rescue_lock = threading.Lock()
        set_mapping_divergence_callback = getattr(navigation, "set_mapping_divergence_callback", None)
        if callable(set_mapping_divergence_callback):
            set_mapping_divergence_callback(self._handle_mapping_divergence_event)
        origin_payload_snapshot = getattr(navigation, "origin_payload_snapshot", None)
        if callable(origin_payload_snapshot):
            self.mapping_adapter.set_origin_payload_provider(origin_payload_snapshot)
        self.teleop_control_adapter = TeleopControlAdapter(config.teleop_control)
        self.person_follow_controller = PersonFollowController(navigation, config.person_follow)
        self.sensor_control_adapter = SensorControlAdapter(config.sensor_control)
        self.power_mode_controller = PowerModeController(config.power_mode)
        self.charge_control_adapter = ChargeControlAdapter(
            config.charge_control,
            self.power_mode_controller,
            self.store,
            power_refresh=self._refresh_charge_power,
        )
        self.charge_control_adapter.set_low_battery_handler(self._handle_low_battery_alert)
        self._docking_undock_pending = False
        self.charge_control_adapter.set_charge_started_handler(self._handle_charge_started)
        self.charge_control_adapter.set_full_charge_handler(self._finish_docking_undock)
        self.task_executor.docking_arrived_handler = self._start_docking_charge
        self.audio_control_adapter = AudioControlAdapter(config.audio_control)
        self.safety = SafetyPolicy(config.safety, self.safety_state)
        self.commands = CommandProcessor(
            robot_id=config.robot.id,
            store=self.store,
            safety=self.safety,
            task_executor=self.task_executor,
            publish_ack=self.mqtt.publish_ack,
            publish_result=self.mqtt.publish_result,
            mapping_adapter=self.mapping_adapter,
            map_activation_adapter=self.map_activation_adapter,
            navigation_stack_adapter=self.navigation_stack_adapter,
            localization_adapter=navigation,
            teleop_control_adapter=self.teleop_control_adapter,
            person_follow_controller=self.person_follow_controller,
            sensor_control_adapter=self.sensor_control_adapter,
            charge_control_adapter=self.charge_control_adapter,
            audio_control_adapter=self.audio_control_adapter,
            structured_logs=self.structured_logs,
            navigation_boundary=self.navigation_boundary,
            temporary_fusion_release_callback=self._restore_temporary_fusion_profile,
        )
        self.trajectory = TrajectoryBuffer(
            robot_id=config.robot.id,
            session_id=self.mqtt.session_id,
            store=self.store,
            batch_size=config.telemetry.trajectory_batch_size,
            flush_seconds=config.telemetry.trajectory_flush_seconds,
        )
        self.alerts = AlertBridge(self.telemetry, self.task_executor, self.mqtt.publish_alert)
        self.mqtt.set_handlers(self.commands.handle_command, self._handle_sync_message)
        self._threads: list[threading.Thread] = []

    def start(self) -> None:
        self.power_mode_controller.reconcile_startup()
        if self.ros_runtime:
            self.ros_runtime.start()
            self.navigation.wait_until_ready(timeout_seconds=10.0)
        # If Edge restarted while localization stayed alive, reconcile a
        # non-nominal profile left by the previous process. The generation in
        # /localization/decision makes this conditional and race-safe.
        self._restore_temporary_fusion_profile("edge_startup_reconcile")
        self.mqtt.connect()
        if not self.mqtt.wait_connected(15):
            LOGGER.warning("MQTT initial connection did not complete within 15 seconds")
        self.task_executor.restore_paused_localization_recovery()
        self.task_executor.report_startup_interruption()
        self.system_telemetry.poll()
        self.charge_control_adapter.observe_power(self.telemetry.latest_power())
        self.power_mode_controller.refresh_service_status(self.telemetry.latest_power())
        self._publish_online()
        self.structured_logs.emit("INFO", "system", "edge.started", "Edge Agent 已启动")
        self.structured_logs.flush()
        self._publish_sync_request()
        self._threads = [
            threading.Thread(target=self._heartbeat_loop, daemon=True, name="heartbeat"),
            threading.Thread(target=self._status_loop, daemon=True, name="status"),
            threading.Thread(target=self._trajectory_loop, daemon=True, name="trajectory"),
            threading.Thread(target=self._outbox_loop, daemon=True, name="outbox"),
            threading.Thread(target=self._system_telemetry_loop, daemon=True, name="system-telemetry"),
            threading.Thread(target=self._boundary_loop, daemon=True, name="navigation-boundary"),
            threading.Thread(target=self._log_flush_loop, daemon=True, name="structured-log-flush"),
        ]
        for thread in self._threads:
            thread.start()

    def stop(self) -> None:
        # ROS is still available here, so release a temporary fusion lease
        # before shutting its executor down. The localization-side TTL remains
        # the final fallback if this best-effort call cannot complete.
        self._restore_temporary_fusion_profile("edge_shutdown")
        self.stop_event.set()
        self.trajectory.flush_active()
        self.person_follow_controller.stop("edge_shutdown")
        self.task_executor.stop()
        for thread in self._threads:
            thread.join(timeout=3)
        self.structured_logs.emit("INFO", "system", "edge.stopping", "Edge Agent 正在停止")
        self.structured_logs.flush()
        self.mqtt.disconnect()
        if self.ros_runtime:
            self.ros_runtime.stop()
            rclpy.shutdown()
        self.store.close()

    def run_forever(self) -> None:
        self.start()
        self.stop_event.wait()

    def _start_docking_charge(self, docking: dict) -> None:
        """Called only after the second docking waypoint has been reached."""
        self.task_executor.report_docking_charge("task.docking_contact_checking", message="已到充电桩，正在检查蓝牙与极片")
        passive = getattr(getattr(self, "navigation", None), "confirmed_remote_teleop_action", None)
        if callable(passive):
            passive("passive", {"passive"}, {"passive_failed"}, timeout_seconds=5.0)
        self._docking_undock_pending = True
        result = self.charge_control_adapter.start()
        if result.get("dock_ready"):
            return
        # Reaching the dock and starting its controller is a successful
        # hand-off. Contact can settle after Nav2 has completed; the pending
        # monitor owns that wait and starts charging on the first valid sample.
        self.task_executor.report_docking_charge(
            "task.docking_contact_waiting",
            message="充电服务已启动，等待蓝牙与极片接触",
            extra={"dock": result},
        )

    def _refresh_charge_power(self) -> dict:
        power = self.system_telemetry.poll_power()
        self.power_mode_controller.refresh_service_status(power)
        return power

    def _handle_charge_started(self, result: dict) -> None:
        self.task_executor.report_docking_charge(
            "task.docking_charge_started",
            message="极片接触已确认，已自动开始充电",
            extra={"charge_stage": result.get("charge_stage"), "dock": result},
        )

    def _finish_docking_undock(self) -> None:
        """Leave the dock only for a completed one-key docking operation."""
        if not self._docking_undock_pending:
            return
        self._docking_undock_pending = False
        snapshot = self.power_mode_controller.snapshot()
        services = list((snapshot.get("services") or {}).values())
        if snapshot.get("mode") != "normal" or snapshot.get("transition_state") != "ready" or not services or not all(service.get("matches_mode") for service in services):
            LOGGER.error("full charge restored incompletely; refusing automatic undock: %s", snapshot)
            return
        remote_action = getattr(self.navigation, "remote_teleop_action", None)
        velocity = getattr(self.navigation, "teleop_velocity", None)
        if not callable(remote_action) or not callable(velocity):
            LOGGER.error("automatic undock unavailable: remote teleop adapter missing")
            return
        remote_action("speed_micro")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            velocity(vx=-0.60)
            time.sleep(0.15)
        velocity()
        LOGGER.info("full charge automatic undock completed")

    def _publish_online(self) -> None:
        self.mqtt.publish_presence(
            "presence.online",
            {
                "agent_version": self.config.robot.agent_version,
                "software_version": "genisom-roamerx-open",
                "boot_id": self._read_boot_id(),
                "capabilities": [
                    "task.start",
                    "task.pause",
                    "task.resume",
                    "task.cancel",
                    "task.recover.v1",
                    "mapping.start",
                    "mapping.save",
                    "mapping.cancel",
                    "mapping.status",
                    "nav.status",
                    "nav.start",
                    "nav.restart",
                    "nav.recover",
                    "nav.stop",
                    "nav.initial_pose",
                    "nav.relocalize",
                    "nav.single_goal",
                    "sensor.restart",
                    "charge.start",
                    "charge.stop",
                    "motion.start",
                    "motion.stop",
                    "audio.volume",
                    "map.activate",
                    "map.optimize",
                    "map.boundary_apply",
                    "diagnostics.log_config",
                    "diagnostics.nav_rosbag_stop",
                    "structured_logs.v1",
                    "navigation_boundaries.v1",
                    "map_set.v1",
                    "teleop.takeover_enter",
                    "teleop.takeover_exit",
                    "teleop.stand_up",
                    "teleop.lie_down",
                    "teleop.speed_micro",
                    "teleop.speed_slow",
                    "teleop.speed_normal",
                    "teleop.speed_fast",
                    "teleop.move_forward",
                    "teleop.move_backward",
                    "teleop.move_left",
                    "teleop.move_right",
                    "teleop.turn_left",
                    "teleop.turn_right",
                    "teleop.move_velocity",
                    "teleop.move_stop",
                    "teleop.passive",
                    "teleop.skill",
                    "teleop.skill_status",
                    "teleop.skill_cancel",
                    "map.uploaded",
                    "telemetry.pose",
                    "trajectory.batch",
                    "alert.event",
                ],
                "current_map": self._current_map_payload(),
            },
            retain=True,
        )

    def _publish_sync_request(self) -> None:
        context = self.task_executor.context
        latest_event = self._latest_task_state_event
        event_matches_context = bool(
            context
            and latest_event.get("task_execution_id") == context.task_execution_id
            and latest_event.get("state") == context.state
        )
        payload = {
            "current_task_execution_id": context.task_execution_id if context else None,
            "local_task_state": context.state if context else None,
            "local_task_state_version": context.state_version if context else 0,
            "local_task_reason_code": (
                latest_event.get("reason_code") if event_matches_context else None
            ),
            "local_task_reason_message": (
                latest_event.get("reason_message") if event_matches_context else None
            ),
            "last_processed_command_id": None,
            "last_trajectory_seq": -1,
            "outbox_pending": self.store.outbox_count(),
        }
        self.mqtt.publish(
            f"robots/{self.config.robot.id}/sync/state",
            build_envelope(
                message_type="sync.request",
                robot_id=self.config.robot.id,
                session_id=self.mqtt.session_id,
                payload=payload,
            ),
            qos=1,
        )

    def _handle_sync_message(self, raw: dict) -> None:
        message_type = raw.get("message_type")
        payload = raw.get("payload") or {}
        if message_type == "trajectory.ack":
            self.trajectory.handle_ack(payload)
        elif message_type == "sync.response":
            action = payload.get("action")
            self.task_executor.reconcile_center_state_version(
                str(payload.get("task_execution_id") or ""),
                payload.get("expected_task_state"),
                payload.get("expected_state_version"),
            )
            if action == "cancel" and self.task_executor.context:
                try:
                    self.task_executor.cancel_task(self.task_executor.context.task_execution_id)
                except Exception:
                    LOGGER.exception("failed to apply sync cancellation")
            elif action == "hold" and self.task_executor.context:
                try:
                    self.task_executor.reconcile_center_state(
                        str(payload.get("task_execution_id") or ""),
                        payload.get("expected_task_state"),
                    )
                except Exception:
                    LOGGER.exception("failed to reconcile task state from center")
            # continue/hold/report_only intentionally never auto-start motion after process restart.

    def _heartbeat_loop(self) -> None:
        started = time.monotonic()
        while not self.stop_event.wait(self.config.telemetry.heartbeat_interval_seconds):
            context = self.task_executor.context
            self.mqtt.publish_presence(
                "presence.heartbeat",
                {
                    "uptime_seconds": round(time.monotonic() - started),
                    "outbox_pending": self.store.outbox_count(),
                    "last_processed_command_id": None,
                    "current_task_execution_id": context.task_execution_id if context else None,
                    "current_map": self._current_map_payload(),
                },
            )
            # A broker QoS acknowledgement does not prove the center worker
            # persisted the event. Re-send the versioned state periodically so
            # a transient center restart or uplink loss self-heals safely.
            self._publish_sync_request()

    def _status_loop(self) -> None:
        while not self.stop_event.wait(self.config.telemetry.status_interval_seconds):
            try:
                mapping_status = self.mapping_adapter.status()
                self._observe_mapping_health(mapping_status)
                if mapping_status.get("process_alive"):
                    self.navigation.safety_state.nav_ready = False
                else:
                    self.navigation.wait_until_ready(timeout_seconds=0.5)
                context = self.task_executor.context
                snapshot_started = time.perf_counter()
                snapshot = self.telemetry.build_status_snapshot(
                    context.task_execution_id if context and self.task_executor.has_active_task() else None
                )
                record_performance = getattr(self.navigation, "record_callback_performance", None)
                if callable(record_performance):
                    record_performance(
                        "telemetry_snapshot",
                        time.perf_counter() - snapshot_started,
                    )
                mapping_status["odometry"] = dict((snapshot.get("sensors") or {}).get("odometry") or {})
                snapshot["current_map"] = self._current_map_payload()
                snapshot["map_set"] = self.map_set_coordinator.status()
                snapshot["mapping"] = mapping_status
                obstacle_snapshot = getattr(self.navigation, "obstacle_monitor_snapshot", None)
                snapshot["navigation"] = obstacle_snapshot() if callable(obstacle_snapshot) else {}
                snapshot["power_mode"] = self.power_mode_controller.snapshot()
                self.mqtt.publish_status(snapshot)
                pose = self.telemetry.latest_pose()
                pose_payload = {"x": pose.x, "y": pose.y, "yaw": pose.yaw} if pose else None
                localization = snapshot.get("localization") or {}
                self.structured_logs.emit(
                    "DEBUG", "localization", "localization.sample", "定位变量采样",
                    data={
                        "status": snapshot.get("localization_status"),
                        "quality": snapshot.get("localization_quality"),
                        "decision": localization.get("decision"),
                    },
                    pose=pose_payload,
                    map_id=self.config.robot.current_map_id,
                    task_execution_id=context.task_execution_id if context else None,
                )
                navigation_snapshot = snapshot.get("navigation") or {}
                self.structured_logs.emit(
                    "DEBUG", "avoidance", "avoidance.sample", "避障变量采样",
                    data={
                        "front_obstacle_distance_m": navigation_snapshot.get("front_obstacle_distance_m"),
                        "requested_speed_mps": navigation_snapshot.get("requested_speed_mps"),
                        "actual_speed_mps": navigation_snapshot.get("actual_speed_mps"),
                        "global_plan": navigation_snapshot.get("global_plan"),
                    },
                    pose=pose_payload,
                    map_id=self.config.robot.current_map_id,
                    task_execution_id=context.task_execution_id if context else None,
                )
                self.structured_logs.flush()
            except Exception:
                LOGGER.exception("failed to publish telemetry status")

    def _trajectory_loop(self) -> None:
        while not self.stop_event.wait(0.5):
            try:
                context = self.task_executor.context
                pose = self.telemetry.latest_pose()
                if context and context.state in self.task_executor.TERMINAL_STATES:
                    self.trajectory.flush_active()
                    continue
                if not context or context.state != "running" or not pose:
                    continue
                message = self.trajectory.sample(
                    context.task_execution_id,
                    self.config.robot.current_map_id,
                    self.config.robot.current_map_version,
                    pose,
                )
                if message:
                    self.mqtt.replay_outbox()
            except Exception:
                LOGGER.exception("failed to sample trajectory")

    def _log_flush_loop(self) -> None:
        while not self.stop_event.wait(1.0):
            try:
                self.structured_logs.flush()
            except Exception:
                LOGGER.exception("failed to flush structured logs")

    def _outbox_loop(self) -> None:
        while not self.stop_event.wait(2):
            self.mqtt.replay_outbox()

    def _boundary_loop(self) -> None:
        while not self.stop_event.wait(0.1):
            pose = self.telemetry.latest_pose()
            if not pose:
                continue
            try:
                observation = self.navigation_boundary.observe_pose(
                    self.config.robot.current_map_id, float(pose.x), float(pose.y),
                )
                for transition, zone in observation["events"]:
                    transition_labels = {
                        "entered": "进入", "exited": "离开",
                        "approaching": "接近", "approach_cleared": "远离",
                    }
                    self.structured_logs.emit(
                        "WARNING" if transition in {"entered", "approaching"} else "INFO",
                        "boundary", f"boundary.zone_{transition}",
                        f"机器人{transition_labels.get(transition, transition)}导航区域 {zone}",
                        data={"zone": zone}, map_id=self.config.robot.current_map_id,
                        pose={"x": pose.x, "y": pose.y, "yaw": pose.yaw},
                    )
                if observation["speed_changed"]:
                    setter = getattr(self.navigation, "set_boundary_speed_limit", None)
                    speed_applied = True
                    try:
                        if callable(setter):
                            setter(observation["speed_limit_mps"])
                    except Exception as exc:
                        speed_applied = False
                        self.navigation_boundary.retry_speed_application()
                        self.structured_logs.emit(
                            "ERROR", "boundary", "boundary.speed_limit_failed",
                            "导航区域限速应用失败，将自动重试",
                            data={"speed_limit_mps": observation["speed_limit_mps"], "error": str(exc)},
                            map_id=self.config.robot.current_map_id,
                            pose={"x": pose.x, "y": pose.y, "yaw": pose.yaw},
                        )
                    if speed_applied:
                        self.structured_logs.emit(
                            "INFO", "boundary", "boundary.speed_limit_changed",
                            "导航区域限速已更新",
                            data={"speed_limit_mps": observation["speed_limit_mps"]},
                            map_id=self.config.robot.current_map_id,
                            pose={"x": pose.x, "y": pose.y, "yaw": pose.yaw},
                        )
                if observation["violation"] and observation["violation_changed"]:
                    stop_errors = []
                    cancel = getattr(self.navigation, "cancel_navigation", None)
                    try:
                        if callable(cancel):
                            cancel(timeout_seconds=1.0)
                    except Exception as exc:
                        stop_errors.append(f"cancel_navigation: {exc}")
                    velocity = getattr(self.navigation, "teleop_velocity", None)
                    try:
                        if callable(velocity):
                            velocity(0.0, 0.0, 0.0)
                    except Exception as exc:
                        stop_errors.append(f"zero_velocity: {exc}")
                    self.structured_logs.emit(
                        "ERROR", "boundary", "boundary.runtime_violation",
                        "机器人触发硬导航边界，已取消导航并停车",
                        data={"violation": observation["violation"], "stop_errors": stop_errors},
                        map_id=self.config.robot.current_map_id,
                        pose={"x": pose.x, "y": pose.y, "yaw": pose.yaw},
                    )
                    if stop_errors:
                        self.navigation_boundary.retry_violation_stop()
            except Exception as exc:
                LOGGER.warning("navigation boundary monitor failed: %s", exc)

    def _system_telemetry_loop(self) -> None:
        while True:
            cooling = self.power_mode_controller.snapshot().get("mode") == "cooling_standby"
            interval = (
                self.config.telemetry.cooling_system_probe_interval_seconds
                if cooling
                else self.config.telemetry.system_probe_interval_seconds
            )
            if self.stop_event.wait(interval):
                break
            self.system_telemetry.poll()
            power = self.telemetry.latest_power()
            self.charge_control_adapter.observe_power(power)
            self.power_mode_controller.refresh_service_status(power)

    def _observe_mapping_health(self, mapping_status: dict) -> None:
        progress = mapping_status.get("save_progress") or {}
        health = progress.get("slam_health") or {}
        diverged = (
            progress.get("error_code") == "SLAM_DIVERGED"
            or health.get("state") == "diverged"
        )
        state = str(mapping_status.get("state") or "")
        if not diverged:
            if state in {"", "idle", "cancelled", "exited", "completed"}:
                self._mapping_divergence_notified = False
            return
        # Stale save_progress from a previous mapping session must not put a
        # patrol into SAFE_HOLD / passive.
        if state in {"", "idle", "cancelled", "exited", "completed", "failed"}:
            return
        if self._mapping_divergence_notified:
            return
        self._mapping_divergence_notified = True
        reason = progress.get("error") or health.get("warning") or "SLAM pose diverged"
        LOGGER.error("Mapping diverged; stopping capture notification: %s", reason)
        self._request_mapping_passive()
        try:
            self.alerts.emit_system_alert(
                "slam_diverged",
                "high",
                "SLAM_DIVERGED",
                attributes={
                    "message": reason,
                    "map_name": mapping_status.get("map_name", ""),
                    "mapping_session_id": mapping_status.get("mapping_session_id", ""),
                },
                detection={"label": "建图定位已发散", "class": "slam_diverged", "confidence": 1},
                component="mapping",
            )
        except Exception:
            LOGGER.exception("failed to publish mapping divergence alert")
        threading.Thread(
            target=self._speak_mapping_diverged,
            daemon=True,
            name="mapping-diverged-speech",
        ).start()

    def _request_mapping_passive(self) -> None:
        navigation = getattr(self, "navigation", None)
        stop_velocity = getattr(navigation, "teleop_velocity", None)
        if callable(stop_velocity):
            try:
                stop_velocity(0.0, 0.0, 0.0)
            except Exception:
                LOGGER.exception("failed to clear teleop velocity after mapping SAFE_HOLD")
        passive = getattr(
            navigation,
            "confirmed_remote_teleop_action",
            None,
        )
        if not callable(passive):
            return
        for attempt in range(1, 4):
            try:
                passive("passive", {"passive"}, {"passive_failed"}, timeout_seconds=5.0)
                return
            except Exception:
                if attempt == 3:
                    LOGGER.exception("failed to put robot into passive after mapping SAFE_HOLD")
                else:
                    LOGGER.warning("mapping SAFE_HOLD passive confirmation failed; retry %d/3", attempt + 1)
                    time.sleep(0.25)

    def _mapping_session_is_active(self) -> bool:
        adapter = getattr(self, "mapping_adapter", None)
        session = getattr(adapter, "session", None)
        state = str(getattr(session, "state", "") or "")
        return state not in {"", "idle", "cancelled", "exited", "completed", "failed"}

    def _handle_mapping_divergence_event(self, event: dict) -> None:
        """Immobilize only during an active mapping session, then rescue after flush.

        `/slam/divergence_event` is transient_local. A leftover SAFE_HOLD from
        the previous mapping run would otherwise lie the dog down mid-patrol.
        """
        if not self._mapping_session_is_active():
            LOGGER.warning(
                "ignoring mapping divergence event with no active mapping session: %s",
                event,
            )
            return
        self._request_mapping_passive()
        if not bool(event.get("writer_flushed")):
            LOGGER.error("SAFE_HOLD writer did not flush; preserving session for manual recovery: %s", event)
            return
        if not self._mapping_rescue_lock.acquire(blocking=False):
            return
        try:
            self.mapping_adapter.auto_rescue_diverged_mapping(event)
        except Exception:
            LOGGER.exception("automatic diverged-map rescue failed")
        finally:
            self._mapping_rescue_lock.release()

    def _speak_mapping_diverged(self) -> None:
        text = "建图定位已发散，请立即停止移动。回到地图页保存救援地图。"
        commands = []
        if shutil.which("espeak-ng"):
            commands.append(["espeak-ng", "-v", "zh", text])
        if shutil.which("espeak"):
            commands.append(["espeak", "-v", "zh", text])
        if shutil.which("spd-say"):
            commands.append(["spd-say", "-l", "zh", text])
        if shutil.which("speaker-test"):
            commands.append(["timeout", "2", "speaker-test", "-t", "sine", "-f", "880", "-l", "1"])
        for command in commands:
            try:
                subprocess.run(
                    command,
                    check=False,
                    timeout=8,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                return
            except Exception:
                LOGGER.exception("mapping divergence local speech failed command=%s", command)

    def _handle_low_battery_alert(self, episode_id: str, battery_percent: int) -> None:
        """Stop patrol motion and report low battery without requesting docking."""
        context = self.task_executor.context
        execution_id = context.task_execution_id if context else ""
        continuous_loop_session_id = (
            str(getattr(context, "loop_session_id", "") or "")
            if context
            and bool(getattr(context, "record_rosbag", False))
            and bool(getattr(context, "loop_execution", False))
            and bool(getattr(context, "continuous_rosbag", False))
            else ""
        )
        docking_active = bool(
            self.task_executor.has_active_task()
            and context
            and (context.docking or {}).get("enabled")
        )
        action = "docking_already_active" if docking_active else "no_active_task"
        # Low battery is terminal, not recoverable.  Stop locally before the
        # alert crosses an unreliable network and use force-exit semantics so
        # a paused/interrupted task cannot retain a context or restart later.
        if context and not docking_active:
            try:
                stopped = self.task_executor.force_exit(
                    execution_id,
                    reason_code="LOW_BATTERY",
                    reason_message=f"battery {battery_percent}% is below the patrol threshold",
                )
                if continuous_loop_session_id:
                    # The cloud also emits an idempotent stop command when it
                    # closes the loop.  Stop locally as well because the low
                    # battery path must remain terminal even while MQTT is
                    # unavailable.
                    stopped["rosbag"] = self.task_executor.stop_loop_rosbag(
                        continuous_loop_session_id
                    )
                action = "force_exited"
                LOGGER.warning("low battery force-exited active navigation; automatic return is disabled")
            except Exception:
                stopped = {"robot_stopped": False, "cleared": False}
                action = "navigation_stop_failed_alert_only"
                self.navigation.stop_motion()
                LOGGER.exception("failed to force low-battery task exit")
        else:
            stopped = {
                "robot_stopped": self.navigation.is_robot_stopped(),
                "cleared": not bool(context),
            }
        pose = self.navigation.latest_pose()
        pose_payload = {"frame_id": "map"}
        if pose is not None:
            pose_payload.update(
                {
                    "x": float(pose.x),
                    "y": float(pose.y),
                    "yaw": float(getattr(pose, "yaw", 0.0)),
                }
            )
        self.mqtt.publish_alert(
            {
                "event_id": episode_id,
                "event_type": "low_battery_alert",
                "severity": "high",
                "occurred_at": now_iso(),
                "task_execution_id": execution_id or None,
                "map_id": self.safety_state.current_map_id,
                "map_version": self.safety_state.current_map_version,
                "pose": pose_payload,
                "source": {
                    "component": "charge_control_adapter",
                    "code": "LOW_BATTERY_ALERT",
                    "model_version": self.config.robot.agent_version,
                },
                "detection": {
                    "label": "低电量停车告警",
                    "class": "low_battery",
                    "confidence": 1.0,
                },
                "attributes": {
                    "low_battery_episode_id": episode_id,
                    "battery_percent": battery_percent,
                    "threshold_percent": self.config.charge_control.low_battery_start_percent,
                    "rearm_percent": self.config.charge_control.low_battery_rearm_percent,
                    "action": action,
                    "robot_stopped": bool(stopped.get("robot_stopped")),
                    "task_context_cleared": bool(stopped.get("cleared")),
                    "automatic_docking": False,
                },
            }
        )

    def _publish_start_result(self, command_id: str, status: str, result: dict, code: str, message: str) -> None:
        context = self.task_executor.context
        payload = build_envelope(
            message_type="command.result",
            robot_id=self.config.robot.id,
            session_id=self.mqtt.session_id,
            payload={
                "command_id": command_id,
                "task_execution_id": context.task_execution_id if context else None,
                "status": status,
                "started_at": now_iso(),
                "finished_at": now_iso(),
                "error_code": code or None,
                "error_message": message or None,
                "result": result,
            },
        )
        self.store.save_command_result(command_id, payload)
        self.mqtt.publish_result(command_id, payload)

    def _publish_task_event(self, event_type: str, payload: dict, trace_id: str = "") -> None:
        context = self.task_executor.context
        trace_id = str(trace_id or payload.get("trace_id") or (context.trace_id if context else ""))
        event_payload = dict(payload or {})
        if trace_id:
            event_payload.setdefault("trace_id", trace_id)
        if event_type in {
            "task.pausing", "task.paused", "task.resuming", "task.resumed",
            "task.cancelling", "task.cancelled", "task.completed", "task.failed",
            "task.safe_hold",
        }:
            self._latest_task_state_event = {
                "task_execution_id": event_payload.get("task_execution_id"),
                "state": event_payload.get("state"),
                "reason_code": event_payload.get("reason_code"),
                "reason_message": event_payload.get("reason_message"),
            }
        if event_type in {"task.cancelled", "task.completed", "task.failed", "task.safe_hold"}:
            self._restore_temporary_fusion_profile(f"terminal_{event_type.replace('.', '_')}")
        coordinator = getattr(self, "self_healing", None)
        if coordinator is not None and event_type in {
            "task.progress", "task.completed", "task.failed", "task.safe_hold"
        }:
            snapshot = coordinator.snapshot()
            navigation_fault = snapshot.get("fault_label") in {
                "nav_stuck", "nav_action_failed", "collision_stop"
            }
            if snapshot and not snapshot.get("finished") and navigation_fault:
                succeeded = event_type in {"task.progress", "task.completed"}
                coordinator.finish_action(
                    str(snapshot.get("action_id") or ""),
                    success=succeeded,
                    reason=(
                        "navigation_progress_resumed"
                        if succeeded else str(
                            event_payload.get("reason_code") or event_type
                        )
                    ),
                )
                coordinator.complete(
                    success=succeeded,
                    reason=(
                        "navigation_progress_resumed"
                        if succeeded else str(
                            event_payload.get("reason_code") or event_type
                        )
                    ),
                )
        self.mqtt.publish_task_event(event_type, event_payload, trace_id)

    @staticmethod
    def _read_boot_id() -> str:
        try:
            return open("/proc/sys/kernel/random/boot_id", encoding="utf-8").read().strip()
        except OSError:
            return str(uuid.uuid4())

    def _current_map_payload(self) -> dict:
        return self.map_activation_adapter.status()

    def _persist_last_trusted_pose(self, pose) -> None:
        payload = {
            "x": float(pose.x),
            "y": float(pose.y),
            "z": float(pose.z),
            "yaw": float(pose.yaw),
            "sampled_at": pose.sampled_at,
            "source": "last_trusted_localization",
        }
        previous = self.store.load_last_trusted_pose(
            str(self.config.robot.current_map_id or ""),
            str(self.config.robot.current_map_version or ""),
        )
        max_drift = self._trusted_seed_max_drift_m()
        if previous and previous.get("x") is not None and previous.get("y") is not None:
            drift = planar_distance_m(previous, payload)
            if drift > max_drift:
                LOGGER.warning(
                    "skip persisting last_trusted pose: jump %.1fm exceeds %.1fm",
                    drift,
                    max_drift,
                )
                return
        self.store.save_last_trusted_pose(
            str(self.config.robot.current_map_id or ""),
            str(self.config.robot.current_map_version or ""),
            payload,
        )

    def _emit_localization_alert(self, event_type: str, severity: str, code: str, label: str, attributes: dict) -> None:
        """Publish a localization alert, tolerating a broken alert path.

        Alerting must never take down recovery, so every failure here is logged and
        swallowed - same contract as the mapping divergence alert above.
        """
        try:
            self.alerts.emit_system_alert(
                event_type,
                severity,
                code,
                attributes=attributes,
                detection={"label": label, "class": event_type, "confidence": 1},
                component="localization",
            )
        except Exception:
            LOGGER.exception("failed to publish %s alert", event_type)

    def _handle_imu_cross_check_report(self, report: dict) -> None:
        """Report a sustained disagreement between the lidar IMU and the 3588 IMU.

        Monitoring only - nothing here changes what localization consumes. The
        alert fires once per episode and re-arms once the two agree again,
        because a loose or failing IMU is a standing condition, not an event.
        Every report arrives here, including healthy ones, so that re-arming is
        driven by real evidence rather than by a timeout.
        """
        if report.get("status") == "ok":
            self._imu_mismatch_notified = False
            return
        if report.get("status") != "mismatch" or self._imu_mismatch_notified:
            # Absent or insufficient data says nothing about agreement, so it
            # neither raises an alert nor clears one.
            return
        self._imu_mismatch_notified = True
        stationary = bool(report.get("stationary"))
        self._emit_localization_alert(
            "imu_cross_check_mismatch",
            "medium",
            "IMU_CROSS_CHECK_MISMATCH",
            "双 IMU 静止零偏差异过大" if stationary else "双 IMU 角速度不一致",
            {"reason": "imu_cross_check_mismatch", **report},
        )

    def _localization_alert_attributes(self, reason: str) -> dict:
        """Collect the diagnostics the platform needs to triage a localization alert."""
        diagnostics_getter = getattr(self.navigation, "localization_diagnostics", None)
        diagnostics = diagnostics_getter() if callable(diagnostics_getter) else {}
        context = self.task_executor.context
        return {
            "reason": reason,
            "localization_quality": diagnostics.get("quality"),
            "localization_decision": diagnostics.get("decision"),
            "raw_pose": diagnostics.get("raw_pose"),
            "map_id": self.config.robot.current_map_id,
            "map_version": self.config.robot.current_map_version,
            "task_execution_id": context.task_execution_id if context else None,
        }

    def _report_localization_recovery_state(
        self, reason: str, cycle: int, elapsed: float, max_cycles: int
    ) -> None:
        """Publish recovery progress so a task stuck in `paused` is not invisible."""
        try:
            self.telemetry.on_localization_recovery({
                "reason": reason,
                "cycle": cycle,
                "max_cycles": max_cycles or None,
                "elapsed_seconds": round(elapsed, 1),
                "state": "recovering",
            })
        except Exception:
            LOGGER.exception("failed to report localization recovery state")

    def _self_healing_route_snapshot(self) -> dict:
        context = getattr(self.task_executor, "context", None)
        return dict(context.route_snapshot) if context else {}

    def _self_healing_evidence(self, supplied: dict | None = None) -> dict:
        evidence = dict(supplied or {})
        diagnostics_getter = getattr(self.navigation, "localization_diagnostics", None)
        diagnostics = diagnostics_getter() if callable(diagnostics_getter) else {}
        decision_getter = getattr(self.navigation, "localization_decision", None)
        decision = decision_getter() if callable(decision_getter) else {}
        obstacle_getter = getattr(self.navigation, "obstacle_monitor_snapshot", None)
        obstacle = obstacle_getter() if callable(obstacle_getter) else {}
        context = self.task_executor.context
        evidence.setdefault("localization_decision", decision if isinstance(decision, dict) else {})
        safety_state = getattr(self, "safety_state", None)
        evidence.setdefault("localization_status", getattr(safety_state, "localization_status", None))
        # RuntimeSafetyState stores the readable value. The diagnoser consumes
        # the wire value so both ROS and injected test adapters use one rule.
        if evidence.get("localization_status") == "normal":
            evidence["localization_status"] = 3
        elif evidence.get("localization_status") in {"lost", "degraded"}:
            evidence["localization_status"] = 4
        evidence.setdefault("obstacle", obstacle if isinstance(obstacle, dict) else {})
        evidence.setdefault("localization_quality", (diagnostics or {}).get("quality"))
        evidence.setdefault("task_execution_id", getattr(context, "task_execution_id", ""))
        robot_config = getattr(getattr(self, "config", None), "robot", None)
        evidence.setdefault("map_id", getattr(robot_config, "current_map_id", ""))
        return evidence

    def _diagnose_self_healing(
        self,
        fault_label: str,
        *,
        episode_id: str = "",
        level: int | None = None,
        evidence: dict | None = None,
    ):
        coordinator = getattr(self, "self_healing", None)
        if coordinator is None:
            safety = getattr(getattr(self, "config", None), "safety", None)
            diagnoser = FaultDiagnoser(
                ndt_failure_score=float(getattr(safety, "ndt_failure_score", 0.5))
            )
            return diagnoser.diagnose(
                episode_id=episode_id or "legacy-untracked",
                requested_fault=fault_label,
                route_snapshot=self._self_healing_route_snapshot(),
                evidence=self._self_healing_evidence(evidence),
                level=0 if level is None else level,
            )
        return coordinator.diagnose(
            fault_label,
            route_snapshot=self._self_healing_route_snapshot(),
            evidence=self._self_healing_evidence(evidence),
            episode_id=episode_id,
            level=level,
        )

    def _begin_self_heal_action(self, *, level: int, action_type: str) -> str:
        coordinator = getattr(self, "self_healing", None)
        return coordinator.begin_action(level=level, action_type=action_type) if coordinator else ""

    def _finish_self_heal_action(
        self, action_id: str, *, success: bool, reason: str = ""
    ) -> None:
        coordinator = getattr(self, "self_healing", None)
        if coordinator:
            coordinator.finish_action(action_id, success=success, reason=reason)

    def _complete_self_healing(self, *, success: bool, reason: str) -> None:
        coordinator = getattr(self, "self_healing", None)
        if coordinator:
            coordinator.complete(success=success, reason=reason)

    def _handle_navigation_self_healing(self, request: dict) -> dict:
        """Serve thin BT nodes while keeping diagnosis and level state in Edge."""
        operation = int(request.get("operation", 0))
        episode_id = str(request.get("episode_id") or "")
        action_id = str(request.get("action_id") or "")
        level = int(request.get("level", 0))
        fault_label = str(request.get("fault_label") or "navigation_failed")
        action_type = str(request.get("action_type") or "")
        detail = str(request.get("detail") or "")
        if getattr(self, "self_healing", None) is None:
            return {"accepted": False, "reason": "self_healing_coordinator_unavailable"}
        if operation in {2, 3, 4} and not episode_id:
            return {"accepted": False, "reason": "active self-healing episode_id is required"}
        before = self.self_healing.snapshot()
        if operation == 4:  # ACTION_FINISHED
            if (
                not before
                or before.get("finished")
                or before.get("episode_id") != episode_id
                or before.get("action_id") != action_id
            ):
                return {"accepted": False, "reason": "stale self-healing action completion"}
            self.self_healing.finish_action(
                action_id, success=bool(request.get("success")), reason=detail
            )
        try:
            diagnosis = self._diagnose_self_healing(
                fault_label,
                episode_id=episode_id,
                level=level,
                evidence=request.get("evidence"),
            )
        except SelfHealingPolicyError as exc:
            return {"accepted": False, "reason": str(exc)}
        if operation == 1 and before.get("action_id") and level > int(before.get("level", 0)):
            self.self_healing.finish_action(
                str(before["action_id"]),
                success=False,
                reason="advanced_to_next_recovery_level",
            )
        if operation == 3:  # ACTION_STARTED
            authorized, authorization_reason = self.self_healing.authorize_action(
                episode_id=diagnosis.episode_id,
                level=level,
                action_type=action_type,
            )
            if not authorized:
                result = vars(diagnosis).copy()
                result.update({"accepted": False, "reason": authorization_reason})
                return result
            active = self.self_healing.snapshot()
            if active.get("action_id"):
                self.self_healing.finish_action(
                    str(active["action_id"]),
                    success=False,
                    reason="replaced_by_concrete_bt_action",
                )
            action_id = self.self_healing.begin_action(level=level, action_type=action_type)
        if diagnosis.recovered:
            self.self_healing.finish_action(
                str(before.get("action_id") or action_id),
                success=True,
                reason=diagnosis.reason,
            )
        elif operation == 1:  # DIAGNOSE selects the logical BT action.
            current = self.self_healing.snapshot()
            if not current.get("action_id"):
                action_id = self.self_healing.begin_action(
                    level=diagnosis.level,
                    action_type=diagnosis.action_type,
                )
        result = vars(diagnosis).copy()
        result.update({"accepted": True, "action_id": action_id})
        return result

    def _fusion_profile_state_lock(self):
        """Return the lease lock, including for lightweight injected tests."""
        lock = getattr(self, "_fusion_profile_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._fusion_profile_lock = lock
            self._active_fusion_profile_generation = 0
        return lock

    def _activate_lio_hold(self, reason: str) -> dict:
        """Create one Edge-owned, generation-scoped LIO-hold lease."""
        navigation = getattr(self, "navigation", None)
        setter = getattr(navigation, "set_localization_fusion_profile", None)
        if not callable(setter):
            return {"accepted": False, "message": "service unavailable", "generation": 0}
        with self._fusion_profile_state_lock():
            expected = max(0, int(getattr(self, "_active_fusion_profile_generation", 0)))
            if expected <= 0:
                expected = self._reported_temporary_fusion_generation()
            try:
                result = setter(
                    "lio_hold",
                    reason=reason,
                    duration_seconds=180.0,
                    expected_generation=expected,
                )
            except Exception as exc:
                LOGGER.exception("failed to activate temporary LIO-hold profile")
                return {"accepted": False, "message": str(exc), "generation": expected}
            result = result if isinstance(result, dict) else {}
            generation = int(result.get("generation") or 0)
            if bool(result.get("accepted")) and generation > 0:
                self._active_fusion_profile_generation = generation
                return result
            if bool(result.get("accepted")):
                # A generation-less response cannot be released safely. Undo
                # it immediately instead of resuming a task with an orphaned
                # temporary profile.
                try:
                    setter("nominal", reason="lio_hold_generation_missing")
                except Exception:
                    LOGGER.exception("failed to undo generation-less LIO hold")
                return {
                    **result,
                    "accepted": False,
                    "message": "LIO hold response did not include a valid generation",
                }
            return result

    def _transition_temporary_fusion_profile(
        self,
        expected_generation: int,
        profile: str,
        *,
        reason: str,
        duration_seconds: float = 0.0,
    ) -> dict:
        """Change only the temporary fusion lease owned by this generation."""
        expected_generation = max(0, int(expected_generation))
        if expected_generation <= 0:
            return {"accepted": False, "message": "no owned temporary fusion generation"}
        navigation = getattr(self, "navigation", None)
        setter = getattr(navigation, "set_localization_fusion_profile", None)
        if not callable(setter):
            return {"accepted": False, "message": "service unavailable"}
        with self._fusion_profile_state_lock():
            active = max(0, int(getattr(self, "_active_fusion_profile_generation", 0)))
            if active != expected_generation:
                return {"accepted": False, "message": "temporary fusion generation is no longer active"}
            try:
                result = setter(
                    profile,
                    reason=reason,
                    duration_seconds=duration_seconds,
                    expected_generation=expected_generation,
                )
            except Exception as exc:
                LOGGER.exception("failed to transition temporary fusion profile to %s", profile)
                return {"accepted": False, "message": str(exc)}
            result = result if isinstance(result, dict) else {}
            returned_generation = int(result.get("generation") or 0)
            if bool(result.get("accepted")):
                if profile == "nominal":
                    self._active_fusion_profile_generation = 0
                elif returned_generation > 0:
                    self._active_fusion_profile_generation = returned_generation
                else:
                    self._active_fusion_profile_generation = 0
            elif returned_generation and returned_generation != expected_generation:
                # Another caller has already advanced the profile generation.
                # Forget this lease; never retry against the newer generation.
                self._active_fusion_profile_generation = 0
            return result

    def _reported_temporary_fusion_generation(self) -> int:
        decision_getter = getattr(getattr(self, "navigation", None), "localization_decision", None)
        try:
            decision = decision_getter() if callable(decision_getter) else {}
        except Exception:
            LOGGER.exception("failed to read localization fusion profile state")
            return 0
        if not isinstance(decision, dict):
            return 0
        if str(decision.get("fusion_profile") or "nominal").lower() == "nominal":
            return 0
        return max(0, int(decision.get("fusion_profile_generation") or 0))

    def _restore_temporary_fusion_profile(self, reason: str) -> bool:
        """Best-effort restore without ever overwriting a newer profile lease."""
        with self._fusion_profile_state_lock():
            generation = max(
                0,
                int(getattr(self, "_active_fusion_profile_generation", 0)),
            )
            if generation <= 0:
                generation = self._reported_temporary_fusion_generation()
                if generation <= 0:
                    return False
                self._active_fusion_profile_generation = generation
        result = self._transition_temporary_fusion_profile(
            generation,
            "nominal",
            reason=reason,
        )
        return bool(result.get("accepted"))

    def _forget_expired_fusion_generation(self, generation: int, wait_seconds: float) -> None:
        if self.stop_event.wait(max(0.0, wait_seconds)):
            return
        reported_generation = self._reported_temporary_fusion_generation()
        with self._fusion_profile_state_lock():
            if (
                int(getattr(self, "_active_fusion_profile_generation", 0)) == int(generation)
                and reported_generation != int(generation)
            ):
                self._active_fusion_profile_generation = 0

    def _handle_task_localization_recovered(
        self,
        restore_fusion: bool = True,
        recovery_reason: str = "localization_health_recovered",
    ) -> None:
        """Re-arm localization alerting, then resume the task as before."""
        self._localization_alert_notified = False
        self._complete_self_healing(success=True, reason=recovery_reason)
        if restore_fusion:
            try:
                self._restore_temporary_fusion_profile("self_healing_completed")
            except Exception:
                LOGGER.exception("failed to restore nominal localization fusion profile")
        try:
            self.telemetry.on_localization_recovery(None)
        except Exception:
            LOGGER.exception("failed to clear localization recovery state")
        self.task_executor.on_localization_recovered()

    def _restore_fusion_when_absolute_recovers(
        self,
        profile_generation: int,
        timeout_seconds: float = 180.0,
        required_stable_samples: int = 3,
    ) -> None:
        """Conditionally release this LIO-hold generation after stable recovery."""
        if profile_generation <= 0:
            return
        deadline = time.monotonic() + max(1.0, timeout_seconds)
        stable_samples = 0
        while not self.stop_event.is_set() and time.monotonic() < deadline:
            decision_getter = getattr(self.navigation, "localization_decision", None)
            decision = decision_getter() if callable(decision_getter) else {}
            if isinstance(decision, dict):
                ndt_healthy = decision.get("ndt_healthy") is True
                rtk_fixed = (
                    decision.get("rtk_good_for_navigation") is True
                    or (
                        decision.get("rtk_usable") is True
                        and str(decision.get("rtk_quality") or "").lower() == "fixed"
                    )
                )
                stable_samples = stable_samples + 1 if ndt_healthy or rtk_fixed else 0
                if stable_samples >= max(1, int(required_stable_samples)):
                    result = self._transition_temporary_fusion_profile(
                        profile_generation,
                        "balanced",
                        reason="absolute_observer_recovered_stable",
                        duration_seconds=5.0,
                    )
                    balanced_generation = int(result.get("generation") or 0)
                    if bool(result.get("accepted")) and balanced_generation > 0:
                        threading.Thread(
                            target=self._forget_expired_fusion_generation,
                            args=(balanced_generation, 6.0),
                            daemon=True,
                            name="localization-balanced-profile-expiry",
                        ).start()
                    return
            self.stop_event.wait(0.5)
        if not self.stop_event.is_set():
            self._transition_temporary_fusion_profile(
                profile_generation,
                "nominal",
                reason="edge_lio_hold_timeout",
            )

    def _handle_task_localization_loss(self, reason: str = "localization_lost") -> None:
        """Stop motion, then run the same bounded search as 主动重定位.

        `reason` distinguishes a hard localization loss from NDT score degradation;
        both funnel through the same recovery path but are reported separately.
        Indoor LIO can keep status=3 with a false map lock, so NDT degradation is
        treated as loss as well.
        """
        # Initial-pose and global-search commands intentionally pass through
        # non-normal states. Their own timeout/verification owns failure
        # reporting; treating that transition as a new loss starts an
        # automatic search that can supersede the operator command.
        if self._operator_localization_active():
            LOGGER.info(
                "localization transition ignored by auto recovery while an operator request is active"
            )
            return
        # NDT is a low-rate consistency observer, so losing only NDT while an
        # absolute RTK anchor remains healthy is warning-only. FAST-LIO loss
        # or motion anomaly is never masked by RTK: RTK is not a continuous
        # navigation source in the anchor architecture.
        if str(reason).startswith("ndt") and (
            self._rtk_good_for_navigation() or self._rtk_position_good_for_navigation()
        ):
            LOGGER.info(
                "NDT health transition %s ignored while RTK anchor is fixed",
                reason,
            )
            return
        self._diagnose_self_healing(reason, level=0)
        # Alert before the early returns below: localization degrading is worth
        # reporting even when no task is running and there is nothing to pause.
        if not self._localization_alert_notified:
            self._localization_alert_notified = True
            severity = "high" if reason == "localization_lost" else "medium"
            label = "定位丢失" if reason == "localization_lost" else "NDT 匹配退化"
            self._emit_localization_alert(
                reason,
                severity,
                reason.upper(),
                label,
                self._localization_alert_attributes(reason),
            )
        self.task_executor.on_localization_lost()
        if self._mapping_blocks_auto_relocalize():
            LOGGER.warning("localization lost during mapping; skipping auto relocalize")
            self._complete_self_healing(
                success=False, reason="mapping_active_auto_relocalization_suppressed"
            )
            return
        if not self._localization_recovery_lock.acquire(blocking=False):
            return
        lease = None
        acquire = getattr(self.task_executor, "acquire_recovery", None)
        if callable(acquire):
            lease = acquire("EDGE_LOCALIZATION", reason)
            if lease is None:
                LOGGER.warning(
                    "localization recovery skipped; recovery ownership unavailable: %s",
                    getattr(self.task_executor, "recovery_snapshot", lambda: {})(),
                )
                self._complete_self_healing(
                    success=False, reason="recovery_ownership_unavailable"
                )
                self._localization_recovery_lock.release()
                return
        threading.Thread(
            target=self._recover_task_localization,
            args=(reason, lease),
            daemon=True,
            name="task-localization-restart",
        ).start()

    def _mapping_blocks_auto_relocalize(self) -> bool:
        adapter = getattr(self, "mapping_adapter", None)
        if adapter is None:
            return False
        status = getattr(adapter, "status", None)
        if not callable(status):
            return False
        try:
            return bool(status().get("process_alive"))
        except Exception:
            LOGGER.exception("failed to inspect mapping status before auto relocalize")
            return False

    def _hold_motion_for_relocalize(self) -> None:
        cancel = getattr(self.navigation, "cancel_navigation", None)
        if callable(cancel):
            try:
                cancel()
            except Exception:
                LOGGER.warning("auto relocalize could not cancel Nav2")
        stop = getattr(self.navigation, "stop_motion", None)
        if callable(stop):
            try:
                stop()
            except Exception:
                LOGGER.exception("auto relocalize could not zero cmd_vel")

    def _operator_localization_active(self) -> bool:
        active = getattr(self.navigation, "operator_localization_active", None)
        return bool(callable(active) and active())

    def _trusted_seed_max_drift_m(self) -> float:
        return float(getattr(self.config.safety, "localization_trusted_seed_max_drift_m", 15.0))

    def _rtk_usable_for_recovery(self) -> bool:
        getter = getattr(self.navigation, "localization_decision", None)
        decision = getter() if callable(getter) else {}
        if not isinstance(decision, dict):
            return False
        return (
            decision.get("rtk_usable") is True
            and decision.get("rtk_heading_usable") is True
        )

    def _localization_recovery_seed(self) -> dict | None:
        latest_getter = getattr(self.navigation, "latest_pose", None)
        latest = latest_getter() if callable(latest_getter) else None
        memory_trusted = None
        trusted_getter = getattr(self.navigation, "latest_trusted_pose", None)
        if callable(trusted_getter):
            memory_trusted = trusted_getter()
        disk_trusted = self.store.load_last_trusted_pose(
            str(self.config.robot.current_map_id or ""),
            str(self.config.robot.current_map_version or ""),
        )
        waypoint_getter = getattr(self.task_executor, "current_localization_waypoint", None)
        waypoint = waypoint_getter() if callable(waypoint_getter) else None
        seed = select_recovery_seed(
            latest_pose=latest,
            memory_trusted=memory_trusted,
            disk_trusted=disk_trusted,
            waypoint=waypoint,
            max_drift_m=self._trusted_seed_max_drift_m(),
        )
        if seed and seed.get("source") == "current_waypoint":
            LOGGER.info(
                "localization seed fallback: current waypoint index=%s round=%s",
                seed.get("waypoint_index"),
                (waypoint or {}).get("round_number"),
            )
        return seed

    def _attempt_rtk_recovery(self) -> bool:
        if self._rtk_pose_is_driving():
            LOGGER.info("automatic recovery left the GPS pose in place; resuming the task")
            self._handle_task_localization_recovered()
            return True
        # LIO-primary outdoor mode: fixed RTK XY is already correcting the pose.
        # Do not force a dual-antenna reseeding cycle when heading is flickering.
        if self._rtk_position_good_for_navigation():
            getter = getattr(self.navigation, "localization_decision", None)
            decision = getter() if callable(getter) else {}
            source = str((decision or {}).get("active_source") or "")
            if source in {"lio_imu", "rtk_imu"}:
                LOGGER.info(
                    "automatic recovery left the outdoor LIO/RTK pose in place; resuming the task"
                )
                self._handle_task_localization_recovered()
                return True
        if self._recover_with_fixed_rtk():
            self._handle_task_localization_recovered()
            return True
        return False

    def _wait_for_rtk_recovery(self) -> bool:
        if not self._rtk_usable_for_recovery() or self._rtk_good_for_navigation():
            return False
        retry_seconds = float(
            getattr(self.config.safety, "localization_rtk_float_retry_seconds", 5.0)
        )
        deadline = time.monotonic() + max(0.0, retry_seconds)
        LOGGER.warning(
            "RTK is usable but not fixed; waiting up to %.1fs before NDT relocalization",
            retry_seconds,
        )
        while time.monotonic() < deadline:
            if self._rtk_good_for_navigation():
                try:
                    return self._attempt_rtk_recovery()
                except Exception as exc:
                    LOGGER.warning("RTK recovery after float flicker failed: %s", exc)
                    return False
            time.sleep(0.5)
        return False

    def _wait_for_post_handoff_resume(self, timeout_seconds: float | None = None) -> bool:
        """Resume after an NDT commit when LIO/RTK becomes usable a few seconds later.

        `RELOCALIZATION_HANDOFF_FAILED` means the best NDT pose was already written,
        but FAST-LIO did not report `absolute_stable` inside the handoff window.
        Outdoor dogs often settle shortly afterward (or regain fixed RTK XY). Waiting
        here keeps the existing pause/resume contract instead of abandoning self-heal.
        """
        if timeout_seconds is None:
            timeout_seconds = float(
                getattr(self.config.safety, "localization_handoff_settle_seconds", 8.0)
            )
        deadline = time.monotonic() + max(0.0, timeout_seconds)
        while time.monotonic() < deadline:
            if self._operator_localization_active():
                return False
            if not self.task_executor.is_paused_for_localization():
                return True
            if self._rtk_good_for_navigation() or self._rtk_position_good_for_navigation():
                try:
                    if self._attempt_rtk_recovery():
                        return True
                except Exception as exc:
                    LOGGER.warning("post-handoff RTK recovery failed: %s", exc)
                    return False
            getter = getattr(self.navigation, "localization_decision", None)
            decision = getter() if callable(getter) else {}
            if isinstance(decision, dict) and decision.get("absolute_stable") is True:
                LOGGER.info(
                    "localization became absolute-stable after NDT commit; resuming task"
                )
                self._handle_task_localization_recovered()
                return True
            time.sleep(0.5)
        return False

    def _localization_waypoint_seeds(self) -> list[dict]:
        """Return ordered, de-duplicated waypoint seeds around the pending point."""
        getter = getattr(self.task_executor, "current_localization_waypoint", None)
        current = getter() if callable(getter) else None
        if not current or current.get("waypoint_index") is None:
            return []
        points = self.task_executor.context.route_snapshot.get("waypoints", []) if self.task_executor.context else []
        index = int(current.get("waypoint_index", 0))
        indexes = [index, index - 1, index + 1, 0, len(points) - 1]
        seeds, seen = [], set()
        for candidate_index in indexes:
            if candidate_index < 0 or candidate_index >= len(points):
                continue
            point = dict(points[candidate_index])
            key = (round(float(point.get("x", 0)), 3), round(float(point.get("y", 0)), 3))
            if key in seen:
                continue
            seen.add(key)
            seeds.append({"x": float(point["x"]), "y": float(point["y"]), "z": float(point.get("z", 0.0) or 0.0), "yaw": float(point.get("yaw", 0.0) or 0.0), "source": "waypoint", "waypoint_index": candidate_index})
        return seeds

    def _rtk_good_for_navigation(self) -> bool:
        getter = getattr(self.navigation, "localization_decision", None)
        decision = getter() if callable(getter) else {}
        if not isinstance(decision, dict):
            return False
        if decision.get("rtk_good_for_navigation") is True:
            return True
        return (
            decision.get("rtk_usable") is True
            and str(decision.get("rtk_quality") or "").lower() == "fixed"
            and decision.get("rtk_heading_usable") is True
        )

    def _rtk_position_good_for_navigation(self) -> bool:
        getter = getattr(self.navigation, "localization_decision", None)
        decision = getter() if callable(getter) else {}
        if not isinstance(decision, dict):
            return False
        if decision.get("rtk_position_good_for_navigation") is True:
            return True
        return (
            decision.get("rtk_usable") is True
            and str(decision.get("rtk_quality") or "").lower() == "fixed"
        )

    def _rtk_pose_is_driving(self) -> bool:
        getter = getattr(self.navigation, "localization_decision", None)
        decision = getter() if callable(getter) else {}
        if not isinstance(decision, dict):
            return False
        return (
            str(decision.get("active_source") or "") == "rtk_imu"
            and self._rtk_good_for_navigation()
        )

    def _recover_with_fixed_rtk(self) -> bool:
        seed_rtk = getattr(self.navigation, "set_initial_pose_from_rtk", None)
        if not callable(seed_rtk):
            return False
        self._hold_motion_for_relocalize()
        seed_rtk()
        LOGGER.info("automatic recovery accepted a fixed RTK pose")
        return True

    def _recover_task_localization(self, reason: str = "localization_lost", lease=None) -> None:
        try:
            if self._operator_localization_active():
                LOGGER.info("automatic relocalization skipped while an operator request is active")
                return
            diagnosis = self._diagnose_self_healing(reason, level=0)
            level_zero_action = self._begin_self_heal_action(
                level=0, action_type=diagnosis.action_type
            )
            if diagnosis.wait_for_rtk:
                self._hold_motion_for_relocalize()
                if self._wait_for_rtk_recovery():
                    self._finish_self_heal_action(
                        level_zero_action, success=True, reason="rtk_recovered_during_wait"
                    )
                    return
                self._finish_self_heal_action(
                    level_zero_action, success=False, reason="rtk_wait_expired"
                )
                diagnosis = self._diagnose_self_healing(
                    diagnosis.fault_label,
                    episode_id=diagnosis.episode_id,
                    level=1,
                )
                if diagnosis.use_lio_hold:
                    profile_action = self._begin_self_heal_action(
                        level=1, action_type="set_ukf_lio_hold"
                    )
                    result = self._activate_lio_hold(diagnosis.fault_label)
                    accepted = bool(result.get("accepted"))
                    profile_generation = int(result.get("generation") or 0)
                    self._finish_self_heal_action(
                        profile_action,
                        success=accepted,
                        reason=str(result.get("message") or ""),
                    )
                    if accepted:
                        # RTK/NDT are auxiliary anchors. Healthy FAST-LIO may
                        # continue safely without waiting for either source to
                        # improve, and the temporary profile rejects bad anchor
                        # observations until health returns.
                        self._handle_task_localization_recovered(
                            restore_fusion=False,
                            recovery_reason="lio_hold_degraded_continuation",
                        )
                        threading.Thread(
                            target=self._restore_fusion_when_absolute_recovers,
                            args=(profile_generation,),
                            daemon=True,
                            name="localization-fusion-profile-restore",
                        ).start()
                        return
            else:
                self._finish_self_heal_action(
                    level_zero_action,
                    success=False,
                    reason="level_0_requires_active_relocalization",
                )
            cycle_retry = max(
                1.0,
                (
                    self.config.safety.arrival_precision_recovery_retry_seconds
                    if str(reason).startswith("arrival_precision")
                    else self.config.safety.localization_recovery_cycle_seconds
                ),
            )
            max_cycles = max(0, int(self.config.safety.localization_recovery_max_cycles))
            cycle = 0
            started_at = time.time()
            first_cycle = True
            while first_cycle or self.task_executor.is_paused_for_localization():
                first_cycle = False
                cycle += 1
                if self._rtk_good_for_navigation() or self._rtk_position_good_for_navigation():
                    try:
                        if self._attempt_rtk_recovery():
                            return
                    except Exception as exc:
                        LOGGER.warning("fixed RTK recovery failed: %s", exc)
                    if not self.task_executor.is_paused_for_localization():
                        return
                    elapsed = time.time() - started_at
                    self._report_localization_recovery_state(reason, cycle, elapsed, max_cycles)
                    if max_cycles and cycle >= max_cycles:
                        LOGGER.error(
                            "localization recovery gave up after %d cycles (%.0fs); escalating",
                            cycle,
                            elapsed,
                        )
                        self._emit_localization_alert(
                            "localization_recovery_failed",
                            "critical",
                            "LOCALIZATION_RECOVERY_FAILED",
                            "定位恢复失败，需人工介入",
                            {
                                **self._localization_alert_attributes(reason),
                                "recovery_cycles": cycle,
                                "recovery_elapsed_seconds": round(elapsed, 1),
                            },
                        )
                        self._complete_self_healing(
                            success=False, reason="fixed_rtk_recovery_levels_exhausted"
                        )
                        safe_hold = getattr(self.task_executor, "enter_safe_hold", None)
                        if callable(safe_hold):
                            safe_hold(
                                "LOCALIZATION_RECOVERY_EXHAUSTED",
                                "定位自愈等级已耗尽，进入安全保持",
                            )
                        return
                    LOGGER.warning(
                        "fixed RTK XY is available; skipping open-sky NDT search and retrying GPS in %.1fs",
                        cycle_retry,
                    )
                    time.sleep(cycle_retry)
                    continue
                if self._wait_for_rtk_recovery():
                    return
                primary_seed = self._localization_recovery_seed()
                waypoint_seeds = self._localization_waypoint_seeds()
                seeds = []
                seen_seeds = set()
                for seed in ([primary_seed] if primary_seed else []) + waypoint_seeds:
                    key = (
                        round(float(seed["x"]), 3),
                        round(float(seed["y"]), 3),
                        round(float(seed.get("yaw", 0.0)), 3),
                    )
                    if key in seen_seeds:
                        continue
                    seen_seeds.add(key)
                    seeds.append(seed)
                if not seeds:
                    LOGGER.error(
                        "localization recovery has no trusted pose; retrying in %.1fs",
                        cycle_retry,
                    )
                else:
                    if (
                        self.task_executor.has_active_task()
                        and not self.task_executor.is_paused_for_localization()
                    ):
                        return
                    relocalize = getattr(self.navigation, "active_relocalize", None)
                    for seed in seeds:
                        if self._operator_localization_active():
                            LOGGER.info(
                                "automatic relocalization stopped before it could preempt an operator request"
                            )
                            return
                        LOGGER.warning("localization lost; try waypoint seed index=%s x=%.3f y=%.3f yaw=%.3f", seed.get("waypoint_index"), seed["x"], seed["y"], seed["yaw"])
                        local_action = self._begin_self_heal_action(
                            level=1, action_type="local_relocalize"
                        )
                        try:
                            self._hold_motion_for_relocalize()
                            if not callable(relocalize):
                                raise RuntimeError("active_relocalize is unavailable")
                            relocalize({
                                **seed,
                                "max_attempts": 12,
                                "_automatic_recovery": True,
                            })
                            self._finish_self_heal_action(
                                local_action,
                                success=True,
                                reason="local_relocalization_accepted",
                            )
                            LOGGER.info("active relocalize accepted on cycle %d waypoint=%s", cycle, seed.get("waypoint_index"))
                            return
                        except Exception as exc:
                            self._finish_self_heal_action(
                                local_action,
                                success=False,
                                reason=str(getattr(exc, "code", "") or exc),
                            )
                            error_code = getattr(exc, "code", "")
                            if error_code == "RELOCALIZATION_SUPERSEDED":
                                LOGGER.info(
                                    "automatic relocalization was superseded by a newer localization request"
                                )
                                return
                            if error_code == "RELOCALIZATION_HANDOFF_FAILED":
                                # NDT already committed the map pose; LIO absolute
                                # handoff just did not settle in time. Do not exit
                                # the recovery worker — that permanently parks the
                                # paused task with no further self-heal attempts.
                                LOGGER.warning(
                                    "best NDT pose committed but LIO handoff failed; "
                                    "continuing self-heal instead of giving up"
                                )
                                if (
                                    self._rtk_usable_for_recovery()
                                    or self._rtk_good_for_navigation()
                                    or self._rtk_position_good_for_navigation()
                                ):
                                    LOGGER.warning(
                                        "NDT/LIO handoff failed; switching to RTK recovery"
                                    )
                                    try:
                                        if self._wait_for_rtk_recovery() or (
                                            (
                                                self._rtk_good_for_navigation()
                                                or self._rtk_position_good_for_navigation()
                                            )
                                            and self._attempt_rtk_recovery()
                                        ):
                                            return
                                    except Exception as rtk_exc:
                                        LOGGER.warning(
                                            "RTK recovery after NDT handoff failed: %s",
                                            rtk_exc,
                                        )
                                if self._wait_for_post_handoff_resume():
                                    return
                                break
                            LOGGER.warning("active relocalize cycle %d waypoint=%s failed: %s", cycle, seed.get("waypoint_index"), exc)
                    if cycle == 1:
                        level_two = self._diagnose_self_healing(
                            reason,
                            episode_id=diagnosis.episode_id,
                            level=2,
                        )
                        search = getattr(self.navigation, "search_laser_feature", None)
                        if level_two.search_laser and callable(search):
                            search_action = self._begin_self_heal_action(
                                level=2, action_type="search_laser_feature"
                            )
                            search_succeeded = False
                            try:
                                search_succeeded = bool(search(
                                    max_distance_m=0.15,
                                    timeout_seconds=4.0,
                                    allow_rotation=not level_two.forbid_spin,
                                ))
                            except Exception as exc:
                                LOGGER.warning("laser feature search failed: %s", exc)
                            self._finish_self_heal_action(
                                search_action,
                                success=search_succeeded,
                                reason=(
                                    "ndt_recovered_during_feature_search"
                                    if search_succeeded else "feature_search_exhausted"
                                ),
                            )
                            if search_succeeded:
                                self._handle_task_localization_recovered()
                                return
                        global_relocalize = getattr(self.navigation, "global_relocalize", None)
                        if callable(global_relocalize):
                            if self._operator_localization_active():
                                LOGGER.info(
                                    "automatic global relocalization skipped for an operator request"
                                )
                                return
                            try:
                                global_action = self._begin_self_heal_action(
                                    level=3, action_type="global_relocalize"
                                )
                                global_relocalize(wait_seconds=90.0, automatic=True)
                                self._finish_self_heal_action(
                                    global_action,
                                    success=True,
                                    reason="global_relocalization_accepted",
                                )
                                LOGGER.info("global relocalize accepted after waypoint seed failure")
                                return
                            except Exception as exc:
                                self._finish_self_heal_action(
                                    locals().get("global_action", ""),
                                    success=False,
                                    reason=str(getattr(exc, "code", "") or exc),
                                )
                                LOGGER.warning("global relocalize fallback failed: %s", exc)
                if not self.task_executor.is_paused_for_localization():
                    return
                elapsed = time.time() - started_at
                self._report_localization_recovery_state(reason, cycle, elapsed, max_cycles)
                if max_cycles and cycle >= max_cycles:
                    LOGGER.error(
                        "localization recovery gave up after %d cycles (%.0fs); escalating",
                        cycle,
                        elapsed,
                    )
                    self._emit_localization_alert(
                        "localization_recovery_failed",
                        "critical",
                        "LOCALIZATION_RECOVERY_FAILED",
                        "定位恢复失败，需人工介入",
                        {
                            **self._localization_alert_attributes(reason),
                            "recovery_cycles": cycle,
                            "recovery_elapsed_seconds": round(elapsed, 1),
                        },
                    )
                    self._complete_self_healing(
                        success=False, reason="localization_recovery_levels_exhausted"
                    )
                    safe_hold = getattr(self.task_executor, "enter_safe_hold", None)
                    if callable(safe_hold):
                        safe_hold(
                            "LOCALIZATION_RECOVERY_EXHAUSTED",
                            "定位自愈等级已耗尽，进入安全保持",
                        )
                    return
                LOGGER.warning(
                    "localization recovery cycle %d%s exhausted after %.0fs; "
                    "task remains stopped and will retry in %.1fs",
                    cycle,
                    "/%d" % max_cycles if max_cycles else " (unbounded)",
                    elapsed,
                    cycle_retry,
                )
                time.sleep(cycle_retry)
        except Exception:
            LOGGER.exception("task localization recovery worker failed; task remains paused")
        finally:
            # Whatever the outcome, no recovery is in flight once this worker exits.
            try:
                self.telemetry.on_localization_recovery(None)
            except Exception:
                LOGGER.exception("failed to clear localization recovery state")
            release = getattr(self.task_executor, "release_recovery", None)
            if callable(release):
                release(lease)
            self._localization_recovery_lock.release()


def main() -> None:
    parser = argparse.ArgumentParser(description="RoamerX Edge Agent")
    parser.add_argument("--config", default="config.yaml")
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )
    app = EdgeAgentApplication(EdgeConfig.load(args.config), config_path=args.config)

    def stop(*_args):
        app.stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        app.run_forever()
    finally:
        app.stop()
