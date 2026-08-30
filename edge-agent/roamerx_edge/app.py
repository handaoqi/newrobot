from __future__ import annotations

import argparse
import logging
import shutil
import signal
import subprocess
import threading
import time
import uuid

from .alert_bridge import AlertBridge
from .audio_control_adapter import AudioControlAdapter
from .charge_control_adapter import ChargeControlAdapter
from .command_processor import CommandProcessor
from .config import EdgeConfig
from .local_store import LocalStore
from .map_activation_adapter import MapActivationAdapter
from .map_set_coordinator import MapSetCoordinator
from .mapping_adapter import MappingAdapter
from .media_client import MediaClient
from .mqtt_client import EdgeMqttClient
from .navigation_stack_adapter import NavigationStackAdapter
from .protocol import ProtocolError, build_envelope, now_iso
from .power_mode_controller import PowerModeController
from .person_follow_controller import PersonFollowController
from .ros_adapter import ROS_AVAILABLE, RosAdapter, RosRuntime, rclpy
from .rosbag_recorder import RosbagRecorder
from .safety_policy import RuntimeSafetyState, SafetyPolicy
from .sensor_control_adapter import SensorControlAdapter
from .task_executor import TaskExecutor
from .telemetry_collector import TelemetryCollector
from .teleop_control_adapter import TeleopControlAdapter
from .trajectory_buffer import TrajectoryBuffer
from .system_telemetry import SystemTelemetryProbe

LOGGER = logging.getLogger(__name__)


class EdgeAgentApplication:
    def __init__(self, config: EdgeConfig, navigation=None, config_path: str = "config.yaml") -> None:
        self.config = config
        self.store = LocalStore(config.storage.sqlite_path)
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
            )
            self.ros_runtime = RosRuntime(navigation)
        self.navigation = navigation
        self.map_activation_adapter = MapActivationAdapter(config, self.safety_state, config_path)
        self.navigation_stack_adapter = NavigationStackAdapter(config.navigation_stack)
        self._localization_recovery_lock = threading.Lock()
        self.map_set_coordinator = MapSetCoordinator(self.map_activation_adapter, self.navigation_stack_adapter)
        self.navigation_rosbag = RosbagRecorder(
            config.navigation_stack.rosbag_script,
            config.navigation_stack.rosbag_stop_timeout_seconds,
        )
        self.task_executor = TaskExecutor(
            self.store,
            navigation,
            event_callback=self.mqtt.publish_task_event,
            start_result_callback=self._publish_start_result,
            final_waypoint_tolerance_m=config.safety.final_waypoint_tolerance_m,
            docking_goal_tolerance_m=config.safety.docking_goal_tolerance_m,
            docking_goal_yaw_tolerance_rad=config.safety.docking_goal_yaw_tolerance_rad,
            standup_confirmation_timeout_seconds=config.safety.standup_confirmation_timeout_seconds,
            map_set_coordinator=self.map_set_coordinator,
            obstacle_speech=config.obstacle_speech,
            waypoint_speech=config.waypoint_speech,
            rosbag_recorder=self.navigation_rosbag,
        )
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
        self.mapping_adapter = MappingAdapter(config.mapping, self.media_client)
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
        self.mqtt.connect()
        if not self.mqtt.wait_connected(15):
            LOGGER.warning("MQTT initial connection did not complete within 15 seconds")
        self.task_executor.report_startup_interruption()
        self.system_telemetry.poll()
        self.charge_control_adapter.observe_power(self.telemetry.latest_power())
        self.power_mode_controller.refresh_service_status(self.telemetry.latest_power())
        self._publish_online()
        self._publish_sync_request()
        self._threads = [
            threading.Thread(target=self._heartbeat_loop, daemon=True, name="heartbeat"),
            threading.Thread(target=self._status_loop, daemon=True, name="status"),
            threading.Thread(target=self._trajectory_loop, daemon=True, name="trajectory"),
            threading.Thread(target=self._outbox_loop, daemon=True, name="outbox"),
            threading.Thread(target=self._system_telemetry_loop, daemon=True, name="system-telemetry"),
        ]
        for thread in self._threads:
            thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.person_follow_controller.stop("edge_shutdown")
        self.task_executor.stop()
        for thread in self._threads:
            thread.join(timeout=3)
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
        payload = {
            "current_task_execution_id": context.task_execution_id if context else None,
            "local_task_state": context.state if context else None,
            "local_task_state_version": context.state_version if context else 0,
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
                snapshot = self.telemetry.build_status_snapshot(
                    context.task_execution_id if context and self.task_executor.has_active_task() else None
                )
                mapping_status["odometry"] = dict((snapshot.get("sensors") or {}).get("odometry") or {})
                snapshot["current_map"] = self._current_map_payload()
                snapshot["map_set"] = self.map_set_coordinator.status()
                snapshot["mapping"] = mapping_status
                obstacle_snapshot = getattr(self.navigation, "obstacle_monitor_snapshot", None)
                snapshot["navigation"] = obstacle_snapshot() if callable(obstacle_snapshot) else {}
                snapshot["power_mode"] = self.power_mode_controller.snapshot()
                self.mqtt.publish_status(snapshot)
            except Exception:
                LOGGER.exception("failed to publish telemetry status")

    def _trajectory_loop(self) -> None:
        while not self.stop_event.wait(0.5):
            try:
                context = self.task_executor.context
                pose = self.telemetry.latest_pose()
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

    def _outbox_loop(self) -> None:
        while not self.stop_event.wait(2):
            self.mqtt.replay_outbox()

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

    def _handle_mapping_divergence_event(self, event: dict) -> None:
        """Immediately immobilize the robot, then rescue only after SLAM flushed evidence."""
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
        docking_active = bool(
            self.task_executor.has_active_task()
            and context
            and (context.docking or {}).get("enabled")
        )
        action = "docking_already_active" if docking_active else "alert_only"
        # Always pass a retained non-docking context through the idempotent
        # cancel path. It may have become terminal after the snapshot above;
        # cancel_task then performs only the local zero-velocity safety action
        # and returns immediately without waiting for Nav2.
        if context and not docking_active:
            try:
                self.task_executor.cancel_task(execution_id)
                LOGGER.warning("low battery cancelled active navigation; automatic return is disabled")
            except Exception:
                LOGGER.exception("graceful low-battery task cancellation failed; forcing local exit")
                action = "alert_only_after_force_exit"
                try:
                    self.task_executor.force_exit(execution_id)
                except Exception:
                    action = "navigation_stop_failed_alert_only"
                    LOGGER.exception("failed to force low-battery task exit")
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

    @staticmethod
    def _read_boot_id() -> str:
        try:
            return open("/proc/sys/kernel/random/boot_id", encoding="utf-8").read().strip()
        except OSError:
            return str(uuid.uuid4())

    def _current_map_payload(self) -> dict:
        return self.map_activation_adapter.status()

    def _persist_last_trusted_pose(self, pose) -> None:
        self.store.save_last_trusted_pose(
            str(self.config.robot.current_map_id or ""),
            str(self.config.robot.current_map_version or ""),
            {
                "x": float(pose.x),
                "y": float(pose.y),
                "z": float(pose.z),
                "yaw": float(pose.yaw),
                "sampled_at": pose.sampled_at,
                "source": "last_trusted_localization",
            },
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

    def _handle_task_localization_recovered(self) -> None:
        """Re-arm localization alerting, then resume the task as before."""
        self._localization_alert_notified = False
        try:
            self.telemetry.on_localization_recovery(None)
        except Exception:
            LOGGER.exception("failed to clear localization recovery state")
        self.task_executor.on_localization_recovered()

    def _handle_task_localization_loss(self, reason: str = "localization_lost") -> None:
        """Stop motion, then run the same bounded search as 主动重定位.

        `reason` distinguishes a hard localization loss from NDT score degradation;
        both funnel through the same recovery path but are reported separately.
        Indoor LIO can keep status=3 with a false map lock, so NDT degradation is
        treated as loss as well.
        """
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
            return
        if not self._localization_recovery_lock.acquire(blocking=False):
            return
        threading.Thread(
            target=self._recover_task_localization,
            args=(reason,),
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

    def _localization_recovery_seed(self) -> dict | None:
        waypoint_getter = getattr(self.task_executor, "current_localization_waypoint", None)
        waypoint = waypoint_getter() if callable(waypoint_getter) else None
        if waypoint and waypoint.get("x") is not None and waypoint.get("y") is not None:
            LOGGER.info("localization seed: current waypoint index=%s round=%s", waypoint.get("waypoint_index"), waypoint.get("round_number"))
            return {"x": float(waypoint["x"]), "y": float(waypoint["y"]), "z": float(waypoint.get("z", 0.0) or 0.0), "yaw": float(waypoint.get("yaw", 0.0) or 0.0), "source": "current_waypoint"}
        pose = self.navigation.latest_trusted_pose()
        if not pose:
            pose = self.store.load_last_trusted_pose(
                str(self.config.robot.current_map_id or ""),
                str(self.config.robot.current_map_version or ""),
            )
        if pose:
            return dict(pose)
        latest_getter = getattr(self.navigation, "latest_pose", None)
        latest = latest_getter() if callable(latest_getter) else None
        if latest is None:
            return None
        return {
            "x": float(latest.x),
            "y": float(latest.y),
            "z": float(getattr(latest, "z", 0.0) or 0.0),
            "yaw": float(latest.yaw),
            "source": "last_trusted",
        }

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

    def _recover_task_localization(self, reason: str = "localization_lost") -> None:
        try:
            cycle_retry = max(1.0, self.config.safety.localization_recovery_cycle_seconds)
            max_cycles = max(0, int(self.config.safety.localization_recovery_max_cycles))
            cycle = 0
            started_at = time.time()
            first_cycle = True
            while first_cycle or self.task_executor.is_paused_for_localization():
                first_cycle = False
                cycle += 1
                waypoint_seeds = self._localization_waypoint_seeds()
                fallback_seed = None if waypoint_seeds else self._localization_recovery_seed()
                seeds = waypoint_seeds or ([fallback_seed] if fallback_seed else [])
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
                        LOGGER.warning("localization lost; try waypoint seed index=%s x=%.3f y=%.3f yaw=%.3f", seed.get("waypoint_index"), seed["x"], seed["y"], seed["yaw"])
                        try:
                            self._hold_motion_for_relocalize()
                            if not callable(relocalize):
                                raise RuntimeError("active_relocalize is unavailable")
                            relocalize({**seed, "max_attempts": 12})
                            LOGGER.info("active relocalize accepted on cycle %d waypoint=%s", cycle, seed.get("waypoint_index"))
                            return
                        except Exception as exc:
                            LOGGER.warning("active relocalize cycle %d waypoint=%s failed: %s", cycle, seed.get("waypoint_index"), exc)
                    if cycle == 1:
                        global_relocalize = getattr(self.navigation, "global_relocalize", None)
                        if callable(global_relocalize):
                            try:
                                global_relocalize(wait_seconds=90.0)
                                LOGGER.info("global relocalize accepted after waypoint seed failure")
                                return
                            except Exception as exc:
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
