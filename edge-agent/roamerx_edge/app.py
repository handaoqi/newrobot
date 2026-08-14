from __future__ import annotations

import argparse
import logging
import signal
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
            navigation = RosAdapter(config.ros, config.safety, self.telemetry, self.safety_state)
            self.ros_runtime = RosRuntime(navigation)
        self.navigation = navigation
        self.map_activation_adapter = MapActivationAdapter(config, self.safety_state, config_path)
        self.navigation_stack_adapter = NavigationStackAdapter(config.navigation_stack)
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
            standup_confirmation_timeout_seconds=config.safety.standup_confirmation_timeout_seconds,
            map_set_coordinator=self.map_set_coordinator,
            obstacle_speech=config.obstacle_speech,
            rosbag_recorder=self.navigation_rosbag,
        )
        set_localization_failure_callback = getattr(
            navigation, "set_localization_failure_callback", None
        )
        if callable(set_localization_failure_callback):
            set_localization_failure_callback(self.task_executor.on_localization_lost)
        set_localization_recovery_callback = getattr(
            navigation, "set_localization_recovery_callback", None
        )
        if callable(set_localization_recovery_callback):
            set_localization_recovery_callback(self.task_executor.on_localization_recovered)
        set_trusted_pose_callback = getattr(navigation, "set_trusted_pose_callback", None)
        if callable(set_trusted_pose_callback):
            set_trusted_pose_callback(self._persist_last_trusted_pose)
        self.mapping_adapter = MappingAdapter(config.mapping, self.media_client)
        self.teleop_control_adapter = TeleopControlAdapter(config.teleop_control)
        self.sensor_control_adapter = SensorControlAdapter(config.sensor_control)
        self.power_mode_controller = PowerModeController(config.power_mode)
        self.charge_control_adapter = ChargeControlAdapter(config.charge_control, self.power_mode_controller)
        self.charge_control_adapter.set_low_battery_handler(self._handle_low_battery_charge)
        self._docking_undock_pending = False
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
            self.navigation.wait_until_ready(timeout_seconds=3.0)
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
        passive = getattr(self.navigation, "confirmed_remote_teleop_action", None)
        if callable(passive):
            passive("passive", {"passive"}, {"passive_failed"}, timeout_seconds=5.0)
        self._docking_undock_pending = True
        retries = max(1, int(docking.get("charge_retries", 3)))
        last_result = {}
        for attempt in range(retries):
            last_result = self.charge_control_adapter.start()
            if last_result.get("dock_ready") or last_result.get("charge_stage") not in {"waiting_for_dock", "idle"}:
                self.task_executor.report_docking_charge("task.docking_charge_started", message="充电条件通过，已发起充电", extra={"charge_stage": last_result.get("charge_stage"), "dock": last_result})
                LOGGER.info("docking charge accepted on attempt %d", attempt + 1)
                return
            time.sleep(2)
        self._docking_undock_pending = False
        self.task_executor.report_docking_charge("task.docking_charge_failed", message="充电条件未满足", extra={"dock": last_result})
        raise ProtocolError(
            "DOCK_CONTACT_NOT_READY",
            "充电桩蓝牙、极片或正负极未满足，已重试 3 次: " + str(last_result.get("missing") or "unknown"),
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
                    "sensor.restart",
                    "charge.start",
                    "charge.stop",
                    "motion.start",
                    "motion.stop",
                    "audio.volume",
                    "map.activate",
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
                if mapping_status.get("process_alive"):
                    self.navigation.safety_state.nav_ready = False
                else:
                    self.navigation.wait_until_ready(timeout_seconds=0.0)
                context = self.task_executor.context
                snapshot = self.telemetry.build_status_snapshot(
                    context.task_execution_id if context and self.task_executor.has_active_task() else None
                )
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

    def _handle_low_battery_charge(self) -> None:
        """Stop autonomous motion before waiting for an operator to dock the robot."""
        if not self.task_executor.has_active_task():
            return
        context = self.task_executor.context
        try:
            self.task_executor.cancel_task(context.task_execution_id if context else "")
            LOGGER.warning("low battery cancelled active navigation before charge preparation")
        except Exception:
            LOGGER.exception("graceful low-battery task cancellation failed; forcing local exit")
            try:
                self.task_executor.force_exit(context.task_execution_id if context else "")
            except Exception:
                LOGGER.exception("failed to force low-battery task exit")

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
