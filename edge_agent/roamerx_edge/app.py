from __future__ import annotations

import argparse
import logging
import signal
import threading
import time
import uuid

from .alert_bridge import AlertBridge
from .command_processor import CommandProcessor
from .config import EdgeConfig
from .local_store import LocalStore
from .mapping_adapter import MappingAdapter
from .media_client import MediaClient
from .mqtt_client import EdgeMqttClient
from .protocol import build_envelope, now_iso
from .ros_adapter import ROS_AVAILABLE, RosAdapter, RosRuntime, rclpy
from .safety_policy import RuntimeSafetyState, SafetyPolicy
from .task_executor import TaskExecutor
from .telemetry_collector import TelemetryCollector
from .trajectory_buffer import TrajectoryBuffer

LOGGER = logging.getLogger(__name__)


class EdgeAgentApplication:
    def __init__(self, config: EdgeConfig, navigation=None) -> None:
        self.config = config
        self.store = LocalStore(config.storage.sqlite_path)
        self.stop_event = threading.Event()
        self.safety_state = RuntimeSafetyState(
            control_mode="autonomous",
            current_map_id=config.robot.current_map_id,
            current_map_version=config.robot.current_map_version,
        )
        self.telemetry = TelemetryCollector(config.robot, self.safety_state)
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
        self.task_executor = TaskExecutor(
            self.store,
            navigation,
            event_callback=self.mqtt.publish_task_event,
            start_result_callback=self._publish_start_result,
        )
        self.mapping_adapter = MappingAdapter(config.mapping, self.media_client)
        self.safety = SafetyPolicy(config.safety, self.safety_state)
        self.commands = CommandProcessor(
            robot_id=config.robot.id,
            store=self.store,
            safety=self.safety,
            task_executor=self.task_executor,
            publish_ack=self.mqtt.publish_ack,
            publish_result=self.mqtt.publish_result,
            mapping_adapter=self.mapping_adapter,
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
        if self.ros_runtime:
            self.ros_runtime.start()
            self.navigation.wait_until_ready(timeout_seconds=3.0)
        self.mqtt.connect()
        if not self.mqtt.wait_connected(15):
            LOGGER.warning("MQTT initial connection did not complete within 15 seconds")
        self._publish_online()
        self._publish_sync_request()
        self._threads = [
            threading.Thread(target=self._heartbeat_loop, daemon=True, name="heartbeat"),
            threading.Thread(target=self._status_loop, daemon=True, name="status"),
            threading.Thread(target=self._trajectory_loop, daemon=True, name="trajectory"),
            threading.Thread(target=self._outbox_loop, daemon=True, name="outbox"),
        ]
        for thread in self._threads:
            thread.start()

    def stop(self) -> None:
        self.stop_event.set()
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
                    "map.uploaded",
                    "telemetry.pose",
                    "trajectory.batch",
                    "alert.event",
                ],
                "current_map": {
                    "map_id": self.config.robot.current_map_id,
                    "map_version": self.config.robot.current_map_version,
                    "sha256": None,
                },
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
                },
            )

    def _status_loop(self) -> None:
        while not self.stop_event.wait(self.config.telemetry.status_interval_seconds):
            try:
                self.navigation.wait_until_ready(timeout_seconds=0.1)
            except Exception:
                LOGGER.exception("failed to refresh navigation readiness")
            context = self.task_executor.context
            self.mqtt.publish_status(
                self.telemetry.build_status_snapshot(
                    context.task_execution_id if context and self.task_executor.has_active_task() else None
                )
            )

    def _trajectory_loop(self) -> None:
        while not self.stop_event.wait(0.5):
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

    def _outbox_loop(self) -> None:
        while not self.stop_event.wait(2):
            self.mqtt.replay_outbox()

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


def main() -> None:
    parser = argparse.ArgumentParser(description="RoamerX Edge Agent")
    parser.add_argument("--config", default="config.yaml")
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
    )
    app = EdgeAgentApplication(EdgeConfig.load(args.config))

    def stop(*_args):
        app.stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        app.run_forever()
    finally:
        app.stop()
