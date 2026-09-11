from __future__ import annotations

import json
import logging
import ssl
import threading
import time
import uuid
from typing import Any

from django.conf import settings
from django.db import models
from django.utils import timezone

from .message_handlers import handle_mqtt_message
from .dev_message_handlers import handle_dev_mqtt_message
from .models import DevelopmentTask, RemoteCommand
from .protocol import CENTER_DOWNLINK_MESSAGE_TYPES, ProtocolError, build_command_message
from .services.command_service import CommandService

LOGGER = logging.getLogger(__name__)

try:
    import paho.mqtt.client as mqtt
except ImportError:  # pragma: no cover - reported clearly by management command
    mqtt = None


class PlatformMqttClient:
    def __init__(self) -> None:
        if mqtt is None:
            raise RuntimeError("paho-mqtt is required to run the device worker")
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=getattr(settings, "MQTT_CLIENT_ID", "roamerx-platform-worker"),
            protocol=mqtt.MQTTv5,
        )
        username = getattr(settings, "MQTT_USERNAME", "")
        if username:
            self.client.username_pw_set(username, getattr(settings, "MQTT_PASSWORD", ""))
        ca_file = getattr(settings, "MQTT_CA_FILE", "")
        if ca_file:
            self.client.tls_set(ca_certs=ca_file, tls_version=ssl.PROTOCOL_TLS_CLIENT)
        elif getattr(settings, "MQTT_TLS_ENABLED", True):
            self.client.tls_set(tls_version=ssl.PROTOCOL_TLS_CLIENT)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self._last_protocol_error_at: dict[str, float] = {}
        self._connected = threading.Event()

    def connect(self) -> None:
        self.client.connect(
            settings.MQTT_HOST,
            settings.MQTT_PORT,
            keepalive=getattr(settings, "MQTT_KEEPALIVE_SECONDS", 20),
        )

    def loop_forever(self) -> None:
        self.client.loop_forever(retry_first_connection=True)

    def disconnect(self) -> None:
        self.client.disconnect()

    def on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        if reason_code != 0:
            LOGGER.error("MQTT connection failed: %s", reason_code)
            return
        self._connected.set()
        for topic in (
            "robots/+/presence",
            "robots/+/telemetry/status",
            "robots/+/telemetry/pose",
            "robots/+/telemetry/trajectory",
            "robots/+/events/task",
            "robots/+/events/alert",
            "robots/+/commands/+/ack",
            "robots/+/commands/+/progress",
            "robots/+/commands/+/result",
            "robots/+/sync/state",
            "robots/+/dev/presence",
            "robots/+/dev/tasks/+/events",
            "robots/+/dev/tasks/+/result",
            "robots/+/dev/voice/audio",
        ):
            qos = 1 if "pose" not in topic and "status" not in topic else 0
            if topic.endswith("/sync/state"):
                client.subscribe(topic, options=mqtt.SubscribeOptions(qos=qos, noLocal=True))
            else:
                client.subscribe(topic, qos=qos)
        LOGGER.info("MQTT device worker connected")

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties) -> None:
        self._connected.clear()
        LOGGER.warning("MQTT device worker disconnected: %s", reason_code)

    def on_message(self, client, userdata, message) -> None:
        # sync/state is bidirectional. MQTT v5 No Local prevents the normal
        # self-loop, while this filter also protects against brokers that do
        # not honor it and messages published by another center instance.
        if self._is_center_sync_downlink(message.topic, message.payload):
            return
        try:
            if "/dev/" in message.topic:
                handle_dev_mqtt_message(message.topic, message.payload, self.publish_json)
            else:
                handle_mqtt_message(message.topic, message.payload, self.publish_json)
        except ProtocolError as exc:
            # A trajectory referencing a task that is absent from the center
            # can never succeed on replay.  Return a drop ACK so one stale
            # batch cannot hold the whole edge outbox at its head.
            if message.topic.endswith("/telemetry/trajectory") and exc.code in {
                "INVALID_MESSAGE",
                "UNKNOWN_TASK_EXECUTION",
            }:
                self._publish_trajectory_rejection(message.topic, message.payload, exc)
            # A malformed telemetry batch must not turn into thousands of tracebacks.
            now = time.monotonic()
            last_reported = self._last_protocol_error_at.get(message.topic, 0.0)
            if now - last_reported >= 60.0:
                LOGGER.warning("discarded invalid device message topic=%s error=%s", message.topic, exc)
                self._last_protocol_error_at[message.topic] = now
        except Exception:
            LOGGER.exception("failed to process device message topic=%s", message.topic)

    @staticmethod
    def _is_center_sync_downlink(topic: str, raw_payload: bytes | str) -> bool:
        if not topic.endswith("/sync/state"):
            return False
        try:
            if isinstance(raw_payload, bytes):
                raw_payload = raw_payload.decode("utf-8")
            payload = json.loads(raw_payload)
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError):
            return False
        return isinstance(payload, dict) and payload.get("message_type") in CENTER_DOWNLINK_MESSAGE_TYPES

    def _publish_trajectory_rejection(self, topic: str, raw_payload: bytes, error: ProtocolError) -> None:
        """Tell Edge to drop a permanently invalid trajectory batch."""
        try:
            envelope = json.loads(raw_payload.decode("utf-8"))
            payload = envelope.get("payload") or {}
            batch_id = payload.get("batch_id")
            if not batch_id:
                return
            robot_code = topic.split("/")[1]
            self.publish_json(
                f"robots/{robot_code}/sync/state",
                {
                    "protocol_version": "1.0",
                    "message_id": str(uuid.uuid4()),
                    "message_type": "trajectory.ack",
                    "robot_id": robot_code,
                    "session_id": "center",
                    "sent_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "trace_id": envelope.get("trace_id", ""),
                    "payload": {
                        "batch_id": batch_id,
                        "task_execution_id": payload.get("task_execution_id"),
                        "accepted": False,
                        "reason_code": error.code,
                        "reason_message": error.message,
                    },
                },
                qos=1,
            )
        except Exception:
            LOGGER.exception("failed to publish trajectory rejection topic=%s", topic)

    def publish_json(self, topic: str, payload: dict[str, Any], qos: int = 1, retain: bool = False) -> None:
        info = self.client.publish(
            topic,
            json.dumps(payload, ensure_ascii=False, default=str),
            qos=qos,
            retain=retain,
        )
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(f"MQTT publish failed rc={info.rc}")

    def publish_command(self, command: RemoteCommand) -> None:
        try:
            self.publish_json(
                f"robots/{command.robot.code}/commands",
                build_command_message(command),
                qos=1,
                retain=False,
            )
            CommandService.mark_published(command)
        except Exception as exc:
            CommandService.mark_publish_failed(command, str(exc))
            raise

    def publish_pending_commands(self) -> int:
        count = 0
        for command in RemoteCommand.objects.filter(status="created").select_related("robot", "task_execution"):
            self.publish_command(command)
            count += 1
        return count

    def publish_pending_development_tasks(self) -> int:
        now = timezone.now()
        retry_before = now - timezone.timedelta(seconds=5)
        tasks = DevelopmentTask.objects.select_related("robot").filter(
            models.Q(status="created") | models.Q(status="published", published_at__lt=retry_before)
        )[:10]
        count = 0
        for task in tasks:
            self.publish_json(
                f"robots/{task.robot.code}/dev/tasks",
                {
                    "task_id": str(task.id),
                    "workspace": task.workspace,
                    "model": task.model,
                    "prompt": task.prompt,
                    "conversation_id": "main",
                    "created_at": task.created_at,
                },
                qos=1,
                retain=False,
            )
            task.status = "published"
            task.published_at = now
            task.save(update_fields=["status", "published_at", "updated_at"])
            count += 1
        return count

    def publish_pending_development_controls(self) -> int:
        now = timezone.now()
        retry_before = now - timezone.timedelta(seconds=5)
        tasks = DevelopmentTask.objects.select_related("robot").filter(status="cancelling").filter(
            models.Q(cancel_published_at__isnull=True) | models.Q(cancel_published_at__lt=retry_before)
        )[:10]
        count = 0
        for task in tasks:
            self.publish_json(
                f"robots/{task.robot.code}/dev/tasks/{task.id}/control",
                {"task_id": str(task.id), "action": "cancel", "timestamp": now},
                qos=1,
                retain=False,
            )
            task.cancel_published_at = now
            task.save(update_fields=["cancel_published_at", "updated_at"])
            count += 1
        return count
