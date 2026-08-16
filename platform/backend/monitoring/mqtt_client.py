from __future__ import annotations

import json
import logging
import ssl
import threading
import time
from typing import Any

from django.conf import settings
from django.db import models
from django.utils import timezone

from .message_handlers import handle_mqtt_message
from .dev_message_handlers import handle_dev_mqtt_message
from .models import DevelopmentTask, RemoteCommand
from .protocol import ProtocolError, build_command_message
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
            "robots/+/commands/+/result",
            "robots/+/sync/state",
            "robots/+/dev/presence",
            "robots/+/dev/tasks/+/events",
            "robots/+/dev/tasks/+/result",
            "robots/+/dev/voice/audio",
        ):
            client.subscribe(topic, qos=1 if "pose" not in topic and "status" not in topic else 0)
        LOGGER.info("MQTT device worker connected")

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties) -> None:
        self._connected.clear()
        LOGGER.warning("MQTT device worker disconnected: %s", reason_code)

    def on_message(self, client, userdata, message) -> None:
        try:
            if "/dev/" in message.topic:
                handle_dev_mqtt_message(message.topic, message.payload, self.publish_json)
            else:
                handle_mqtt_message(message.topic, message.payload, self.publish_json)
        except ProtocolError as exc:
            # A malformed telemetry batch must not turn into thousands of tracebacks.
            now = time.monotonic()
            last_reported = self._last_protocol_error_at.get(message.topic, 0.0)
            if now - last_reported >= 60.0:
                LOGGER.warning("discarded invalid device message topic=%s error=%s", message.topic, exc)
                self._last_protocol_error_at[message.topic] = now
        except Exception:
            LOGGER.exception("failed to process device message topic=%s", message.topic)

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
