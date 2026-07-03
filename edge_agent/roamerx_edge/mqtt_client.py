from __future__ import annotations

import json
import logging
import ssl
import threading
import time
import uuid
from typing import Callable

import paho.mqtt.client as mqtt

from .config import EdgeConfig
from .local_store import LocalStore
from .protocol import build_envelope, encode_message

LOGGER = logging.getLogger(__name__)


class EdgeMqttClient:
    def __init__(self, config: EdgeConfig, store: LocalStore) -> None:
        self.config = config
        self.store = store
        self.session_id = str(uuid.uuid4())
        self._sequence = 0
        self._connected = threading.Event()
        self._command_handler: Callable | None = None
        self._sync_handler: Callable | None = None
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"roamerx-edge-{config.robot.id}",
            protocol=mqtt.MQTTv5,
        )
        if config.mqtt.username:
            self.client.username_pw_set(config.mqtt.username, config.mqtt.password)
        if config.mqtt.ca_file:
            self.client.tls_set(
                ca_certs=config.mqtt.ca_file,
                certfile=config.mqtt.cert_file or None,
                keyfile=config.mqtt.key_file or None,
                tls_version=ssl.PROTOCOL_TLS_CLIENT,
            )
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        will = build_envelope(
            message_type="presence.offline",
            robot_id=config.robot.id,
            session_id=self.session_id,
            sequence=0,
            payload={"reason": "unexpected_disconnect"},
        )
        self.client.will_set(self._topic("presence"), encode_message(will), qos=1, retain=True)

    def set_handlers(self, command_handler: Callable, sync_handler: Callable) -> None:
        self._command_handler = command_handler
        self._sync_handler = sync_handler

    def connect(self) -> None:
        properties = mqtt.Properties(mqtt.PacketTypes.CONNECT)
        properties.SessionExpiryInterval = self.config.mqtt.session_expiry_seconds
        self.client.connect(
            self.config.mqtt.host,
            self.config.mqtt.port,
            keepalive=self.config.mqtt.keepalive_seconds,
            clean_start=False,
            properties=properties,
        )
        self.client.loop_start()

    def disconnect(self) -> None:
        self.publish_presence("presence.offline", {"reason": "graceful_shutdown"}, retain=True)
        self.client.disconnect()
        self.client.loop_stop()

    def wait_connected(self, timeout: float = 10) -> bool:
        return self._connected.wait(timeout)

    def _on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        if reason_code != 0:
            LOGGER.error("MQTT connect failed: %s", reason_code)
            return
        self._connected.set()
        client.subscribe(self._topic("commands"), qos=1)
        client.subscribe(self._topic("sync/state"), qos=1)
        LOGGER.info("connected to center MQTT")

    def _on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties) -> None:
        self._connected.clear()
        LOGGER.warning("MQTT disconnected: %s", reason_code)

    def _on_message(self, client, userdata, message) -> None:
        try:
            payload = json.loads(message.payload.decode("utf-8"))
            LOGGER.info("MQTT msg topic=%s type=%s", message.topic, payload.get("message_type", "?"))
            if message.topic.endswith("/commands") and self._command_handler:
                self._command_handler(payload)
            elif message.topic.endswith("/sync/state") and self._sync_handler:
                self._sync_handler(payload)
        except Exception:
            LOGGER.exception("failed to handle center message topic=%s", message.topic)

    def publish_presence(self, message_type: str, payload: dict, retain: bool = False) -> None:
        self.publish(
            self._topic("presence"),
            build_envelope(
                message_type=message_type,
                robot_id=self.config.robot.id,
                session_id=self.session_id,
                sequence=self._next_sequence(),
                payload=payload,
            ),
            qos=1,
            retain=retain,
        )

    def publish_status(self, payload: dict) -> None:
        self.publish(
            self._topic("telemetry/status"),
            build_envelope(
                message_type="telemetry.status",
                robot_id=self.config.robot.id,
                session_id=self.session_id,
                sequence=self._next_sequence(),
                payload=payload,
            ),
            qos=0,
            retain=True,
        )

    def publish_task_event(self, event_type: str, payload: dict, trace_id: str = "") -> None:
        self.publish(
            self._topic("events/task"),
            build_envelope(
                message_type=event_type,
                robot_id=self.config.robot.id,
                session_id=self.session_id,
                trace_id=trace_id or None,
                sequence=self._next_sequence(),
                payload=payload,
            ),
            qos=1,
        )

    def publish_alert(self, payload: dict, trace_id: str = "") -> None:
        self.publish(
            self._topic("events/alert"),
            build_envelope(
                message_type="alert.event",
                robot_id=self.config.robot.id,
                session_id=self.session_id,
                trace_id=trace_id or None,
                sequence=self._next_sequence(),
                payload=payload,
            ),
            qos=1,
        )

    def publish_ack(self, command_id: str, payload: dict) -> None:
        self.publish(self._topic(f"commands/{command_id}/ack"), payload, qos=1)

    def publish_result(self, command_id: str, payload: dict) -> None:
        self.publish(self._topic(f"commands/{command_id}/result"), payload, qos=1)

    def publish(self, topic: str, payload: dict, *, qos: int, retain: bool = False) -> None:
        if not self._connected.is_set():
            self.store.enqueue_outbox(
                topic,
                payload,
                qos=qos,
                retain=retain,
                dedupe_key=payload.get("payload", {}).get("batch_id"),
            )
            return
        info = self.client.publish(topic, encode_message(payload), qos=qos, retain=retain)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            self.store.enqueue_outbox(topic, payload, qos=qos, retain=retain)

    def replay_outbox(self) -> int:
        sent = 0
        if not self._connected.is_set():
            return sent
        for row in self.store.list_pending_outbox():
            info = self.client.publish(
                row["topic"],
                encode_message(row["payload"]),
                qos=row["qos"],
                retain=row["retain"],
            )
            if info.rc == mqtt.MQTT_ERR_SUCCESS and row["payload"]["message_type"] != "trajectory.batch":
                self.store.ack_outbox(row_id=row["id"])
            sent += 1
        return sent

    def _topic(self, suffix: str) -> str:
        return f"robots/{self.config.robot.id}/{suffix}"

    def _next_sequence(self) -> int:
        self._sequence += 1
        return self._sequence
