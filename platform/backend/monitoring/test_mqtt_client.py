import json
from unittest.mock import Mock, patch

from django.test import SimpleTestCase

from .mqtt_client import PlatformMqttClient
from .protocol import ProtocolError


class PlatformMqttClientTests(SimpleTestCase):
    def test_invalid_trajectory_protocol_error_publishes_drop_ack(self):
        client = object.__new__(PlatformMqttClient)
        client.publish_json = Mock()
        raw = json.dumps(
            {
                "trace_id": "trace-1",
                "payload": {
                    "batch_id": "batch-1",
                    "task_execution_id": "execution-1",
                },
            }
        ).encode()

        client._publish_trajectory_rejection(
            "robots/rx-001/telemetry/trajectory",
            raw,
            ProtocolError("INVALID_MESSAGE", "trajectory seq must be contiguous"),
        )

        client.publish_json.assert_called_once()
        topic, message = client.publish_json.call_args.args[:2]
        assert topic == "robots/rx-001/sync/state"
        assert message["message_type"] == "trajectory.ack"
        assert message["payload"]["accepted"] is False
        assert message["payload"]["reason_code"] == "INVALID_MESSAGE"

    def test_on_message_drops_permanently_invalid_trajectory_batches(self):
        client = object.__new__(PlatformMqttClient)
        client._publish_trajectory_rejection = Mock()
        client._last_protocol_error_at = {}
        message = Mock()
        message.topic = "robots/rx-001/telemetry/trajectory"
        message.payload = b"{}"

        with patch("monitoring.mqtt_client.handle_mqtt_message", side_effect=ProtocolError("INVALID_MESSAGE", "bad")):
            client.on_message(None, None, message)
        client._publish_trajectory_rejection.assert_called_once()

        client._publish_trajectory_rejection.reset_mock()
        with patch(
            "monitoring.mqtt_client.handle_mqtt_message",
            side_effect=ProtocolError("UNKNOWN_TASK_EXECUTION", "missing"),
        ):
            client.on_message(None, None, message)
        client._publish_trajectory_rejection.assert_called_once()
