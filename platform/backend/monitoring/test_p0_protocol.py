import json
from pathlib import Path

from django.test import SimpleTestCase

from .protocol import ProtocolError, build_command_message, parse_message


class ProtocolContractTests(SimpleTestCase):
    fixture_path = Path(__file__).parent / "fixtures" / "task_start.json"

    def test_shared_task_start_fixture(self):
        envelope = parse_message(json.loads(self.fixture_path.read_text()))
        self.assertEqual(envelope.message_type, "task.start")
        self.assertEqual(len(envelope.payload["command"]["route_snapshot"]["waypoints"]), 3)

    def test_rejects_bad_protocol_version(self):
        payload = json.loads(self.fixture_path.read_text())
        payload["protocol_version"] = "2.0"
        with self.assertRaises(ProtocolError) as raised:
            parse_message(payload)
        self.assertEqual(raised.exception.code, "UNSUPPORTED_PROTOCOL_VERSION")

    def test_rejects_non_contiguous_waypoints(self):
        payload = json.loads(self.fixture_path.read_text())
        payload["payload"]["command"]["route_snapshot"]["waypoints"][1]["sequence"] = 8
        with self.assertRaises(ProtocolError):
            parse_message(payload)

    def test_accepts_nav_recover_command(self):
        payload = json.loads(self.fixture_path.read_text())
        payload["message_type"] = "nav.recover"
        payload["payload"].pop("task_execution_id", None)
        payload["payload"]["command"] = {"reason": "test"}
        envelope = parse_message(payload)
        self.assertEqual(envelope.message_type, "nav.recover")

    def test_accepts_mapping_origin_workflow_commands(self):
        for command_type in (
            "mapping.origin_start", "mapping.origin_cancel", "mapping.slam_start", "mapping.begin",
        ):
            payload = json.loads(self.fixture_path.read_text())
            payload["message_type"] = command_type
            payload["payload"].pop("task_execution_id", None)
            payload["payload"]["command"] = {"mapping_type": "outdoor"}
            self.assertEqual(parse_message(payload).message_type, command_type)

    def test_accepts_remote_trick_commands(self):
        for command_type in ("teleop.shake_hand", "teleop.two_leg_stand"):
            payload = json.loads(self.fixture_path.read_text())
            payload["message_type"] = command_type
            payload["payload"].pop("task_execution_id", None)
            payload["payload"]["command"] = {}
            self.assertEqual(parse_message(payload).message_type, command_type)

    def test_robot_command_omits_expected_state_without_task_execution(self):
        class Robot:
            code = "ZSL-1A-07"
            last_state_version = 113

        class Command:
            id = "5204ed65-d150-4fbf-8354-1ac5dfa01641"
            task_execution_id = None
            command_type = "nav.start"
            robot = Robot()
            trace_id = "5204ed65-d150-4fbf-8354-1ac5dfa01642"
            payload = {"reason": "test"}
            operator_id = None

        from django.utils import timezone

        command = Command()
        command.issued_at = timezone.now()
        command.expires_at = command.issued_at + timezone.timedelta(seconds=30)
        message = build_command_message(command)
        self.assertNotIn("expected_robot_state_version", message["payload"])
