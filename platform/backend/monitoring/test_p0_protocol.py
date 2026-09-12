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

    def test_accepts_continuous_loop_rosbag_and_stop_command(self):
        loop_session_id = "63b66a16-1947-4be7-889b-d851a5f4ba20"
        payload = json.loads(self.fixture_path.read_text())
        payload["payload"]["command"].update(
            {
                "record_rosbag": True,
                "continuous_rosbag": True,
                "loop_session_id": loop_session_id,
            }
        )
        self.assertEqual(parse_message(payload).message_type, "task.start")

        payload["message_type"] = "diagnostics.nav_rosbag_stop"
        payload["payload"].pop("task_execution_id", None)
        payload["payload"]["command"] = {"loop_session_id": loop_session_id}
        self.assertEqual(
            parse_message(payload).message_type,
            "diagnostics.nav_rosbag_stop",
        )

    def test_rejects_bad_protocol_version(self):
        payload = json.loads(self.fixture_path.read_text())
        payload["protocol_version"] = "2.0"
        with self.assertRaises(ProtocolError) as raised:
            parse_message(payload)
        self.assertEqual(raised.exception.code, "UNSUPPORTED_PROTOCOL_VERSION")

    def test_rejects_non_uuid_edge_session_before_persistence(self):
        payload = json.loads(self.fixture_path.read_text())
        payload["message_type"] = "sync.request"
        payload["payload"] = {}
        payload["session_id"] = "center"
        with self.assertRaises(ProtocolError) as raised:
            parse_message(payload)
        self.assertEqual(raised.exception.code, "INVALID_MESSAGE")
        self.assertEqual(raised.exception.message, "session_id must be UUID")

    def test_accepts_arrival_stage_events(self):
        for message_type in (
            "task.arrival_heading_aligning",
            "task.arrival_heading_aligned",
            "task.waypoint_postprocess_completed",
        ):
            payload = json.loads(self.fixture_path.read_text())
            payload["message_type"] = message_type
            payload["session_id"] = "63b66a16-1947-4be7-889b-d851a5f4ba20"
            payload["payload"] = {
                "task_execution_id": payload["payload"]["task_execution_id"],
                "state": "running",
                "state_version": 7,
            }
            self.assertEqual(parse_message(payload).message_type, message_type)

    def test_rejects_non_contiguous_waypoints(self):
        payload = json.loads(self.fixture_path.read_text())
        payload["payload"]["command"]["route_snapshot"]["waypoints"][1]["sequence"] = 8
        with self.assertRaises(ProtocolError):
            parse_message(payload)

    def test_rejects_invalid_waypoint_global_controller(self):
        payload = json.loads(self.fixture_path.read_text())
        payload["payload"]["command"]["route_snapshot"]["waypoints"][0]["global_controller"] = "invalid"
        with self.assertRaises(ProtocolError):
            parse_message(payload)

    def test_accepts_smac_hybrid_and_ilqr_waypoint(self):
        payload = json.loads(self.fixture_path.read_text())
        waypoint = payload["payload"]["command"]["route_snapshot"]["waypoints"][0]
        waypoint["global_controller"] = "smac_hybrid"
        waypoint["local_controller"] = "ilqr"
        self.assertEqual(parse_message(payload).message_type, "task.start")

    def test_rejects_nav2_micro_goal_for_pass_through_or_dock(self):
        for policy in ("pass_through", "dock"):
            payload = json.loads(self.fixture_path.read_text())
            waypoint = payload["payload"]["command"]["route_snapshot"]["waypoints"][0]
            waypoint["arrival_policy"] = policy
            waypoint["arrival_micro_adjust_mode"] = "nav2_goal"
            with self.assertRaises(ProtocolError) as raised:
                parse_message(payload)
            self.assertEqual(raised.exception.code, "INVALID_MESSAGE")

    def test_accepts_explicit_rtk_primary_waypoint_metadata(self):
        payload = json.loads(self.fixture_path.read_text())
        waypoint = payload["payload"]["command"]["route_snapshot"]["waypoints"][0]
        waypoint.update(
            {
                "localization_mode": "rtk",
                "localization_anchor_preference": "rtk",
                "rtk_primary_allowed": True,
            }
        )
        self.assertEqual(parse_message(payload).message_type, "task.start")

    def test_accepts_nav_recover_command(self):
        payload = json.loads(self.fixture_path.read_text())
        payload["message_type"] = "nav.recover"
        payload["payload"].pop("task_execution_id", None)
        payload["payload"]["command"] = {"reason": "test"}
        envelope = parse_message(payload)
        self.assertEqual(envelope.message_type, "nav.recover")

    def test_accepts_nav_single_goal_command(self):
        payload = json.loads(self.fixture_path.read_text())
        payload["message_type"] = "nav.single_goal"
        payload["payload"].pop("task_execution_id", None)
        payload["payload"]["command"] = {
            "x": 1.25,
            "y": -0.5,
            "yaw": 0.75,
            "global_controller": "navfn",
        }
        envelope = parse_message(payload)
        self.assertEqual(envelope.message_type, "nav.single_goal")

    def test_rejects_invalid_nav_single_goal_global_controller(self):
        payload = json.loads(self.fixture_path.read_text())
        payload["message_type"] = "nav.single_goal"
        payload["payload"].pop("task_execution_id", None)
        payload["payload"]["command"] = {
            "x": 1.25,
            "y": -0.5,
            "yaw": 0.75,
            "global_controller": "invalid",
        }
        with self.assertRaises(ProtocolError):
            parse_message(payload)

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
