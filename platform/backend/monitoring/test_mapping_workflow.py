from django.test import TestCase
from django.utils import timezone
from rest_framework.test import APIClient

from .models import RemoteCommand, Robot


class MappingWorkflowApiTests(TestCase):
    def setUp(self):
        self.client = APIClient()
        self.robot = Robot.objects.create(
            code="map-workflow-dog",
            name="Map Workflow Dog",
            location="anchor",
            area="anchor",
            connection_status="online",
            status="online",
            last_seen_at=timezone.now(),
        )

    def command_for(self, path, payload):
        response = self.client.post(f"/api/robots/{self.robot.id}/mapping/{path}/", payload, format="json")
        self.assertEqual(response.status_code, 202)
        return RemoteCommand.objects.get(id=response.data["command_id"])

    def test_outdoor_workflow_commands_keep_required_payload(self):
        origin = self.command_for(
            "origin/start",
            {"map_name": "园区 A", "scene_scope": "outdoor"},
        )
        self.assertEqual(origin.command_type, "mapping.origin_start")
        self.assertEqual(origin.payload["mapping_type"], "outdoor")

        prepare = self.command_for(
            "origin/start",
            {"map_name": "园区 A", "scene_scope": "outdoor", "prepare_only": True},
        )
        self.assertTrue(prepare.payload["prepare_only"])

        slam = self.command_for(
            "slam/start",
            {"map_name": "园区 A", "mapping_type": "outdoor", "record_rosbag": True},
        )
        self.assertEqual(slam.command_type, "mapping.slam_start")
        self.assertTrue(slam.payload["record_rosbag"])

        begin = self.command_for("begin", {"heading_check_confirmed": True})
        self.assertEqual(begin.command_type, "mapping.begin")
        self.assertTrue(begin.payload["heading_check_confirmed"])

    def test_slam_start_rejects_unknown_mapping_type(self):
        response = self.client.post(
            f"/api/robots/{self.robot.id}/mapping/slam/start/",
            {"mapping_type": "warehouse"},
            format="json",
        )
        self.assertEqual(response.status_code, 400)

    def test_origin_status_alias_returns_unified_mapping_snapshot(self):
        response = self.client.get(f"/api/robots/{self.robot.id}/mapping/origin/status/")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["robot_id"], self.robot.id)
        self.assertIn("mapping_state", response.data)
