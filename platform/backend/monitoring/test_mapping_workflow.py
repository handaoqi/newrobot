import json
from datetime import timedelta

from django.test import TestCase
from django.utils import timezone
from rest_framework.test import APIClient
from types import SimpleNamespace

from .models import MapData, RemoteCommand, Robot, RobotStatusLatest
from .serializers import MapDataSerializer
from .views import RobotNavigationRelocalizeView


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

    def test_extract_global_origin_builds_edge_command_from_map_metadata(self):
        map_data = MapData.objects.create(
            name="室外原点地图",
            robot=self.robot,
            description=json.dumps({
                "gnss_origin_yaml": (
                    "alignment_locked: 1\n"
                    "origin_latitude: 39.9\n"
                    "origin_longitude: 116.4\n"
                    "origin_altitude: 42.0\n"
                    "heading_confirmed: true\n"
                    "confirmed_heading_deg: 93.2\n"
                ),
            }),
        )
        command = self.command_for("origin/extract-global", {"map_id": map_data.id})
        self.assertEqual(command.command_type, "mapping.origin_extract_global")
        self.assertEqual(command.payload["global_enu"]["origin_latitude"], 39.9)
        self.assertEqual(command.payload["global_enu"]["confirmed_heading_deg"], 93.2)

    def test_map_serializer_exposes_map_and_locked_rtk_origins(self):
        map_data = MapData.objects.create(
            name="双原点地图",
            robot=self.robot,
            origin=[-10.0, -20.0, 0.25],
            description=json.dumps({
                "gnss_origin_yaml": (
                    "alignment_locked: 1\n"
                    "origin_latitude: 39.9\n"
                    "origin_longitude: 116.4\n"
                    "origin_altitude: 42.0\n"
                    "map_offset_x: 1.25\n"
                    "map_offset_y: -0.75\n"
                    "enu_to_map_yaw: 0.1\n"
                ),
            }),
        )
        origins = MapDataSerializer(map_data).data["origin_display"]
        self.assertEqual(origins["map"], {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame_id": "map"})
        self.assertEqual(origins["occupancy_grid"]["x"], -10.0)
        self.assertEqual(origins["rtk_enu"]["latitude"], 39.9)
        self.assertEqual(origins["rtk_enu"]["map_x"], 1.25)

    def test_relocalize_payload_accepts_quick_then_global_metadata(self):
        payload = RobotNavigationRelocalizeView().build_payload(
            SimpleNamespace(data={
                "seed_source": "quick_then_global",
                "scene_scope": "outdoor",
                "coordinate_mode": "rtk_fixed",
                "x": 1.0,
                "y": 2.0,
                "yaw": 0.3,
            }),
            self.robot,
        )
        self.assertEqual(payload["seed_source"], "quick_then_global")
        self.assertEqual(payload["scene_scope"], "outdoor")
        self.assertEqual(payload["coordinate_mode"], "rtk_fixed")
        self.assertEqual(payload["wait_seconds"], 120.0)

    def test_origin_status_alias_returns_unified_mapping_snapshot(self):
        response = self.client.get(f"/api/robots/{self.robot.id}/mapping/origin/status/")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["robot_id"], self.robot.id)
        self.assertIn("mapping_state", response.data)

    def test_successful_upload_overrides_stale_live_saving_progress(self):
        uploaded = MapData.objects.create(name="室内完整地图", robot=self.robot, active=True)
        now = timezone.now()
        command = RemoteCommand.objects.create(
            robot=self.robot,
            command_type="mapping.save",
            status="succeeded",
            issued_at=now - timedelta(minutes=6),
            expires_at=now + timedelta(hours=1),
            finished_at=now,
            result_payload={
                "state": "saving",
                "upload_result": {"id": uploaded.id},
                "mapping_metrics": {"keyframe_count": 178},
                "post_save_validation": {
                    "state": "queued",
                    "map_dir": "/maps/20260830_120000_001",
                },
                "save_progress": {
                    "stage": "writing_pcd",
                    "progress_percent": 65,
                    "recoverable": True,
                },
            },
        )
        RobotStatusLatest.objects.create(
            robot=self.robot,
            sampled_at=now,
            raw_payload={
                "mapping": {
                    "state": "saving",
                    "process_alive": False,
                    "post_save_validation": {
                        "state": "passed",
                        "map_dir": "/maps/20260830_120000_001",
                        "result": {
                            "schema": "roamerx.post-save-localization-check.v1",
                            "state": "passed",
                            "accurate": True,
                            "pose": {"x": 1.25, "y": -2.5, "yaw_deg": 92.0},
                        },
                    },
                    "save_progress": {
                        "stage": "writing_pcd",
                        "progress_percent": 65,
                        "updated_at_unix": now.timestamp(),
                    },
                }
            },
        )

        response = self.client.get(f"/api/robots/{self.robot.id}/mapping/status/")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["command_id"], str(command.id))
        self.assertEqual(response.data["command_status"], "succeeded")
        self.assertEqual(response.data["mapping_state"], "exited")
        self.assertEqual(response.data["result"]["state"], "exited")
        self.assertEqual(response.data["result"]["save_progress"]["stage"], "completed")
        self.assertEqual(response.data["result"]["save_progress"]["progress_percent"], 100.0)
        self.assertEqual(response.data["result"]["post_save_validation"]["state"], "passed")
        self.assertTrue(response.data["result"]["post_save_validation"]["result"]["accurate"])
        self.assertEqual(response.data["result"]["post_save_validation"]["result"]["pose"]["x"], 1.25)

    def test_successful_save_does_not_merge_validation_from_another_map(self):
        uploaded = MapData.objects.create(name="本次地图", robot=self.robot, active=True)
        now = timezone.now()
        RemoteCommand.objects.create(
            robot=self.robot,
            command_type="mapping.save",
            status="succeeded",
            issued_at=now - timedelta(minutes=1),
            expires_at=now + timedelta(hours=1),
            finished_at=now,
            result_payload={
                "state": "exited",
                "upload_result": {"id": uploaded.id},
                "post_save_validation": {"state": "queued", "map_dir": "/maps/current"},
            },
        )
        RobotStatusLatest.objects.create(
            robot=self.robot,
            sampled_at=now,
            raw_payload={
                "mapping": {
                    "state": "idle",
                    "post_save_validation": {
                        "state": "passed",
                        "map_dir": "/maps/different",
                        "result": {"accurate": True},
                    },
                }
            },
        )

        response = self.client.get(f"/api/robots/{self.robot.id}/mapping/status/")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["result"]["post_save_validation"]["state"], "queued")
