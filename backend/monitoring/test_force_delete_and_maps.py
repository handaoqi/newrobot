import io
import json
import zipfile
from datetime import time as datetime_time

from django.contrib.auth import get_user_model
from django.core.files.uploadedfile import SimpleUploadedFile
from django.test import TestCase
from django.utils import timezone
from rest_framework.test import APIClient
from rest_framework.test import APIRequestFactory

from .models import (
    InspectionEvent,
    MapData,
    PatrolRoute,
    PatrolSchedule,
    PatrolTask,
    Robot,
    ScheduleRun,
    TaskExecution,
    Track,
    Zone,
)
from .views import _map_activation_payload


class ForceDeleteTests(TestCase):
    def setUp(self):
        self.user = get_user_model().objects.create_user("cleanup-user", password="secret")
        self.client = APIClient()
        self.client.force_authenticate(self.user)
        self.robot = Robot.objects.create(
            code="rx-force",
            name="RX Force",
            location="park",
            area="park",
            connection_status="online",
            status="online",
            last_seen_at=timezone.now(),
        )
        self.map = MapData.objects.create(name="old map", robot=self.robot, active=True)
        self.fallback_map = MapData.objects.create(name="fallback map", robot=self.robot, active=False)
        self.route = PatrolRoute.objects.create(
            name="route",
            map_data=self.map,
            robot=self.robot,
            waypoints=[{"frame_id": "map", "x": 1.0, "y": 2.0, "yaw": 0.0}],
        )
        now = timezone.now()
        self.task = PatrolTask.objects.create(
            name="task",
            robot=self.robot,
            route=self.route,
            route_name=self.route.name,
            scheduled_start=now,
            scheduled_end=now + timezone.timedelta(hours=1),
        )
        self.execution = TaskExecution.objects.create(
            task=self.task,
            robot=self.robot,
            route=self.route,
            map_data=self.map,
            route_snapshot={"waypoints": []},
            state="completed",
            total_waypoints=1,
        )
        self.schedule = PatrolSchedule.objects.create(
            name="schedule",
            robot=self.robot,
            task_template=self.task,
            route=self.route,
            map_data=self.map,
            time_of_day=datetime_time(9, 0),
        )
        ScheduleRun.objects.create(
            schedule=self.schedule,
            robot=self.robot,
            planned_start_at=now,
            task_execution=self.execution,
            status="created",
        )
        Track.objects.create(
            robot=self.robot,
            map_data=self.map,
            route=self.route,
            task=self.task,
            path=[],
            start_time=now,
        )
        Zone.objects.create(name="zone", map_data=self.map, polygon=[])
        InspectionEvent.objects.create(
            robot=self.robot,
            title="event",
            event_type="test",
            location="park",
            task_execution=self.execution,
            map_data=self.map,
        )

    def test_force_delete_map_removes_related_records_and_sets_fallback_active(self):
        response = self.client.delete(f"/api/maps/{self.map.id}/?force=true")

        self.assertEqual(response.status_code, 200)
        self.assertFalse(MapData.objects.filter(id=self.map.id).exists())
        self.assertFalse(PatrolRoute.objects.filter(id=self.route.id).exists())
        self.assertFalse(PatrolTask.objects.filter(id=self.task.id).exists())
        self.assertFalse(TaskExecution.objects.filter(id=self.execution.id).exists())
        self.assertFalse(PatrolSchedule.objects.filter(id=self.schedule.id).exists())
        self.assertEqual(Track.objects.count(), 0)
        self.assertEqual(Zone.objects.count(), 0)
        self.assertEqual(InspectionEvent.objects.count(), 0)
        self.fallback_map.refresh_from_db()
        self.assertTrue(self.fallback_map.active)

    def test_force_delete_task_removes_related_records(self):
        response = self.client.delete(f"/api/patrol-tasks/{self.task.id}/?force=true")

        self.assertEqual(response.status_code, 200)
        self.assertFalse(PatrolTask.objects.filter(id=self.task.id).exists())
        self.assertFalse(TaskExecution.objects.filter(id=self.execution.id).exists())
        self.assertFalse(PatrolSchedule.objects.filter(id=self.schedule.id).exists())
        self.assertEqual(Track.objects.count(), 0)
        self.assertEqual(InspectionEvent.objects.count(), 0)


class MapUploadMetadataTests(TestCase):
    def setUp(self):
        self.robot = Robot.objects.create(code="rx-upload", name="RX Upload", location="park", area="park")
        self.client = APIClient()

    def test_device_map_upload_reads_yaml_and_pgm_metadata(self):
        package = io.BytesIO()
        with zipfile.ZipFile(package, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.writestr("map.yaml", "image: map.pgm\nresolution: 0.08\norigin: [-3.5, 1.25, 0.0]\n")
            archive.writestr("map.pgm", b"P5\n# comment\n4 3\n255\n" + bytes([0] * 12))
        package.seek(0)

        response = self.client.post(
            "/api/device/maps/upload/",
            {
                "robot_code": self.robot.code,
                "map_name": "uploaded",
                "metadata": json.dumps({"map_version": "v1"}),
                "map_package": SimpleUploadedFile("map.zip", package.read(), content_type="application/zip"),
            },
            format="multipart",
        )

        self.assertEqual(response.status_code, 201)
        created = MapData.objects.get(name="uploaded")
        self.assertEqual(created.resolution, 0.08)
        self.assertEqual(created.origin, [-3.5, 1.25, 0.0])
        self.assertEqual(created.width, 4)
        self.assertEqual(created.height, 3)


class MapActivationPayloadTests(TestCase):
    def test_uses_uploaded_source_map_dir_before_original_image_path(self):
        robot = Robot.objects.create(code="rx-map-payload", name="RX Payload", location="park", area="park")
        map_data = MapData.objects.create(
            name="candidate",
            robot=robot,
            description=json.dumps(
                {
                    "source_map_dir": "/home/robot/.jszr/map/source/filter_variants/candidate",
                    "image": "/home/robot/.jszr/map/source/map.pgm",
                }
            ),
        )

        payload = _map_activation_payload(map_data, APIRequestFactory().post("/"))

        self.assertEqual(payload["local_map_dir"], "/home/robot/.jszr/map/source/filter_variants/candidate")
        self.assertEqual(payload["local_image_path"], "/home/robot/.jszr/map/source/filter_variants/candidate/map.pgm")
