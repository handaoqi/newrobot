import json
import struct
import tempfile
import zipfile
from io import BytesIO

from django.contrib.auth import get_user_model
from django.core.files.base import ContentFile
from django.test import override_settings
from rest_framework.test import APITestCase

from .models import MapData, MapNavigationBoundary


def pcd_bytes(points):
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
        "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA binary\n"
    ).encode()
    return header + b"".join(struct.pack("<ffff", *point) for point in points)


class MapSceneApiTests(APITestCase):
    def setUp(self):
        self.media = tempfile.TemporaryDirectory()
        self.override = override_settings(MEDIA_ROOT=self.media.name)
        self.override.enable()
        self.user = get_user_model().objects.create_user(username="scene-operator", password="unused")
        payload = BytesIO()
        with zipfile.ZipFile(payload, "w", zipfile.ZIP_STORED) as archive:
            archive.writestr("map.pcd", pcd_bytes([(1, 2, 0, 4), (2, 3, 1, 8), (3, 4, 2, 12)]))
        self.map = MapData.objects.create(
            name="scene map", width=100, height=80, resolution=0.1, origin=[-2, -3, 0],
            description=json.dumps({
                "package_files": ["map.pcd"],
                "scene_manifest": {"static_assets": [{"asset": "wall", "position": {"x": 1, "y": 2, "z": 0}}]},
            }),
        )
        self.map.package_file.save("scene.zip", ContentFile(payload.getvalue()), save=True)

    def tearDown(self):
        self.override.disable()
        self.media.cleanup()

    def test_scene_manifest_is_authenticated_and_read_only(self):
        MapNavigationBoundary.objects.create(
            map_data=self.map, outer_polygon=[[0, 0], [4, 0], [4, 3]],
            revision=2, active_revision=1,
            active_payload={"outer_polygon": [[0, 0], [3, 0], [3, 2]], "zones": []},
            apply_status="active",
        )
        unauthenticated = self.client.get(f"/api/maps/{self.map.id}/scene/")
        self.assertIn(unauthenticated.status_code, {401, 403})
        self.client.force_authenticate(self.user)
        response = self.client.get(f"/api/maps/{self.map.id}/scene/")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["schema"], "roamerx.scene-manifest.v1")
        self.assertEqual(response.data["bounds"]["max_x"], 8.0)
        self.assertTrue(response.data["cloud"]["available"])
        self.assertEqual(response.data["static_assets"][0]["asset"], "wall")
        self.assertEqual(response.data["boundary"]["points"][1], [3, 0])
        self.assertEqual(response.data["boundary"]["active_revision"], 1)

    def test_scene_cloud_is_sampled_cached_and_supports_range(self):
        self.client.force_authenticate(self.user)
        response = self.client.get(f"/api/maps/{self.map.id}/scene-cloud/")
        self.assertEqual(response.status_code, 200)
        body = b"".join(response.streaming_content)
        self.assertIn(b"POINTS 3\n", body[:300])
        self.assertEqual(response["X-Point-Count"], "3")
        ranged = self.client.get(f"/api/maps/{self.map.id}/scene-cloud/", HTTP_RANGE="bytes=0-9")
        self.assertEqual(ranged.status_code, 206)
        self.assertEqual(len(b"".join(ranged.streaming_content)), 10)
        self.assertTrue(ranged["Content-Range"].startswith("bytes 0-9/"))

    def test_manifest_read_does_not_create_boundary_configuration(self):
        self.client.force_authenticate(self.user)
        self.assertFalse(MapNavigationBoundary.objects.filter(map_data=self.map).exists())
        response = self.client.get(f"/api/maps/{self.map.id}/scene/")
        self.assertEqual(response.status_code, 200)
        self.assertFalse(MapNavigationBoundary.objects.filter(map_data=self.map).exists())

    def test_corrupt_package_returns_diagnostic_error(self):
        broken = MapData.objects.create(name="broken", description=json.dumps({"package_files": ["map.pcd"]}))
        broken.package_file.save("broken.zip", ContentFile(b"not-a-zip"), save=True)
        self.client.force_authenticate(self.user)
        response = self.client.get(f"/api/maps/{broken.id}/scene-cloud/")
        self.assertEqual(response.status_code, 422)
