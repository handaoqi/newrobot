from pathlib import Path
from urllib.parse import urlparse

from django.conf import settings
from rest_framework import serializers
from PIL import Image, ImageDraw

from .models import InspectionEvent, MediaAsset, PatrolTask, Robot


def _snapshot_path(snapshot_url: str) -> Path | None:
    if not snapshot_url:
        return None

    parsed = urlparse(snapshot_url)
    path = parsed.path or snapshot_url
    media_url = settings.MEDIA_URL if settings.MEDIA_URL.startswith("/") else f"/{settings.MEDIA_URL}"
    if not path.startswith(media_url):
        return None

    relative_path = path.removeprefix(media_url).lstrip("/")
    candidate = (settings.MEDIA_ROOT / relative_path).resolve()
    media_root = settings.MEDIA_ROOT.resolve()
    if media_root not in candidate.parents and candidate != media_root:
        return None
    return candidate if candidate.exists() else None


def _media_public_url(relative_path: str) -> str:
    media_url = settings.MEDIA_URL if settings.MEDIA_URL.startswith("/") else f"/{settings.MEDIA_URL}"
    return f"{media_url}{relative_path}"


def _pink_region_bbox(image: Image.Image) -> tuple[int, int, int, int] | None:
    rgb_image = image.convert("RGB")
    width, height = rgb_image.size
    xs: list[int] = []
    ys: list[int] = []

    for y in range(height):
        for x, (red, green, blue) in enumerate(rgb_image.crop((0, y, width, y + 1)).getdata()):
            if red > 125 and blue > 115 and green < 135 and red > green * 1.15 and blue > green * 1.05:
                xs.append(x)
                ys.append(y)

    if not xs:
        return None

    return min(xs), min(ys), max(xs), max(ys)


def _stored_bbox(event: InspectionEvent, image: Image.Image) -> tuple[int, int, int, int] | None:
    values = [event.bbox_x, event.bbox_y, event.bbox_width, event.bbox_height]
    if any(value is None for value in values):
        return None

    image_width, image_height = image.size
    frame_width = event.frame_width or image_width
    frame_height = event.frame_height or image_height
    scale_x = image_width / frame_width
    scale_y = image_height / frame_height
    left = round(event.bbox_x * scale_x)
    top = round(event.bbox_y * scale_y)
    right = round((event.bbox_x + event.bbox_width) * scale_x)
    bottom = round((event.bbox_y + event.bbox_height) * scale_y)
    return left, top, right, bottom


def _clamp_bbox(bbox: tuple[int, int, int, int], image: Image.Image) -> tuple[int, int, int, int]:
    width, height = image.size
    left, top, right, bottom = bbox
    padding = max(8, round(min(width, height) * 0.01))
    return (
        max(0, left - padding),
        max(0, top - padding),
        min(width - 1, right + padding),
        min(height - 1, bottom + padding),
    )


def build_annotated_snapshot(event: InspectionEvent) -> str:
    source_path = _snapshot_path(event.snapshot_url)
    if not source_path:
        return event.snapshot_url

    target_dir = settings.MEDIA_ROOT / "annotated-events"
    target_dir.mkdir(parents=True, exist_ok=True)
    target_path = target_dir / f"event-{event.id}.jpg"
    if target_path.exists() and target_path.stat().st_mtime >= source_path.stat().st_mtime:
        return _media_public_url(f"annotated-events/{target_path.name}")

    with Image.open(source_path) as image:
        image = image.convert("RGB")
        bbox = _pink_region_bbox(image) or _stored_bbox(event, image)
        if not bbox:
            return event.snapshot_url

        bbox = _clamp_bbox(bbox, image)
        draw = ImageDraw.Draw(image)
        line_width = max(6, round(min(image.size) * 0.008))
        for offset in range(line_width):
            draw.rectangle(
                (bbox[0] - offset, bbox[1] - offset, bbox[2] + offset, bbox[3] + offset),
                outline=(0, 244, 255),
            )
        image.save(target_path, quality=92)

    return _media_public_url(f"annotated-events/{target_path.name}")


class RobotSerializer(serializers.ModelSerializer):
    status_label = serializers.CharField(source="get_status_display", read_only=True)
    mode_label = serializers.CharField(source="get_mode_display", read_only=True)

    class Meta:
        model = Robot
        fields = [
            "id",
            "code",
            "name",
            "location",
            "area",
            "status",
            "status_label",
            "mode",
            "mode_label",
            "battery_level",
            "network_strength",
            "speaker_volume",
            "today_alerts",
            "current_task_name",
            "last_heartbeat_at",
            "camera_id",
            "stream_id",
            "play_urls",
        ]


class EventSerializer(serializers.ModelSerializer):
    robot_code = serializers.CharField(source="robot.code", read_only=True)
    robot_name = serializers.CharField(source="robot.name", read_only=True)
    risk_label = serializers.CharField(source="get_risk_level_display", read_only=True)
    status_label = serializers.CharField(source="get_status_display", read_only=True)
    annotated_snapshot_url = serializers.SerializerMethodField()

    class Meta:
        model = InspectionEvent
        fields = [
            "id",
            "title",
            "event_type",
            "location",
            "detected_at",
            "confidence",
            "risk_level",
            "risk_label",
            "status",
            "status_label",
            "snapshot_url",
            "annotated_snapshot_url",
            "description",
            "handling_notes",
            "robot_code",
            "robot_name",
            "camera_id",
            "stream_id",
            "object_class",
            "track_id",
            "bbox_x",
            "bbox_y",
            "bbox_width",
            "bbox_height",
            "frame_width",
            "frame_height",
            "raw_detection",
        ]

    def get_annotated_snapshot_url(self, obj):
        annotated_url = build_annotated_snapshot(obj)
        if annotated_url.startswith("http"):
            return annotated_url

        request = self.context.get("request")
        if request:
            return request.build_absolute_uri(annotated_url)

        parsed = urlparse(obj.snapshot_url)
        if parsed.scheme and parsed.netloc:
            return f"{parsed.scheme}://{parsed.netloc}{annotated_url}"
        return annotated_url


class PatrolTaskSerializer(serializers.ModelSerializer):
    robot_name = serializers.CharField(source="robot.name", read_only=True)

    class Meta:
        model = PatrolTask
        fields = [
            "id",
            "name",
            "robot_name",
            "route_name",
            "scheduled_start",
            "scheduled_end",
            "status",
            "completion_rate",
        ]


class RobotDetailSerializer(RobotSerializer):
    recent_events = serializers.SerializerMethodField()
    tasks = serializers.SerializerMethodField()

    class Meta(RobotSerializer.Meta):
        fields = RobotSerializer.Meta.fields + [
            "patrol_duration_minutes",
            "firmware_version",
            "recent_events",
            "tasks",
        ]

    def get_recent_events(self, obj):
        return EventSerializer(obj.events.all()[:5], many=True).data

    def get_tasks(self, obj):
        return PatrolTaskSerializer(obj.tasks.all()[:5], many=True).data


class PositionSerializer(serializers.Serializer):
    name = serializers.CharField(max_length=128)
    latitude = serializers.DecimalField(max_digits=10, decimal_places=6, required=False)
    longitude = serializers.DecimalField(max_digits=10, decimal_places=6, required=False)


class MotionSerializer(serializers.Serializer):
    speed = serializers.DecimalField(max_digits=6, decimal_places=2, required=False)
    heading = serializers.DecimalField(max_digits=6, decimal_places=2, required=False)


class PowerSerializer(serializers.Serializer):
    battery_level = serializers.IntegerField(min_value=0, max_value=100)
    charging = serializers.BooleanField(required=False, default=False)


class NetworkSerializer(serializers.Serializer):
    signal_strength = serializers.IntegerField(min_value=0, max_value=100)
    network_type = serializers.CharField(max_length=32, required=False, allow_blank=True)


class RuntimeSerializer(serializers.Serializer):
    mode = serializers.ChoiceField(choices=["auto", "manual", "standby", "returning"])
    status = serializers.ChoiceField(choices=["online", "offline", "warning", "charging"])


class TelemetryIngestSerializer(serializers.Serializer):
    sequence_id = serializers.CharField(max_length=64)
    robot_code = serializers.CharField(max_length=32)
    robot_name = serializers.CharField(max_length=64, required=False, allow_blank=True)
    reported_at = serializers.DateTimeField()
    position = PositionSerializer()
    motion = MotionSerializer(required=False, default=dict)
    power = PowerSerializer()
    network = NetworkSerializer()
    runtime = RuntimeSerializer()
    video = serializers.DictField(required=False, default=dict)
    detections = serializers.ListField(child=serializers.DictField(), required=False, default=list)


class MediaUploadSerializer(serializers.Serializer):
    robot_code = serializers.CharField(max_length=32)
    camera_id = serializers.CharField(max_length=32, required=False, allow_blank=True)
    media_type = serializers.ChoiceField(choices=["snapshot", "clip"])
    event_time = serializers.DateTimeField(required=False)
    sequence_id = serializers.CharField(max_length=64, required=False, allow_blank=True)
    sha256 = serializers.CharField(max_length=64, required=False, allow_blank=True)
    file = serializers.FileField()


class MediaAssetSerializer(serializers.ModelSerializer):
    robot_code = serializers.CharField(source="robot.code", read_only=True)

    class Meta:
        model = MediaAsset
        fields = [
            "id",
            "robot_code",
            "media_type",
            "camera_id",
            "sequence_id",
            "event_time",
            "url",
            "sha256",
            "file_size",
            "created_at",
        ]
