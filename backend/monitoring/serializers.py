from __future__ import annotations

from pathlib import Path
from urllib.parse import urlparse

from django.conf import settings
from django.utils import timezone
from rest_framework import serializers
from PIL import Image, ImageDraw

from .models import (
    CalendarDay,
    CommandEvent,
    InspectionEvent,
    MediaAsset,
    PatrolTask,
    PatrolSchedule,
    RemoteCommand,
    Robot,
    RobotCommand,
    RobotSession,
    RobotStatusLatest,
    TaskExecution,
    TaskExecutionEvent,
    TrajectoryPoint,
    MapData,
    MapSet,
    MapSetMember,
    PatrolRoute,
    Zone,
    Track,
    ScheduleRun,
)


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


def _absolute_media_url(path: str) -> str:
    public_base_url = getattr(settings, "PUBLIC_BASE_URL", "")
    if public_base_url:
        if path.startswith("http"):
            parsed = urlparse(path)
            path = parsed.path
        if not path.startswith("/"):
            path = f"/{path}"
        return f"{public_base_url}{path}"
    return path


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
        bbox = _stored_bbox(event, image)
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
    connection_status = serializers.SerializerMethodField()
    # 展示真实"当日"告警数（按 detected_at 当天计），而非永不清零的累计计数器字段。
    today_alerts = serializers.SerializerMethodField()

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
            "connection_status",
            "agent_version",
            "capabilities",
            "localization_status",
            "ros_ready",
            "nav_ready",
            "control_mode",
            "current_map_id",
            "current_map_version",
            "last_seen_at",
        ]

    def get_connection_status(self, obj):
        return obj.effective_connection_status()

    def get_today_alerts(self, obj):
        return InspectionEvent.objects.filter(
            robot=obj, detected_at__date=timezone.localdate()
        ).count()


class EventSerializer(serializers.ModelSerializer):
    robot_code = serializers.CharField(source="robot.code", read_only=True)
    robot_name = serializers.CharField(source="robot.name", read_only=True)
    risk_label = serializers.CharField(source="get_risk_level_display", read_only=True)
    status_label = serializers.CharField(source="get_status_display", read_only=True)
    review_result_label = serializers.CharField(source="get_review_result_display", read_only=True)
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
            "review_result",
            "review_result_label",
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
            "event_id",
            "task_execution",
            "map_id",
            "map_version",
            "frame_id",
            "position_x",
            "position_y",
            "position_yaw",
            "source_component",
            "source_code",
            "model_version",
            "snapshot_asset",
            "clip_asset",
            "handled_at",
        ]

    def get_annotated_snapshot_url(self, obj):
        annotated_url = build_annotated_snapshot(obj)
        request = self.context.get("request")
        if annotated_url.startswith("http"):
            parsed = urlparse(annotated_url)
            media_url = settings.MEDIA_URL if settings.MEDIA_URL.startswith("/") else f"/{settings.MEDIA_URL}"
            if parsed.path.startswith(media_url):
                return _absolute_media_url(parsed.path)
            if request and parsed.path.startswith(media_url):
                return request.build_absolute_uri(parsed.path)
            return annotated_url

        media_url = settings.MEDIA_URL if settings.MEDIA_URL.startswith("/") else f"/{settings.MEDIA_URL}"
        if annotated_url.startswith(media_url):
            return _absolute_media_url(annotated_url)

        if request:
            return request.build_absolute_uri(annotated_url)

        parsed = urlparse(obj.snapshot_url)
        if parsed.scheme and parsed.netloc:
            return f"{parsed.scheme}://{parsed.netloc}{annotated_url}"
        return annotated_url


class PatrolTaskSerializer(serializers.ModelSerializer):
    robot_name = serializers.CharField(source="robot.name", read_only=True)
    robot_code = serializers.CharField(source="robot.code", read_only=True)
    route_name_display = serializers.CharField(source="route.name", read_only=True, allow_null=True)
    map_id = serializers.IntegerField(source="route.map_data_id", read_only=True, allow_null=True)
    latest_execution = serializers.SerializerMethodField()

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
            "robot",
            "robot_code",
            "route",
            "route_name_display",
            "map_id",
            "enabled",
            "description",
            "latest_execution",
            "created_at",
            "updated_at",
        ]

    def get_latest_execution(self, obj):
        execution = obj.executions.order_by("-created_at").first()
        if not execution:
            return None
        return {
            "id": str(execution.id),
            "state": execution.state,
            "state_version": execution.state_version,
            "created_at": execution.created_at,
        }


class PatrolTaskCreateSerializer(serializers.ModelSerializer):
    scheduled_start = serializers.DateTimeField(required=False)
    scheduled_end = serializers.DateTimeField(required=False)

    class Meta:
        model = PatrolTask
        fields = ["name", "robot", "route", "enabled", "description", "scheduled_start", "scheduled_end"]

    def validate(self, attrs):
        robot = attrs.get("robot", getattr(self.instance, "robot", None))
        route = attrs.get("route", getattr(self.instance, "route", None))
        if route.robot_id != robot.id:
            raise serializers.ValidationError({"route": "路线必须属于所选机器人"})
        start = attrs.get("scheduled_start", getattr(self.instance, "scheduled_start", None))
        end = attrs.get("scheduled_end", getattr(self.instance, "scheduled_end", None))
        if start and end and end <= start:
            raise serializers.ValidationError({"scheduled_end": "结束时间必须晚于开始时间"})
        return attrs

    def create(self, validated_data):
        route = validated_data["route"]
        start = validated_data.setdefault("scheduled_start", timezone.now())
        validated_data.setdefault("scheduled_end", start + timezone.timedelta(hours=1))
        return PatrolTask.objects.create(
            route_name=route.name,
            status="pending",
            completion_rate=0,
            created_by=self.context["request"].user,
            **validated_data,
        )


class CalendarDaySerializer(serializers.ModelSerializer):
    day_type_label = serializers.CharField(source="get_day_type_display", read_only=True)

    class Meta:
        model = CalendarDay
        fields = ["id", "date", "name", "day_type", "day_type_label", "enabled", "note", "created_at", "updated_at"]


class PatrolScheduleSerializer(serializers.ModelSerializer):
    robot_name = serializers.CharField(source="robot.name", read_only=True)
    robot_code = serializers.CharField(source="robot.code", read_only=True)
    task_name = serializers.CharField(source="task_template.name", read_only=True)
    route_name = serializers.CharField(source="route.name", read_only=True)
    map_name = serializers.CharField(source="map_data.name", read_only=True)
    schedule_type_label = serializers.CharField(source="get_schedule_type_display", read_only=True)
    latest_run = serializers.SerializerMethodField()

    class Meta:
        model = PatrolSchedule
        fields = [
            "id",
            "name",
            "robot",
            "robot_name",
            "robot_code",
            "task_template",
            "task_name",
            "route",
            "route_name",
            "map_data",
            "map_name",
            "schedule_type",
            "schedule_type_label",
            "time_of_day",
            "weekdays",
            "run_date",
            "priority",
            "enabled",
            "last_triggered_at",
            "note",
            "latest_run",
            "created_at",
            "updated_at",
        ]

    def validate(self, attrs):
        robot = attrs.get("robot", getattr(self.instance, "robot", None))
        route = attrs.get("route", getattr(self.instance, "route", None))
        map_data = attrs.get("map_data", getattr(self.instance, "map_data", None))
        task = attrs.get("task_template", getattr(self.instance, "task_template", None))
        schedule_type = attrs.get("schedule_type", getattr(self.instance, "schedule_type", "daily"))
        run_date = attrs.get("run_date", getattr(self.instance, "run_date", None))
        weekdays = attrs.get("weekdays", getattr(self.instance, "weekdays", []))

        if route and robot and route.robot_id != robot.id:
            raise serializers.ValidationError({"route": "路线必须属于所选机器人"})
        if route:
            attrs["map_data"] = route.map_data
            map_data = route.map_data
        if route and map_data and route.map_data_id != map_data.id:
            raise serializers.ValidationError({"map_data": "地图必须与路线一致"})
        if task and robot and task.robot_id != robot.id:
            raise serializers.ValidationError({"task_template": "任务模板必须属于所选机器人"})
        if task and not task.enabled:
            raise serializers.ValidationError({"task_template": "停用的任务模板不能加入巡检日历"})
        if task and route and task.route_id and task.route_id != route.id:
            raise serializers.ValidationError({"route": "MVP 阶段计划路线必须与任务模板路线一致"})
        if schedule_type == "once" and not run_date:
            raise serializers.ValidationError({"run_date": "一次性计划必须选择执行日期"})
        if schedule_type != "once" and run_date:
            attrs["run_date"] = None
        if weekdays:
            if not isinstance(weekdays, list) or any(day not in [1, 2, 3, 4, 5, 6, 7] for day in weekdays):
                raise serializers.ValidationError({"weekdays": "weekdays 必须是 1-7 的数组"})
        return attrs

    def create(self, validated_data):
        request = self.context.get("request")
        if request and request.user and request.user.is_authenticated:
            validated_data["created_by"] = request.user
        return super().create(validated_data)

    def get_latest_run(self, obj):
        run = obj.runs.order_by("-planned_start_at").first()
        if not run:
            return None
        return ScheduleRunSerializer(run).data


class ScheduleRunSerializer(serializers.ModelSerializer):
    schedule_name = serializers.CharField(source="schedule.name", read_only=True)
    robot_name = serializers.CharField(source="robot.name", read_only=True)
    task_name = serializers.CharField(source="schedule.task_template.name", read_only=True)
    route_name = serializers.CharField(source="schedule.route.name", read_only=True)
    execution_state = serializers.CharField(source="task_execution.state", read_only=True, allow_null=True)

    class Meta:
        model = ScheduleRun
        fields = [
            "id",
            "schedule",
            "schedule_name",
            "robot",
            "robot_name",
            "task_name",
            "route_name",
            "planned_start_at",
            "triggered_at",
            "status",
            "task_execution",
            "execution_state",
            "remote_command",
            "skip_reason",
            "error_message",
            "created_at",
            "updated_at",
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
        return EventSerializer(obj.events.all()[:5], many=True, context=self.context).data

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
    event_id = serializers.UUIDField(required=False)
    task_execution_id = serializers.UUIDField(required=False)
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
            "media_id",
            "event_id",
            "task_execution",
            "content_type",
            "uploaded_at",
        ]


class RobotCommandCreateSerializer(serializers.Serializer):
    action = serializers.ChoiceField(choices=[choice[0] for choice in RobotCommand.ACTION_CHOICES])
    payload = serializers.DictField(required=False, default=dict)


class RobotCommandSerializer(serializers.ModelSerializer):
    robot_code = serializers.CharField(source="robot.code", read_only=True)
    action_label = serializers.CharField(source="get_action_display", read_only=True)
    status_label = serializers.CharField(source="get_status_display", read_only=True)

    class Meta:
        model = RobotCommand
        fields = [
            "id",
            "robot_code",
            "action",
            "action_label",
            "payload",
            "status",
            "status_label",
            "response_payload",
            "error_message",
            "sent_at",
            "created_at",
        ]


class MapDataSerializer(serializers.ModelSerializer):
    robot_name = serializers.CharField(source="robot.name", read_only=True, allow_null=True)
    robot_code = serializers.CharField(source="robot.code", read_only=True, allow_null=True)
    pgm_file = serializers.SerializerMethodField()
    yaml_file = serializers.SerializerMethodField()
    thumbnail = serializers.SerializerMethodField()
    pgm_url = serializers.SerializerMethodField()
    yaml_url = serializers.SerializerMethodField()
    thumbnail_url = serializers.SerializerMethodField()
    file_size = serializers.SerializerMethodField()

    class Meta:
        model = MapData
        fields = [
            "id",
            "name",
            "robot",
            "robot_name",
            "robot_code",
            "pgm_file",
            "yaml_file",
            "thumbnail",
            "pgm_url",
            "yaml_url",
            "thumbnail_url",
            "resolution",
            "width",
            "height",
            "origin",
            "active",
            "description",
            "file_size",
            "created_at",
            "updated_at",
        ]

    def get_pgm_file(self, obj):
        return obj.pgm_file.url if obj.pgm_file else None

    def get_yaml_file(self, obj):
        return obj.yaml_file.url if obj.yaml_file else None

    def get_thumbnail(self, obj):
        return obj.thumbnail.url if obj.thumbnail else None

    def get_pgm_url(self, obj):
        if obj.pgm_file:
            return obj.pgm_file.url
        return None

    def get_yaml_url(self, obj):
        if obj.yaml_file:
            return obj.yaml_file.url
        return None

    def get_thumbnail_url(self, obj):
        # 返回相对路径，前端 getFullUrl() 会自动拼接正确的 base URL
        return f"/api/maps/{obj.id}/preview/"

    def get_file_size(self, obj):
        size = 0
        if obj.pgm_file and hasattr(obj.pgm_file, 'size'):
            size += obj.pgm_file.size
        if obj.yaml_file and hasattr(obj.yaml_file, 'size'):
            size += obj.yaml_file.size
        return size


class MapSetMemberSerializer(serializers.ModelSerializer):
    map_data = MapDataSerializer(read_only=True)

    class Meta:
        model = MapSetMember
        fields = ["sequence", "submap_id", "metadata", "map_data"]


class MapSetSerializer(serializers.ModelSerializer):
    robot_name = serializers.CharField(source="robot.name", read_only=True, allow_null=True)
    members = MapSetMemberSerializer(many=True, read_only=True)

    class Meta:
        model = MapSet
        fields = ["id", "name", "robot", "robot_name", "version", "manifest", "active", "members", "created_at", "updated_at"]


class PatrolRouteSerializer(serializers.ModelSerializer):
    robot_name = serializers.CharField(source="robot.name", read_only=True)
    robot_code = serializers.CharField(source="robot.code", read_only=True)
    map_name = serializers.CharField(source="map_data.name", read_only=True)
    map_set_name = serializers.CharField(source="map_set.name", read_only=True, allow_null=True)

    class Meta:
        model = PatrolRoute
        fields = [
            "id",
            "name",
            "map_data",
            "map_name",
            "map_set",
            "map_set_name",
            "robot",
            "robot_name",
            "robot_code",
            "waypoints",
            "waypoint_names",
            "description",
            "created_at",
            "updated_at",
        ]


class ZoneSerializer(serializers.ModelSerializer):
    map_name = serializers.CharField(source="map_data.name", read_only=True)
    zone_type_label = serializers.CharField(source="get_zone_type_display", read_only=True)

    class Meta:
        model = Zone
        fields = [
            "id",
            "name",
            "map_data",
            "map_name",
            "zone_type",
            "zone_type_label",
            "polygon",
            "description",
            "active",
            "created_at",
            "updated_at",
        ]


class TrackSerializer(serializers.ModelSerializer):
    robot_name = serializers.CharField(source="robot.name", read_only=True)
    robot_code = serializers.CharField(source="robot.code", read_only=True)
    map_name = serializers.CharField(source="map_data.name", read_only=True, allow_null=True)
    route_name = serializers.CharField(source="route.name", read_only=True, allow_null=True)
    task_name = serializers.CharField(source="task.name", read_only=True, allow_null=True)

    class Meta:
        model = Track
        fields = [
            "id",
            "robot",
            "robot_name",
            "robot_code",
            "map_data",
            "map_name",
            "route",
            "route_name",
            "task",
            "task_name",
            "path",
            "start_time",
            "end_time",
            "distance",
            "duration",
            "description",
            "created_at",
            "updated_at",
        ]


class RobotSessionSerializer(serializers.ModelSerializer):
    class Meta:
        model = RobotSession
        fields = [
            "id",
            "session_id",
            "transport",
            "connected_at",
            "last_heartbeat_at",
            "disconnected_at",
            "disconnect_reason",
            "agent_version",
            "boot_id",
            "remote_ip",
            "capabilities",
            "metadata",
        ]


class RobotStatusSerializer(serializers.ModelSerializer):
    task_execution_id = serializers.UUIDField(source="task_execution.id", read_only=True, allow_null=True)
    localization_quality = serializers.SerializerMethodField()
    current_map = serializers.SerializerMethodField()

    def get_localization_quality(self, obj):
        raw_quality = (obj.raw_payload or {}).get("localization", {}).get("quality")
        return raw_quality or obj.localization_quality or {}

    def get_current_map(self, obj):
        return (obj.raw_payload or {}).get("current_map") or {
            "map_id": obj.map_id,
            "map_version": obj.map_version,
        }

    class Meta:
        model = RobotStatusLatest
        fields = [
            "state_version",
            "sampled_at",
            "received_at",
            "frame_id",
            "map_id",
            "map_version",
            "current_map",
            "x",
            "y",
            "z",
            "yaw",
            "speed_mps",
            "localization_status",
            "localization_source_status",
            "localization_quality",
            "power_available",
            "battery_percent",
            "charging",
            "network_type",
            "signal_percent",
            "ros_ready",
            "nav_ready",
            "emergency_stop",
            "control_mode",
            "task_execution_id",
        ]


class CommandEventSerializer(serializers.ModelSerializer):
    class Meta:
        model = CommandEvent
        fields = ["id", "event_type", "event_at", "source", "message_id", "payload"]


class RemoteCommandSerializer(serializers.ModelSerializer):
    events = CommandEventSerializer(many=True, read_only=True)

    class Meta:
        model = RemoteCommand
        fields = [
            "id",
            "command_type",
            "status",
            "trace_id",
            "issued_at",
            "expires_at",
            "published_at",
            "acknowledged_at",
            "started_at",
            "finished_at",
            "ack_reason_code",
            "ack_reason_message",
            "result_payload",
            "error_code",
            "error_message",
            "events",
        ]


class TaskExecutionEventSerializer(serializers.ModelSerializer):
    class Meta:
        model = TaskExecutionEvent
        fields = [
            "id",
            "state",
            "state_version",
            "event_type",
            "occurred_at",
            "received_at",
            "message_id",
            "reason_code",
            "reason_message",
            "payload",
        ]


class TaskExecutionSerializer(serializers.ModelSerializer):
    robot_name = serializers.CharField(source="robot.name", read_only=True)
    robot_code = serializers.CharField(source="robot.code", read_only=True)
    task_name = serializers.CharField(source="task.name", read_only=True)
    route_name = serializers.CharField(source="route.name", read_only=True, allow_null=True)
    map_name = serializers.CharField(source="map_data.name", read_only=True, allow_null=True)
    events = TaskExecutionEventSerializer(many=True, read_only=True)
    commands = RemoteCommandSerializer(many=True, read_only=True)

    class Meta:
        model = TaskExecution
        fields = [
            "id",
            "task",
            "task_name",
            "robot",
            "robot_name",
            "robot_code",
            "route",
            "route_name",
            "map_data",
            "map_name",
            "route_snapshot",
            "state",
            "state_version",
            "current_waypoint_index",
            "current_waypoint_id",
            "completed_waypoints",
            "total_waypoints",
            "distance_remaining_m",
            "estimated_time_remaining_s",
            "started_at",
            "finished_at",
            "accepted_at",
            "paused_at",
            "failure_code",
            "failure_message",
            "last_edge_event_at",
            "created_at",
            "updated_at",
            "events",
            "commands",
        ]


class TaskExecutionActionSerializer(serializers.Serializer):
    reason = serializers.CharField(required=False, default="operator_request", max_length=256)


class TrajectoryPointSerializer(serializers.ModelSerializer):
    class Meta:
        model = TrajectoryPoint
        fields = [
            "seq",
            "sampled_at",
            "received_at",
            "frame_id",
            "map_id",
            "map_version",
            "x",
            "y",
            "yaw",
            "speed_mps",
            "localization_status",
            "batch_id",
        ]


class AlertTimelineSerializer(serializers.Serializer):
    event_id = serializers.UUIDField()
    task_execution_id = serializers.UUIDField(allow_null=True)
    pose = serializers.DictField()
    trajectory = serializers.ListField(child=serializers.DictField())
    task_events = serializers.ListField(child=serializers.DictField())
    media = serializers.DictField()
