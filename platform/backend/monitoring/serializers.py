from __future__ import annotations

import json
import math
import uuid
from pathlib import Path
from urllib.parse import urlparse

from django.conf import settings
from django.core import signing
from django.utils import timezone
from rest_framework import serializers
from PIL import Image, ImageDraw
import yaml

from .models import (
    AlertSkillBinding,
    CalendarDay,
    CommandEvent,
    DebugLogSession,
    InspectionEvent,
    MediaAsset,
    PatrolTask,
    PatrolLoopEvent,
    PatrolLoopSession,
    PatrolSchedule,
    RemoteCommand,
    Robot,
    RobotCommand,
    RecordedAudio,
    SpeechCategory,
    SpeechTemplate,
    RobotSession,
    RobotStatusLatest,
    TaskExecution,
    TaskExecutionEvent,
    TrajectoryPoint,
    MapData,
    MapNavigationBoundary,
    MapSet,
    MapSetMember,
    PatrolRoute,
    Zone,
    Track,
    ScheduleRun,
    SystemLog,
    GoldenBaseline,
    ValidationArtifact,
    ValidationAttempt,
    ValidationCheckResult,
    ValidationJob,
    ValidationProfile,
    ValidationRecording,
    ValidationRunner,
)

from .services.map_coordinate import MapConstraintError, constraints_from_map_data, validate_route_against_map
from .services.navigation_boundary_service import validate_waypoints_against_boundary
from .services.trajectory_distance_service import (
    calculate_loop_total_distance,
    stored_loop_total_distance,
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
    battery_level = serializers.SerializerMethodField()
    # 展示真实"当日"告警数（按 detected_at 当天计），而非永不清零的累计计数器字段。
    today_alerts = serializers.SerializerMethodField()
    charging_config = serializers.SerializerMethodField()

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
            "charging_config",
        ]

    def get_connection_status(self, obj):
        return obj.effective_connection_status()

    def get_battery_level(self, obj):
        """Prefer the latest valid BMS sample over the seeded robot profile value."""
        try:
            latest = obj.latest_status
        except RobotStatusLatest.DoesNotExist:
            latest = None
        if (
            latest is not None
            and latest.power_available
            and latest.battery_percent is not None
        ):
            return int(latest.battery_percent)
        return obj.battery_level

    def get_today_alerts(self, obj):
        if hasattr(obj, "today_alert_count"):
            return obj.today_alert_count
        return InspectionEvent.objects.filter(
            robot=obj, detected_at__date=timezone.localdate()
        ).count()

    def get_charging_config(self, obj):
        route = obj.charging_route
        map_data = obj.charging_map
        return {
            "map_id": map_data.id if map_data else None,
            "map_name": map_data.name if map_data else "",
            "route_id": route.id if route else None,
            "route_name": route.name if route else "",
            "configured": bool(map_data and route),
        }


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
    route_record_rosbag = serializers.BooleanField(
        source="route.record_rosbag", read_only=True, default=False
    )
    effective_record_rosbag = serializers.SerializerMethodField()

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
            "record_rosbag",
            "route_record_rosbag",
            "effective_record_rosbag",
            "description",
            "latest_execution",
            "created_at",
            "updated_at",
        ]

    def get_effective_record_rosbag(self, obj):
        return bool(
            (obj.route_id and obj.route and obj.route.record_rosbag)
            or obj.record_rosbag
        )

    def get_latest_execution(self, obj):
        execution = obj.executions.order_by("-created_at").first()
        if not execution:
            return None
        return {
            "id": str(execution.id),
            "state": execution.state,
            "state_version": execution.state_version,
            "failure_code": execution.failure_code,
            "failure_message": execution.failure_message,
            "loop_session_id": str(execution.loop_session_id) if execution.loop_session_id else None,
            "round_number": execution.round_number,
            "created_at": execution.created_at,
        }


class PatrolTaskCreateSerializer(serializers.ModelSerializer):
    scheduled_start = serializers.DateTimeField(required=False)
    scheduled_end = serializers.DateTimeField(required=False)

    class Meta:
        model = PatrolTask
        fields = [
            "name",
            "robot",
            "route",
            "enabled",
            "record_rosbag",
            "description",
            "scheduled_start",
            "scheduled_end",
        ]

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


class PersonDetectionIngestSerializer(serializers.Serializer):
    robot_code = serializers.CharField(max_length=32)
    camera_id = serializers.CharField(max_length=32, required=False, default="front")
    frame_width = serializers.IntegerField(min_value=1, max_value=16384)
    frame_height = serializers.IntegerField(min_value=1, max_value=16384)
    captured_at = serializers.DateTimeField()
    detections = serializers.ListField(child=serializers.DictField(), required=False, default=list)

    def validate_detections(self, detections):
        cleaned = []
        for item in detections[:20]:
            bbox = item.get("bbox") or {}
            try:
                x = max(0, int(bbox.get("x", 0)))
                y = max(0, int(bbox.get("y", 0)))
                width = max(0, int(bbox.get("width", 0)))
                height = max(0, int(bbox.get("height", 0)))
                confidence = min(1.0, max(0.0, float(item.get("confidence", 0))))
            except (TypeError, ValueError):
                continue
            track_id = str(item.get("track_id", ""))[:64]
            if not track_id or width <= 0 or height <= 0:
                continue
            label = str(item.get("label", "person")).strip().lower()
            if label not in {
                "person", "pedestrian", "bicycle", "bike", "自行车",
                "car", "truck", "bus", "motorcycle",
            }:
                continue
            cleaned.append(
                {
                    "track_id": track_id,
                    "label": label,
                    "confidence": round(confidence, 4),
                    "bbox": {"x": x, "y": y, "width": width, "height": height},
                }
            )
        return cleaned


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

    def validate(self, attrs):
        if attrs["action"] == "audio_volume":
            payload = attrs.get("payload") or {}
            if str(payload.get("target") or "") not in {"speaker_3588", "speaker_nx"}:
                raise serializers.ValidationError({"payload": "扬声器目标无效"})
            try:
                volume = int(payload.get("volume"))
            except (TypeError, ValueError):
                raise serializers.ValidationError({"payload": "音量必须是 0-100 的整数"})
            if not 0 <= volume <= 100:
                raise serializers.ValidationError({"payload": "音量必须在 0-100 之间"})
            attrs["payload"] = {**payload, "volume": volume}
            return attrs
        if attrs["action"] != "move_velocity":
            return attrs
        payload = attrs.get("payload") or {}
        cleaned = dict(payload)
        limits = (
            {"vx": 0.2, "vy": 0.15, "yaw_rate": 0.35}
            if payload.get("source") == "person_follow"
            else {"vx": 0.5, "vy": 0.5, "yaw_rate": 0.5}
        )
        for field, limit in limits.items():
            try:
                value = float(payload.get(field, 0.0))
            except (TypeError, ValueError):
                raise serializers.ValidationError({"payload": f"{field} 必须是数值"})
            cleaned[field] = max(-limit, min(limit, value))
        attrs["payload"] = cleaned
        return attrs


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


class SpeechCategorySerializer(serializers.ModelSerializer):
    template_count = serializers.IntegerField(source="templates.count", read_only=True)
    recording_count = serializers.IntegerField(source="recordings.count", read_only=True)

    class Meta:
        model = SpeechCategory
        fields = ["id", "name", "template_count", "recording_count", "created_at", "updated_at"]

    def validate_name(self, value):
        value = value.strip()
        if not value:
            raise serializers.ValidationError("分类名称不能为空")
        return value


class SpeechTemplateSerializer(serializers.ModelSerializer):
    category_name = serializers.CharField(source="category.name", read_only=True, default="未分类")
    source_type = serializers.SerializerMethodField()

    class Meta:
        model = SpeechTemplate
        fields = ["id", "name", "text", "category", "category_name", "source_type", "created_at", "updated_at"]

    def get_source_type(self, obj):
        return "tts"

    def validate_name(self, value):
        value = value.strip()
        if not value:
            raise serializers.ValidationError("文案名称不能为空")
        return value

    def validate_text(self, value):
        value = value.strip()
        if not value:
            raise serializers.ValidationError("播报文字不能为空")
        return value


class AlertSkillBindingSerializer(serializers.ModelSerializer):
    display_name = serializers.CharField(source="get_skill_key_display", read_only=True)
    template = SpeechTemplateSerializer(read_only=True)
    template_id = serializers.PrimaryKeyRelatedField(
        source="template",
        queryset=SpeechTemplate.objects.all(),
        write_only=True,
    )

    class Meta:
        model = AlertSkillBinding
        fields = ["skill_key", "display_name", "enabled", "template", "template_id", "updated_at"]
        read_only_fields = ["skill_key", "display_name", "updated_at"]


class AlertSkillPreviewSerializer(serializers.Serializer):
    robot_id = serializers.IntegerField(min_value=1)


class SpeechSynthesisSerializer(serializers.Serializer):
    text = serializers.CharField(max_length=500, trim_whitespace=True)
    audio_name = serializers.CharField(max_length=64, required=False, allow_blank=True, trim_whitespace=True)


class RecordedAudioSerializer(serializers.ModelSerializer):
    category_name = serializers.CharField(source="category.name", read_only=True, default="未分类")
    audio_url = serializers.SerializerMethodField()
    source_type = serializers.SerializerMethodField()

    class Meta:
        model = RecordedAudio
        fields = [
            "id",
            "title",
            "category",
            "category_name",
            "audio_url",
            "transcript",
            "asr_status",
            "asr_error",
            "source_type",
            "content_type",
            "file_size",
            "created_at",
        ]

    def get_source_type(self, obj):
        return "recording"

    def get_audio_url(self, obj):
        if not obj.file:
            return ""
        url = obj.file.url
        public_base_url = str(getattr(settings, "PUBLIC_BASE_URL", "") or "").rstrip("/")
        if public_base_url:
            return f"{public_base_url}{url}"
        request = self.context.get("request")
        return request.build_absolute_uri(url) if request else url


class MapDataSerializer(serializers.ModelSerializer):
    robot_name = serializers.CharField(source="robot.name", read_only=True, allow_null=True)
    robot_code = serializers.CharField(source="robot.code", read_only=True, allow_null=True)
    pgm_file = serializers.SerializerMethodField()
    yaml_file = serializers.SerializerMethodField()
    thumbnail = serializers.SerializerMethodField()
    pgm_url = serializers.SerializerMethodField()
    yaml_url = serializers.SerializerMethodField()
    thumbnail_url = serializers.SerializerMethodField()
    mapping_trace_url = serializers.SerializerMethodField()
    package_url = serializers.SerializerMethodField()
    file_size = serializers.SerializerMethodField()
    optimization_summary = serializers.SerializerMethodField()
    origin_display = serializers.SerializerMethodField()

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
            "mapping_trace_url",
            "package_url",
            "resolution",
            "width",
            "height",
            "origin",
            "active",
            "description",
            "parent_map",
            "edit_metadata",
            "coordinate_mode",
            "scene_scope",
            "localization_mode",
            "origin_status",
            "map_completeness",
            "mapping_metrics",
            "optimization_summary",
            "origin_display",
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

    def get_mapping_trace_url(self, obj):
        return f"/api/maps/{obj.id}/mapping-trace/"

    def get_package_url(self, obj):
        return obj.package_file.url if obj.package_file else None

    def get_file_size(self, obj):
        size = 0
        if obj.pgm_file and hasattr(obj.pgm_file, 'size'):
            size += obj.pgm_file.size
        if obj.yaml_file and hasattr(obj.yaml_file, 'size'):
            size += obj.yaml_file.size
        if obj.package_file and hasattr(obj.package_file, 'size'):
            size += obj.package_file.size
        return size

    def get_optimization_summary(self, obj):
        try:
            description = json.loads(obj.description or "{}")
        except (TypeError, ValueError, json.JSONDecodeError):
            return {}
        if not isinstance(description, dict):
            return {}
        optimization = description.get("optimization")
        if isinstance(optimization, dict) and optimization:
            return optimization
        manifest = description.get("map_manifest")
        if isinstance(manifest, dict) and isinstance(manifest.get("optimization"), dict):
            return manifest["optimization"]
        return {}

    def get_origin_display(self, obj):
        def finite(value, default=None):
            try:
                number = float(value)
            except (TypeError, ValueError):
                return default
            return number if math.isfinite(number) else default

        grid = list(obj.origin or [])
        result = {
            "map": {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame_id": "map"},
            "occupancy_grid": {
                "x": finite(grid[0], 0.0) if len(grid) > 0 else 0.0,
                "y": finite(grid[1], 0.0) if len(grid) > 1 else 0.0,
                "yaw": finite(grid[2], 0.0) if len(grid) > 2 else 0.0,
            },
            "rtk_enu": None,
        }
        try:
            description = json.loads(obj.description or "{}")
        except (TypeError, ValueError, json.JSONDecodeError):
            description = {}
        origin_text = description.get("gnss_origin_yaml") if isinstance(description, dict) else ""
        try:
            rtk = yaml.safe_load(origin_text) if origin_text else {}
        except yaml.YAMLError:
            rtk = {}
        if not isinstance(rtk, dict) or not rtk.get("alignment_locked"):
            return result
        result["rtk_enu"] = {
            "alignment_locked": True,
            "latitude": finite(rtk.get("origin_latitude")),
            "longitude": finite(rtk.get("origin_longitude")),
            "altitude": finite(rtk.get("origin_altitude")),
            "map_x": finite(rtk.get("map_offset_x"), 0.0),
            "map_y": finite(rtk.get("map_offset_y"), 0.0),
            "map_z": finite(rtk.get("map_offset_z"), 0.0),
            "enu_to_map_yaw": finite(rtk.get("enu_to_map_yaw"), 0.0),
            "confirmed_heading_deg": finite(rtk.get("confirmed_heading_deg")),
            "datum": str(rtk.get("datum") or "WGS84"),
        }
        return result


class MapDataSummarySerializer(MapDataSerializer):
    """Compact map representation for selectors and cards."""

    class Meta(MapDataSerializer.Meta):
        fields = [
            "id", "name", "robot", "robot_name", "robot_code", "thumbnail", "thumbnail_url",
            "resolution", "width", "height", "origin", "active", "parent_map", "coordinate_mode",
            "scene_scope", "localization_mode", "origin_status", "map_completeness", "file_size",
            "origin_display",
            "created_at", "updated_at",
        ]


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


class MapSetSummarySerializer(serializers.ModelSerializer):
    robot_name = serializers.CharField(source="robot.name", read_only=True, allow_null=True)
    member_count = serializers.IntegerField(read_only=True)
    submap_ids = serializers.SerializerMethodField()

    class Meta:
        model = MapSet
        fields = [
            "id", "name", "robot", "robot_name", "version", "manifest", "active",
            "member_count", "submap_ids", "created_at", "updated_at",
        ]

    def get_submap_ids(self, obj):
        return [member.submap_id for member in obj.members.all()]


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
            "scene_scope",
            "global_controller",
            "record_rosbag",
            "created_at",
            "updated_at",
        ]

    def to_representation(self, instance):
        data = super().to_representation(instance)
        route_global_controller = str(data.get("global_controller") or "theta_star").lower()
        if route_global_controller not in {"theta_star", "navfn", "smac_hybrid"}:
            route_global_controller = "theta_star"
            data["global_controller"] = route_global_controller
        # Summary responses omit waypoints. Do not inject an empty list — the
        # route planner treats [] as "already hydrated" and skips detail fetch.
        if "waypoints" not in data:
            return data
        normalized_waypoints = []
        for point in data.get("waypoints") or []:
            if isinstance(point, dict):
                normalized = dict(point)
            elif isinstance(point, (list, tuple)) and len(point) >= 2:
                normalized = {
                    "x": point[0],
                    "y": point[1],
                    "yaw": point[2] if len(point) >= 3 else 0.0,
                }
            else:
                normalized_waypoints.append(point)
                continue
            waypoint_local_controller = str(
                normalized.get("local_controller") or "mppi"
            ).lower()
            normalized["local_controller"] = (
                waypoint_local_controller
                if waypoint_local_controller in {"mppi", "rpp", "ilqr"}
                else "mppi"
            )
            waypoint_global_controller = str(
                normalized.get("global_controller") or route_global_controller
            ).lower()
            normalized["global_controller"] = (
                waypoint_global_controller
                if waypoint_global_controller in {"theta_star", "navfn", "smac_hybrid"}
                else route_global_controller
            )
            normalized_waypoints.append(normalized)
        data["waypoints"] = normalized_waypoints
        return data

    def validate_waypoints(self, value):
        normalized_value = []
        for index, point in enumerate(value or []):
            if isinstance(point, dict):
                normalized = dict(point)
            elif isinstance(point, (list, tuple)) and len(point) >= 2:
                normalized = {"x": point[0], "y": point[1], "yaw": point[2] if len(point) >= 3 else 0.0}
            else:
                raise serializers.ValidationError(f"途经点 {index + 1} 格式无效")
            normalized.setdefault("waypoint_id", f"wp-{uuid.uuid4().hex}")
            normalized_value.append(normalized)
        value = normalized_value
        ids = [str(point["waypoint_id"]) for point in value]
        if len(ids) != len(set(ids)):
            raise serializers.ValidationError("同一路线的 waypoint_id 不能重复")
        for index, point in enumerate(value):
            if not isinstance(point, dict):
                continue
            for field in ("avoidance_to_next", "require_yaw", "rtk_primary_allowed", "force_localization_correction"):
                if field in point and not isinstance(point[field], bool):
                    raise serializers.ValidationError(f"途经点 {index + 1} 的 {field} 必须是布尔值")
            if "dwell_seconds" in point:
                try:
                    dwell_seconds = float(point["dwell_seconds"] or 0)
                except (TypeError, ValueError):
                    raise serializers.ValidationError(f"途经点 {index + 1} 的停留秒数必须是数字")
                if not 0 <= dwell_seconds <= 3600:
                    raise serializers.ValidationError(f"途经点 {index + 1} 的停留秒数必须在 0 到 3600 之间")
        invalid_modes = [
            point.get("localization_mode")
            for point in value
            if isinstance(point, dict)
            and str(point.get("localization_mode") or "ndt").lower() not in {"ndt", "rtk", "ukf"}
        ]
        if invalid_modes:
            raise serializers.ValidationError("途经点定位方式只能是 NDT、UKF 或 RTK")
        for index, point in enumerate(value):
            if not isinstance(point, dict):
                continue
            policy = str(point.get("arrival_policy") or "stop_and_confirm").lower()
            micro_mode = str(point.get("arrival_micro_adjust_mode") or "cmd_vel").lower()
            if micro_mode not in {"cmd_vel", "nav2_goal"}:
                raise serializers.ValidationError(
                    f"途经点 {index + 1} 的到点微调模式只能是 cmd_vel 或 nav2_goal"
                )
            if micro_mode == "nav2_goal" and policy not in {"stop_and_confirm", "precision"}:
                raise serializers.ValidationError(
                    f"途经点 {index + 1} 的 nav2_goal 微调只适用于停车校正确认或精确到点"
                )
            anchor = str(point.get("localization_anchor_preference") or "balanced").lower()
            if anchor not in {"ndt", "rtk", "balanced"}:
                raise serializers.ValidationError(
                    f"途经点 {index + 1} 的定位锚点偏好只能是 ndt、rtk 或 balanced"
                )
        invalid_local_controllers = [
            point.get("local_controller")
            for point in value
            if isinstance(point, dict)
            and str(point.get("local_controller") or "mppi").lower() not in {"rpp", "mppi", "ilqr"}
        ]
        if invalid_local_controllers:
            raise serializers.ValidationError("途经点局部控制器只能是 MPPI、RPP 或 iLQR")
        invalid_global_controllers = [
            point.get("global_controller")
            for point in value
            if isinstance(point, dict)
            and str(point.get("global_controller") or "theta_star").lower() not in {"theta_star", "navfn", "smac_hybrid"}
        ]
        if invalid_global_controllers:
            raise serializers.ValidationError("途经点全局控制器只能是 Theta*、NavFn (A*) 或 Smac Hybrid A*")
        try:
            template_ids = {
                int(point["speech_template_id"])
                for point in value
                if isinstance(point, dict) and point.get("speech_template_id") not in (None, "")
            }
        except (TypeError, ValueError):
            raise serializers.ValidationError("途经点播报文案编号无效")
        if not template_ids:
            return value
        valid_ids = set(
            SpeechTemplate.objects.filter(
                id__in=template_ids,
                category__name=settings.INSPECTION_SPEECH_CATEGORY_NAME,
            ).values_list("id", flat=True)
        )
        if valid_ids != template_ids:
            raise serializers.ValidationError("途经点只能选择“巡检智能播报”分类下的文案")
        return value

    def validate(self, attrs):
        if "name" in attrs:
            name = str(attrs.get("name") or "").strip()
            if not name:
                raise serializers.ValidationError({"name": "路线名称不能为空"})
            attrs["name"] = name
            duplicate_query = PatrolRoute.objects.filter(name=name)
            if self.instance is not None:
                duplicate_query = duplicate_query.exclude(pk=self.instance.pk)
            if duplicate_query.exists():
                raise serializers.ValidationError({
                    "code": "ROUTE_NAME_EXISTS",
                    "detail": "路线名称已存在，请修改名称后再新建",
                })
        map_data = attrs.get("map_data") or getattr(self.instance, "map_data", None)
        waypoints = attrs.get("waypoints")
        if waypoints is None and self.instance is not None:
            waypoints = self.instance.waypoints
        scene_scope = attrs.get("scene_scope")
        if scene_scope is None and self.instance is not None:
            scene_scope = self.instance.scene_scope
        global_controller = attrs.get("global_controller")
        if global_controller is not None and str(global_controller).lower() not in {"theta_star", "navfn", "smac_hybrid"}:
            raise serializers.ValidationError("路线全局控制器只能是 Theta*、NavFn (A*) 或 Smac Hybrid A*")
        if map_data is not None:
            try:
                validate_route_against_map(
                    constraints_from_map_data(map_data),
                    scene_scope=str(scene_scope or map_data.scene_scope or "indoor"),
                    waypoints=waypoints or [],
                )
            except MapConstraintError as exc:
                raise serializers.ValidationError(exc.message) from exc
            validate_waypoints_against_boundary(map_data, waypoints or [])
        return attrs


class PatrolRouteSummarySerializer(PatrolRouteSerializer):
    waypoint_count = serializers.SerializerMethodField()
    latest_execution = serializers.SerializerMethodField()

    class Meta(PatrolRouteSerializer.Meta):
        fields = [
            "id", "name", "map_data", "map_name", "map_set", "map_set_name", "robot",
            "robot_name", "robot_code", "waypoint_count", "description", "scene_scope", "global_controller",
            "record_rosbag", "latest_execution", "created_at", "updated_at",
        ]

    def get_waypoint_count(self, obj):
        return len(obj.waypoints or [])

    def get_latest_execution(self, obj):
        prefetched = getattr(obj, "recent_executions", None)
        execution = prefetched[0] if prefetched else None
        if execution is None and prefetched is None:
            execution = obj.executions.order_by("-created_at").first()
        if execution is None:
            return None
        return {
            "id": str(execution.id),
            "task_id": execution.task_id,
            "state": execution.state,
            "created_at": execution.created_at,
            "started_at": execution.started_at,
            "finished_at": execution.finished_at,
        }


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
            "speed_limit_mps",
            "warning_distance_m",
            "created_at",
            "updated_at",
        ]


class SystemLogSerializer(serializers.ModelSerializer):
    class Meta:
        model = SystemLog
        fields = [
            "id", "robot", "occurred_at", "received_at", "level", "module",
            "event_code", "message", "source", "data", "trace_id",
            "task_execution", "command", "map_data", "route", "waypoint_index",
            "waypoint_id", "round_number", "nav_goal_generation",
            "localization_generation", "dedupe_key", "x", "y", "yaw", "repeat_count",
        ]


class DebugLogSessionSerializer(serializers.ModelSerializer):
    class Meta:
        model = DebugLogSession
        fields = [
            "id", "robot", "modules", "sample_hz", "status", "started_at",
            "expires_at", "stopped_at", "created_by",
        ]
        read_only_fields = [
            "id", "status", "started_at", "expires_at", "stopped_at", "created_by",
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


class TrackSummarySerializer(TrackSerializer):
    point_count = serializers.SerializerMethodField()

    class Meta(TrackSerializer.Meta):
        fields = [
            "id", "robot", "robot_name", "robot_code", "map_data", "map_name", "route",
            "route_name", "task", "task_name", "start_time", "end_time", "distance",
            "duration", "point_count", "description", "created_at", "updated_at",
        ]

    def get_point_count(self, obj):
        return len(obj.path or [])


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
    raw_rtk = serializers.SerializerMethodField()
    time_diagnostics = serializers.SerializerMethodField()
    current_map = serializers.SerializerMethodField()
    sensors = serializers.SerializerMethodField()
    power = serializers.SerializerMethodField()
    network = serializers.SerializerMethodField()
    audio = serializers.SerializerMethodField()
    navigation = serializers.SerializerMethodField()
    power_mode = serializers.SerializerMethodField()

    def get_localization_quality(self, obj):
        raw_localization = (obj.raw_payload or {}).get("localization") or {}
        quality = dict(obj.localization_quality or {})
        quality.update(raw_localization.get("quality") or {})
        decision = raw_localization.get("decision") or quality.get("decision")
        if decision:
            quality["decision"] = dict(decision)
        return quality

    def get_current_map(self, obj):
        return (obj.raw_payload or {}).get("current_map") or {
            "map_id": obj.map_id,
            "map_version": obj.map_version,
        }

    def get_raw_rtk(self, obj):
        raw_localization = (obj.raw_payload or {}).get("localization") or {}
        return raw_localization.get("raw_rtk")

    def get_time_diagnostics(self, obj):
        raw_localization = (obj.raw_payload or {}).get("localization") or {}
        return raw_localization.get("time_diagnostics") or {}

    def get_sensors(self, obj):
        return (obj.raw_payload or {}).get("sensors") or {}

    def get_power(self, obj):
        return (obj.raw_payload or {}).get("power") or {
            "available": obj.power_available,
            "percent": obj.battery_percent,
            "charging": obj.charging,
        }

    def get_network(self, obj):
        return (obj.raw_payload or {}).get("network") or {
            "available": obj.signal_percent is not None,
            "type": obj.network_type,
            "signal_percent": obj.signal_percent,
        }

    def get_audio(self, obj):
        return (obj.raw_payload or {}).get("audio") or {}

    def get_navigation(self, obj):
        return (obj.raw_payload or {}).get("navigation") or {}

    def get_power_mode(self, obj):
        return (obj.raw_payload or {}).get("power_mode") or {
            "mode": "normal",
            "transition_state": "unknown",
            "auto_charge_enabled": False,
            "services": {},
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
            "raw_rtk",
            "time_diagnostics",
            "sensors",
            "power_available",
            "battery_percent",
            "charging",
            "power",
            "network_type",
            "signal_percent",
            "network",
            "audio",
            "navigation",
            "power_mode",
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
            "loop_session_id",
            "round_number",
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


class PatrolLoopEventSerializer(serializers.ModelSerializer):
    class Meta:
        model = PatrolLoopEvent
        fields = [
            "id",
            "event_type",
            "state",
            "occurred_at",
            "reason_code",
            "reason_message",
            "recovery_attempt",
            "payload",
        ]


class PatrolLoopSessionSerializer(serializers.ModelSerializer):
    robot_name = serializers.CharField(source="robot.name", read_only=True)
    task_name = serializers.CharField(source="task.name", read_only=True)
    current_execution_detail = TaskExecutionSerializer(source="current_execution", read_only=True)
    recent_events = serializers.SerializerMethodField()
    total_distance_m = serializers.SerializerMethodField()
    record_rosbag = serializers.SerializerMethodField()

    class Meta:
        model = PatrolLoopSession
        fields = [
            "id",
            "robot",
            "robot_name",
            "task",
            "task_name",
            "state",
            "state_version",
            "duration_seconds",
            "rest_seconds",
            "record_rosbag",
            "started_at",
            "ends_at",
            "finished_at",
            "current_round",
            "total_distance_m",
            "current_execution",
            "current_execution_detail",
            "next_action_at",
            "observation_started_at",
            "recovery_episode_id",
            "recovery_reason_code",
            "recovery_reason_message",
            "recovery_attempt",
            "recovery_max_attempts",
            "manual_paused",
            "last_error",
            "metadata",
            "recent_events",
            "created_at",
            "updated_at",
        ]

    def get_recent_events(self, obj):
        return PatrolLoopEventSerializer(obj.events.order_by("-occurred_at")[:20], many=True).data

    def get_record_rosbag(self, obj):
        return bool((obj.metadata or {}).get("record_rosbag", False))

    def get_total_distance_m(self, obj):
        distance = stored_loop_total_distance(obj)
        if distance is None:
            distance = calculate_loop_total_distance(obj.id)
        return format(distance, "f")


class PatrolLoopSessionCreateSerializer(serializers.Serializer):
    task_id = serializers.IntegerField(min_value=1)
    duration_seconds = serializers.IntegerField(min_value=1, max_value=86400)
    rest_seconds = serializers.IntegerField(min_value=0, max_value=3600, default=0)
    session_id = serializers.UUIDField(required=False)


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


class ValidationRecordingSerializer(serializers.ModelSerializer):
    robot_code = serializers.CharField(source="robot.code", read_only=True, allow_null=True)

    class Meta:
        model = ValidationRecording
        fields = [
            "id", "robot", "robot_code", "task_execution", "label", "storage_format", "state",
            "sha256", "size_bytes", "duration_seconds", "start_time_ns", "end_time_ns",
            "topic_manifest", "recording_manifest", "uploaded_at", "invalid_reason",
            "created_at", "updated_at",
        ]
        read_only_fields = [
            "state", "sha256", "size_bytes", "uploaded_at", "invalid_reason", "created_at", "updated_at"
        ]


class ValidationProfileSerializer(serializers.ModelSerializer):
    class Meta:
        model = ValidationProfile
        fields = [
            "id", "name", "version", "mode", "enabled", "description", "required_topics",
            "replay_topics", "output_topics", "thresholds", "runner_config",
        ]


class ValidationRunnerSerializer(serializers.ModelSerializer):
    class Meta:
        model = ValidationRunner
        fields = [
            "id", "display_name", "kind", "state", "capabilities", "version", "last_seen_at"
        ]


class ValidationCheckResultSerializer(serializers.ModelSerializer):
    class Meta:
        model = ValidationCheckResult
        fields = [
            "id", "rule_id", "title", "category", "status", "severity", "hard_failure",
            "metric_name", "actual_value", "expected_value", "start_time_ns", "end_time_ns",
            "topics", "evidence", "message",
        ]


class ValidationArtifactSerializer(serializers.ModelSerializer):
    download_path = serializers.SerializerMethodField()
    signed_download_path = serializers.SerializerMethodField()

    class Meta:
        model = ValidationArtifact
        fields = [
            "id", "role", "name", "sha256", "size_bytes", "content_type", "metadata",
            "download_path", "signed_download_path", "created_at",
        ]

    def get_download_path(self, obj):
        return f"/api/validation-jobs/{obj.job_id}/artifacts/{obj.id}/"

    def get_signed_download_path(self, obj):
        token = signing.TimestampSigner(salt="validation-artifact").sign(str(obj.id))
        return f"/api/validation-artifacts/{obj.id}/signed/?token={token}"


class ValidationAttemptSerializer(serializers.ModelSerializer):
    runner_name = serializers.CharField(source="runner.display_name", read_only=True)

    class Meta:
        model = ValidationAttempt
        fields = [
            "number", "runner", "runner_name", "state", "started_at", "finished_at",
            "error_code", "error_message", "runtime_metrics",
        ]


class ValidationJobSerializer(serializers.ModelSerializer):
    recording_label = serializers.CharField(source="recording.label", read_only=True)
    profile_name = serializers.CharField(source="profile.name", read_only=True)
    profile_version = serializers.IntegerField(source="profile.version", read_only=True)
    runner_name = serializers.CharField(source="runner.display_name", read_only=True, allow_null=True)
    checks = ValidationCheckResultSerializer(source="check_results", many=True, read_only=True)
    artifacts = ValidationArtifactSerializer(many=True, read_only=True)
    attempts = ValidationAttemptSerializer(many=True, read_only=True)

    class Meta:
        model = ValidationJob
        fields = [
            "id", "recording", "recording_label", "profile", "profile_name", "profile_version",
            "baseline_job", "mode", "state", "verdict", "progress_percent", "runner", "runner_name",
            "attempt_count", "requested_config", "resolved_config", "summary", "error_code",
            "error_message", "stack_git_sha", "stack_image_digest", "live_bridge_url", "queued_at",
            "started_at", "finished_at", "cancel_requested_at", "created_at", "updated_at",
            "checks", "artifacts", "attempts",
        ]


class GoldenBaselineSerializer(serializers.ModelSerializer):
    profile_name = serializers.CharField(source="profile.name", read_only=True)

    class Meta:
        model = GoldenBaseline
        fields = [
            "id", "profile", "profile_name", "job", "map_hash", "route_hash",
            "stack_image_digest", "active", "approved_by", "created_at",
        ]
