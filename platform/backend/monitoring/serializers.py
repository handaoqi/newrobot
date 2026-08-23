from __future__ import annotations

from pathlib import Path
from urllib.parse import urlparse

from django.conf import settings
from django.utils import timezone
from rest_framework import serializers
from PIL import Image, ImageDraw

from .models import (
    AlertSkillBinding,
    CalendarDay,
    CommandEvent,
    InspectionEvent,
    MediaAsset,
    PatrolTask,
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
    MapSet,
    MapSetMember,
    PatrolRoute,
    Zone,
    Track,
    ScheduleRun,
)

from .services.map_coordinate import MapConstraintError, constraints_from_map_data, validate_route_against_map


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

    def get_today_alerts(self, obj):
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
            "loop_session_id": str(execution.loop_session_id) if execution.loop_session_id else None,
            "round_number": execution.round_number,
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
            if label not in {"person", "bicycle", "bike", "自行车"}:
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
            "scene_scope",
            "created_at",
            "updated_at",
        ]

    def validate_waypoints(self, value):
        for index, point in enumerate(value):
            if not isinstance(point, dict):
                continue
            for field in ("avoidance_to_next", "require_yaw"):
                if field in point and not isinstance(point[field], bool):
                    raise serializers.ValidationError(f"途经点 {index + 1} 的 {field} 必须是布尔值")
        invalid_modes = [
            point.get("localization_mode")
            for point in value
            if isinstance(point, dict)
            and str(point.get("localization_mode") or "ndt").lower() not in {"ndt", "rtk"}
        ]
        if invalid_modes:
            raise serializers.ValidationError("途经点定位方式只能是 NDT 或 RTK")
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
        map_data = attrs.get("map_data") or getattr(self.instance, "map_data", None)
        waypoints = attrs.get("waypoints")
        if waypoints is None and self.instance is not None:
            waypoints = self.instance.waypoints
        scene_scope = attrs.get("scene_scope")
        if scene_scope is None and self.instance is not None:
            scene_scope = self.instance.scene_scope
        if map_data is not None:
            try:
                validate_route_against_map(
                    constraints_from_map_data(map_data),
                    scene_scope=str(scene_scope or map_data.scene_scope or "indoor"),
                    waypoints=waypoints or [],
                )
            except MapConstraintError as exc:
                raise serializers.ValidationError(exc.message) from exc
        return attrs


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
