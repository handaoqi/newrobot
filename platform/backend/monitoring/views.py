import hashlib
import io
import json
import logging
import math
import uuid
import zipfile
import yaml
from datetime import datetime, time as datetime_time, timedelta
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from django.conf import settings
from django.contrib.auth import authenticate, get_user_model
from django.core.files.base import ContentFile
from django.http import FileResponse, HttpResponse, HttpResponseForbidden, StreamingHttpResponse
from django.db import IntegrityError, transaction
from django.db.models import Case, Count, IntegerField, Max, Min, Q, When
from django.db.models.deletion import ProtectedError
from django.shortcuts import get_object_or_404
from django.utils.dateparse import parse_date, parse_datetime
from django.utils import timezone
from rest_framework import permissions, status
from rest_framework.authtoken.models import Token
from rest_framework.decorators import api_view, permission_classes
from rest_framework.exceptions import ValidationError
from rest_framework.response import Response
from rest_framework.views import APIView

from .models import (
    AlertSkillBinding,
    InspectionEvent,
    CalendarDay,
    MediaAsset,
    PatrolTask,
    PatrolSchedule,
    Robot,
    RemoteCommand,
    RobotPersonDetectionState,
    RobotCommand,
    RecordedAudio,
    SpeechCategory,
    SpeechTemplate,
    RobotSession,
    RobotStatusLatest,
    RobotTelemetry,
    TaskExecution,
    ScheduleRun,
    TrajectoryPoint,
    MapData,
    MapSet,
    MapSetMember,
    PatrolRoute,
    Zone,
    Track,
)
from .permissions import IsAudioDeviceCredential, IsAuthenticatedOrDeviceCredential
from .realtime import event_broker, sse_stream
from .services.alert_service import AlertService
from .services.command_service import CommandService
from .services.schedule_service import ScheduleService
from .services.task_service import TaskExecutionService, TaskStateError
from .services import asr_service, tts_service
from .services.alert_skill_service import resolve_alert_template
from .serializers import (
    AlertSkillBindingSerializer,
    AlertSkillPreviewSerializer,
    EventSerializer,
    CalendarDaySerializer,
    MediaAssetSerializer,
    MediaUploadSerializer,
    PatrolTaskSerializer,
    PatrolTaskCreateSerializer,
    PatrolScheduleSerializer,
    RobotCommandCreateSerializer,
    RobotCommandSerializer,
    RecordedAudioSerializer,
    SpeechCategorySerializer,
    SpeechTemplateSerializer,
    SpeechSynthesisSerializer,
    RobotDetailSerializer,
    RobotSerializer,
    RemoteCommandSerializer,
    TelemetryIngestSerializer,
    PersonDetectionIngestSerializer,
    MapDataSerializer,
    MapSetSerializer,
    PatrolRouteSerializer,
    ZoneSerializer,
    TrackSerializer,
    AlertTimelineSerializer,
    RobotSessionSerializer,
    RobotStatusSerializer,
    TaskExecutionActionSerializer,
    TaskExecutionSerializer,
    TrajectoryPointSerializer,
    ScheduleRunSerializer,
)


# A detection can spend time in model inference and network transport.  Online
# status must therefore reflect when the platform last received a frame, not
# the camera-side timestamp embedded in that frame.
PERSON_DETECTION_STALE_SECONDS = 8


def _person_detection_is_stale(state: RobotPersonDetectionState) -> bool:
    return timezone.now() - state.updated_at > timedelta(seconds=PERSON_DETECTION_STALE_SECONDS)

User = get_user_model()
LOGGER = logging.getLogger(__name__)


def build_public_media_url(request, saved_path: str) -> str:
    media_url = settings.MEDIA_URL if settings.MEDIA_URL.startswith("/") else f"/{settings.MEDIA_URL}"
    media_path = f"{media_url.rstrip('/')}/{saved_path}"
    public_base_url = getattr(settings, "PUBLIC_BASE_URL", "")
    return f"{public_base_url}{media_path}" if public_base_url else request.build_absolute_uri(media_path)


def is_bicycle_detection(detection: dict) -> bool:
    values = {
        str(detection.get("object_class") or "").strip().lower(),
        str(detection.get("type") or "").strip().lower(),
        str(detection.get("label") or "").strip().lower(),
    }
    if values & {"bicycle", "bike", "自行车", "vehicle_illegal_parking", "自行车违停"}:
        return True
    return any("自行车" in value or "bicycle" in value for value in values)


def queue_bicycle_departure_speech(request, robot: Robot, detection: dict, event: InspectionEvent):
    if not settings.BICYCLE_AUTO_SPEECH_ENABLED or not is_bicycle_detection(detection):
        return None

    cutoff = timezone.now() - timedelta(seconds=settings.BICYCLE_AUTO_SPEECH_COOLDOWN_SECONDS)
    recent_filter = {
        "robot": robot,
        "action": "play_audio",
        "payload__source": "vision_bicycle_auto",
        "created_at__gte": cutoff,
    }
    if RobotCommand.objects.filter(**recent_filter).exists():
        return None

    template = resolve_alert_template("bicycle_alert", settings.BICYCLE_AUTO_SPEECH_TEMPLATE_NAME)
    if not template:
        LOGGER.warning("automatic bicycle speech template missing: %s", settings.BICYCLE_AUTO_SPEECH_TEMPLATE_NAME)
        return None

    try:
        saved_path, cache_hit = tts_service.synthesize_speech(template.text)
    except Exception:
        LOGGER.exception("automatic bicycle speech synthesis failed robot=%s event=%s", robot.code, event.event_id)
        return None

    with transaction.atomic():
        locked_robot = Robot.objects.select_for_update().get(pk=robot.pk)
        if RobotCommand.objects.filter(**{**recent_filter, "robot": locked_robot}).exists():
            return None
        return RobotCommand.objects.create(
            robot=locked_robot,
            action="play_audio",
            payload={
                "audio_url": build_public_media_url(request, saved_path),
                "audio_name": template.name,
                "text": template.text,
                "source": "vision_bicycle_auto",
                "alert_skill": "bicycle_alert",
                "dual_output": True,
                "content_type": "audio/mpeg",
                "tts_cache_hit": cache_hit,
                "inspection_event_id": str(event.event_id),
                "object_class": detection.get("object_class", ""),
                "track_id": detection.get("track_id", ""),
            },
        )


def build_period_labels(days: int = 7):
    today = timezone.localdate()
    dates = [today - timezone.timedelta(days=offset) for offset in range(days - 1, -1, -1)]
    return dates


def serialize_trend(title, subtitle, unit, accent, points):
    latest = points[-1]["value"] if points else 0
    previous = points[-2]["value"] if len(points) > 1 else latest
    delta = latest - previous
    total = round(sum(item["value"] for item in points), 1) if points else 0
    return {
        "title": title,
        "subtitle": subtitle,
        "unit": unit,
        "accent": accent,
        "series": points,
        "summary": {
            "latest": latest,
            "delta": abs(delta),
            "direction": "较上一周期上升" if delta >= 0 else "较上一周期回落",
            "total": total,
            "average": round(total / len(points), 1) if points else 0,
        },
    }


def _daily_counts(queryset, date_field, dates):
    """按本地日期统计 queryset 行数，对齐到 `dates`，返回 [{"label","value"}]。"""
    buckets = {date: 0 for date in dates}
    for value in queryset.values_list(date_field, flat=True).iterator(chunk_size=2000):
        if value is None:
            continue
        local_date = timezone.localtime(value).date()
        if local_date in buckets:
            buckets[local_date] += 1
    return [{"label": date.strftime("%m-%d"), "value": buckets[date]} for date in dates]


def _daily_active_minutes(telemetry_qs, dates):
    """按本地日期统计遥测活跃时长（当日 max-min(reported_at) 分钟），无数据则 0。"""
    date_set = set(dates)
    spans = {date: [None, None] for date in dates}
    for reported_at in telemetry_qs.values_list("reported_at", flat=True).iterator(chunk_size=2000):
        if reported_at is None:
            continue
        local_dt = timezone.localtime(reported_at)
        day = local_dt.date()
        if day not in date_set:
            continue
        low, high = spans[day]
        spans[day][0] = local_dt if low is None else min(low, local_dt)
        spans[day][1] = local_dt if high is None else max(high, local_dt)
    result = []
    for date in dates:
        low, high = spans[date]
        minutes = round((high - low).total_seconds() / 60, 1) if low and high else 0
        result.append({"label": date.strftime("%m-%d"), "value": minutes})
    return result


def _haversine_km(lat1, lon1, lat2, lon2):
    radius = 6371.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    return 2 * radius * math.asin(math.sqrt(a))


def _daily_mileage_km(telemetry_qs, dates):
    """按本地日期统计相邻遥测点经纬度的 Haversine 累距（公里），无数据则 0。"""
    date_set = set(dates)
    distances = {date: 0.0 for date in dates}
    previous_points = {}
    rows = (
        telemetry_qs.filter(latitude__isnull=False, longitude__isnull=False)
        .values_list("reported_at", "latitude", "longitude")
        .order_by("reported_at")
    )
    for reported_at, lat, lon in rows.iterator(chunk_size=2000):
        if reported_at is None:
            continue
        day = timezone.localtime(reported_at).date()
        if day not in date_set:
            continue
        point = (float(lat), float(lon))
        previous = previous_points.get(day)
        if previous:
            distances[day] += _haversine_km(*previous, *point)
        previous_points[day] = point
    result = []
    for date in dates:
        result.append({"label": date.strftime("%m-%d"), "value": round(distances[date], 2)})
    return result


def _daily_telemetry_series(telemetry_qs, dates):
    """一次读取遥测点，同时计算每日活跃时长和经纬度累计里程。"""
    date_set = set(dates)
    spans = {date: [None, None] for date in dates}
    distances = {date: 0.0 for date in dates}
    previous_points = {}
    rows = (
        telemetry_qs.values_list("reported_at", "latitude", "longitude")
        .order_by("reported_at")
    )
    for reported_at, lat, lon in rows.iterator(chunk_size=2000):
        if reported_at is None:
            continue
        local_dt = timezone.localtime(reported_at)
        day = local_dt.date()
        if day not in date_set:
            continue
        low, high = spans[day]
        spans[day][0] = local_dt if low is None else min(low, local_dt)
        spans[day][1] = local_dt if high is None else max(high, local_dt)
        if lat is None or lon is None:
            continue
        point = (float(lat), float(lon))
        previous = previous_points.get(day)
        if previous:
            distances[day] += _haversine_km(*previous, *point)
        previous_points[day] = point

    active_minutes = []
    mileage = []
    for date in dates:
        low, high = spans[date]
        minutes = round((high - low).total_seconds() / 60, 1) if low and high else 0
        active_minutes.append({"label": date.strftime("%m-%d"), "value": minutes})
        mileage.append({"label": date.strftime("%m-%d"), "value": round(distances[date], 2)})
    return active_minutes, mileage


def request_force_delete(request) -> bool:
    value = request.query_params.get("force", request.data.get("force") if hasattr(request, "data") else "")
    return str(value).lower() in {"1", "true", "yes", "y"}


def _delete_file_fields(file_fields) -> None:
    for file_field in file_fields:
        if file_field:
            file_field.delete(save=False)


def _parse_simple_map_yaml(content: bytes) -> dict:
    metadata = {}
    for raw_line in content.decode("utf-8", errors="ignore").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip().strip("'\"")
        if key == "resolution":
            try:
                metadata["resolution"] = float(value)
            except ValueError:
                pass
        elif key == "origin":
            try:
                metadata["origin"] = json.loads(value.replace("'", '"'))
            except json.JSONDecodeError:
                numbers = [item.strip() for item in value.strip("[]").split(",") if item.strip()]
                try:
                    metadata["origin"] = [float(item) for item in numbers]
                except ValueError:
                    pass
        elif key == "image":
            metadata["image"] = value
    return metadata


def _read_pgm_dimensions(content: bytes) -> tuple[int, int]:
    tokens = []
    for raw_line in content.splitlines():
        line = raw_line.split(b"#", 1)[0].strip()
        if not line:
            continue
        tokens.extend(line.split())
        if len(tokens) >= 3:
            break
    if len(tokens) < 3 or tokens[0] not in {b"P2", b"P5"}:
        return 0, 0
    try:
        return int(tokens[1]), int(tokens[2])
    except ValueError:
        return 0, 0


def _map_references(map_data: MapData) -> dict[str, int]:
    return {
        "路线": map_data.routes.count(),
        "禁区": map_data.zones.count(),
        "轨迹": map_data.tracks.count(),
        "任务执行记录": map_data.task_executions.count(),
        "告警事件": map_data.inspection_events.count(),
        "日历计划": map_data.calendar_schedules.count(),
    }


def _map_activation_payload(map_data: MapData, request) -> dict:
    description = {}
    if map_data.description:
        try:
            description = json.loads(map_data.description)
        except (TypeError, json.JSONDecodeError):
            description = {}
    local_map_dir = description.get("source_map_dir", "") if isinstance(description, dict) else ""
    local_image_path = f"{local_map_dir}/map.pgm" if local_map_dir else ""
    if not local_map_dir:
        local_image_path = description.get("image", "") if isinstance(description, dict) else ""
        local_map_dir = local_image_path.rsplit("/", 1)[0] if local_image_path and "/" in local_image_path else ""
    if not local_map_dir and map_data.name:
        # Older uploaded maps did not store the edge-local image path in
        # description. Their name is the edge session directory, e.g.
        # 20260703_170635, so infer the local source directory from it.
        local_map_dir = f"/home/dogrobot/runtime/nx-edge/data/jszr/map/{map_data.name}"
        local_image_path = f"{local_map_dir}/map.pgm"
    map_version = f"legacy-mapdata-{map_data.id}"
    edit_metadata = map_data.edit_metadata if isinstance(map_data.edit_metadata, dict) else {}
    return {
        "map_id": str(map_data.id),
        "map_version": map_version,
        "map_name": map_data.name,
        "source_map_version": description.get("map_version", "") if isinstance(description, dict) else "",
        "local_map_dir": local_map_dir,
        "local_image_path": local_image_path,
        "pgm_url": request.build_absolute_uri(map_data.pgm_file.url) if map_data.pgm_file else "",
        "yaml_url": request.build_absolute_uri(map_data.yaml_file.url) if map_data.yaml_file else "",
        "package_url": request.build_absolute_uri(map_data.package_file.url) if map_data.package_file else "",
        "resolution": map_data.resolution,
        "origin": map_data.origin,
        "width": map_data.width,
        "height": map_data.height,
        "manual_edit": edit_metadata.get("mode") == "manual_cleanup",
        "pgm_sha256": _file_sha256(map_data.pgm_file.path) if map_data.pgm_file else "",
        "yaml_sha256": _file_sha256(map_data.yaml_file.path) if map_data.yaml_file else "",
        "package_sha256": _file_sha256(map_data.package_file.path) if map_data.package_file else "",
        "gnss_origin_yaml": description.get("gnss_origin_yaml", "") if isinstance(description, dict) else "",
        "map_manifest": description.get("map_manifest", {}) if isinstance(description, dict) else {},
        "coordinate_mode": map_data.coordinate_mode,
        "scene_scope": map_data.scene_scope,
        "localization_mode": map_data.localization_mode,
        "origin_status": map_data.origin_status,
    }


def _file_sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_map_description(map_data: MapData) -> dict:
    try:
        value = json.loads(map_data.description or "{}")
    except (TypeError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _render_manual_cleanup(map_data: MapData, strokes) -> tuple[bytes, bytes, int, list[dict]]:
    from PIL import Image, ImageDraw

    if not map_data.pgm_file or not map_data.yaml_file:
        raise ValueError("地图缺少 PGM 或 YAML 文件")
    if not isinstance(strokes, list) or not strokes:
        raise ValueError("至少需要一条擦除轨迹")
    if len(strokes) > 500:
        raise ValueError("擦除轨迹过多，请分批保存")

    with Image.open(map_data.pgm_file.path) as source:
        image = source.convert("L")
    width, height = image.size
    if map_data.width and map_data.height and (width, height) != (map_data.width, map_data.height):
        raise ValueError("地图文件尺寸与数据库记录不一致")

    mask = Image.new("L", image.size, 0)
    draw = ImageDraw.Draw(mask)
    normalized = []
    total_points = 0
    resolution = max(float(map_data.resolution or 0.05), 0.001)
    for stroke in strokes:
        if not isinstance(stroke, dict):
            raise ValueError("擦除轨迹格式错误")
        points = stroke.get("points") or []
        if not isinstance(points, list) or not points:
            continue
        total_points += len(points)
        if total_points > 20000:
            raise ValueError("擦除采样点过多，请减少笔画后重试")
        diameter_m = float(stroke.get("diameter_m") or 0.5)
        if not 0.05 <= diameter_m <= 5.0:
            raise ValueError("画笔尺寸必须在 0.05m 到 5m 之间")
        parsed_points = []
        for point in points:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError("擦除坐标格式错误")
            x, y = float(point[0]), float(point[1])
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError("擦除坐标必须是有限数字")
            parsed_points.append((min(max(x, 0.0), width - 1.0), min(max(y, 0.0), height - 1.0)))
        brush_px = max(1, round(diameter_m / resolution))
        if len(parsed_points) == 1:
            x, y = parsed_points[0]
            radius = brush_px / 2
            draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=255)
        else:
            draw.line(parsed_points, fill=255, width=brush_px, joint="curve")
            radius = brush_px / 2
            for x, y in (parsed_points[0], parsed_points[-1]):
                draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=255)
        normalized.append({
            "diameter_m": diameter_m,
            "points": [[round(x, 2), round(y, 2)] for x, y in parsed_points],
        })

    erased_cells = sum(1 for value in mask.getdata() if value)
    if erased_cells == 0:
        raise ValueError("擦除区域为空")
    image.paste(255, mask=mask)
    pgm_buffer = io.BytesIO()
    image.save(pgm_buffer, format="PPM")
    preview_buffer = io.BytesIO()
    image.save(preview_buffer, format="PNG", optimize=True)
    return pgm_buffer.getvalue(), preview_buffer.getvalue(), erased_cells, normalized


def _task_force_delete_counts(task: PatrolTask) -> dict[str, int]:
    execution_ids = list(task.executions.values_list("id", flat=True))
    schedule_ids = list(task.calendar_schedules.values_list("id", flat=True))
    return {
        "日历计划": len(schedule_ids),
        "计划运行记录": ScheduleRun.objects.filter(
            Q(schedule_id__in=schedule_ids) | Q(task_execution_id__in=execution_ids)
        ).count(),
        "任务执行记录": len(execution_ids),
        "轨迹": task.tracks.count(),
        "告警事件": InspectionEvent.objects.filter(task_execution_id__in=execution_ids).count(),
    }


def _force_delete_patrol_task(task: PatrolTask) -> dict[str, int]:
    active_executions = task.executions.filter(state__in=TaskExecution.ACTIVE_STATES)
    if active_executions.exists():
        raise ProtectedError("该巡检任务仍有执行中的记录，请先终止任务后再强制删除。", active_executions)
    counts = _task_force_delete_counts(task)
    execution_ids = list(task.executions.values_list("id", flat=True))
    schedule_ids = list(task.calendar_schedules.values_list("id", flat=True))
    with transaction.atomic():
        InspectionEvent.objects.filter(task_execution_id__in=execution_ids).delete()
        Track.objects.filter(task=task).delete()
        ScheduleRun.objects.filter(Q(schedule_id__in=schedule_ids) | Q(task_execution_id__in=execution_ids)).delete()
        task.calendar_schedules.all().delete()
        task.executions.all().delete()
        task.delete()
    return counts


def _force_delete_map(map_data: MapData) -> dict:
    route_ids = list(map_data.routes.values_list("id", flat=True))
    tasks = PatrolTask.objects.filter(route_id__in=route_ids)
    task_ids = list(tasks.values_list("id", flat=True))
    executions = TaskExecution.objects.filter(
        Q(map_data=map_data) | Q(route_id__in=route_ids) | Q(task_id__in=task_ids)
    )
    active_executions = executions.filter(state__in=TaskExecution.ACTIVE_STATES)
    if active_executions.exists():
        raise ProtectedError("该地图仍有关联任务正在执行，请先终止任务后再强制删除。", active_executions)

    execution_ids = list(executions.values_list("id", flat=True))
    schedules = PatrolSchedule.objects.filter(
        Q(map_data=map_data) | Q(route_id__in=route_ids) | Q(task_template_id__in=task_ids)
    )
    schedule_ids = list(schedules.values_list("id", flat=True))
    counts = {
        "路线": len(route_ids),
        "禁区": map_data.zones.count(),
        "轨迹": Track.objects.filter(Q(map_data=map_data) | Q(route_id__in=route_ids) | Q(task_id__in=task_ids)).count(),
        "巡检任务": len(task_ids),
        "任务执行记录": len(execution_ids),
        "日历计划": len(schedule_ids),
        "计划运行记录": ScheduleRun.objects.filter(
            Q(schedule_id__in=schedule_ids) | Q(task_execution_id__in=execution_ids)
        ).count(),
        "告警事件": InspectionEvent.objects.filter(
            Q(map_data=map_data) | Q(task_execution_id__in=execution_ids)
        ).count(),
    }
    files = [
        map_data.pgm_file,
        map_data.yaml_file,
        map_data.thumbnail,
        map_data.trajectory_file,
        map_data.mapping_trace,
        map_data.package_file,
    ]
    robot = map_data.robot
    was_active = map_data.active
    with transaction.atomic():
        InspectionEvent.objects.filter(Q(map_data=map_data) | Q(task_execution_id__in=execution_ids)).delete()
        Track.objects.filter(Q(map_data=map_data) | Q(route_id__in=route_ids) | Q(task_id__in=task_ids)).delete()
        ScheduleRun.objects.filter(Q(schedule_id__in=schedule_ids) | Q(task_execution_id__in=execution_ids)).delete()
        schedules.delete()
        executions.delete()
        tasks.delete()
        map_data.zones.all().delete()
        map_data.routes.all().delete()
        map_data.active = False
        map_data.save(update_fields=["active", "updated_at"])
        map_data.delete()
        fallback = None
        if was_active:
            fallback = MapData.objects.filter(robot=robot).order_by("-created_at").first()
            if fallback:
                MapData.objects.filter(active=True).update(active=False)
                fallback.active = True
                fallback.save(update_fields=["active", "updated_at"])
    _delete_file_fields(files)
    return {"deleted": counts, "fallback_active_map_id": fallback.id if was_active and fallback else None}


def build_analytics_payload():
    ensure_demo_seed()
    dates = build_period_labels(7)
    start_date = dates[0]
    # Use local-midnight bounds instead of __date__ filters, avoiding a
    # non-sargable date conversion and limiting the rows returned to 7 days.
    start_at = timezone.make_aware(datetime.combine(start_date, datetime.min.time()))
    end_at = timezone.make_aware(datetime.combine(dates[-1] + timedelta(days=1), datetime.min.time()))

    events = InspectionEvent.objects.filter(detected_at__gte=start_at, detected_at__lt=end_at)

    risk_weight_map = {"high": 3, "medium": 2, "low": 1}
    risk_totals = {date: 0 for date in dates}
    for event in events.only("detected_at", "risk_level"):
        event_date = timezone.localtime(event.detected_at).date()
        if event_date in risk_totals:
            risk_totals[event_date] += risk_weight_map.get(event.risk_level, 1)

    # 逐日真实聚合（今天往前滚动 7 天，窗口外的历史数据不参与本次统计）。
    window_events = events
    window_snapshots = MediaAsset.objects.filter(
        media_type="snapshot", event_time__gte=start_at, event_time__lt=end_at
    )
    window_telemetry = RobotTelemetry.objects.filter(reported_at__gte=start_at, reported_at__lt=end_at)

    alert_series = _daily_counts(window_events, "detected_at", dates)
    detection_series = _daily_counts(window_snapshots, "event_time", dates)
    duration_series, mileage_series = _daily_telemetry_series(window_telemetry, dates)

    # 平均完成度：真实完成任务的航点完成比均值；无 completed 执行则无数据
    completion_ratios = [
        completed / total
        for completed, total in TaskExecution.objects.filter(
            state="completed",
            total_waypoints__gt=0,
            created_at__gte=start_at,
            created_at__lt=end_at,
        ).values_list("completed_waypoints", "total_waypoints").iterator(chunk_size=2000)
    ]
    if completion_ratios:
        completion_value = f"{round(sum(completion_ratios) / len(completion_ratios) * 100)}%"
    else:
        completion_value = "无相关数据"

    # 值守响应：已处理事件的平均响应时长；无已处理事件则无数据
    response_deltas = [
        (event.handled_at - event.detected_at).total_seconds()
        for event in events.filter(handled_at__isnull=False).only("handled_at", "detected_at").iterator(chunk_size=2000)
        if event.handled_at and event.detected_at and event.handled_at >= event.detected_at
    ]
    if response_deltas:
        response_value = f"{round(sum(response_deltas) / len(response_deltas) / 60)} 分钟"
    else:
        response_value = "无相关数据"

    online_robot_count = Robot.objects.filter(status="online").count()

    return {
        "updated_at": timezone.now(),
        "cards": [
            {
                "title": "累计预警",
                "value": f"{sum(item['value'] for item in alert_series)} 次",
                "note": "近 7 个统计周期内的异常提醒总量",
            },
            {
                "title": "检测识别",
                "value": f"{sum(item['value'] for item in detection_series)} 次",
                "note": "近 7 个统计周期内的识别抓拍总量",
            },
            {
                "title": "平均完成度",
                "value": completion_value,
                "note": "近 7 个统计周期内已完成任务执行的航点完成度均值",
            },
            {
                "title": "值守响应",
                "value": response_value,
                "note": f"当前在线设备 {online_robot_count} 台，均值取自近 7 个统计周期内已处理事件响应时长",
            },
        ],
        "trends": [
            serialize_trend("预警次数趋势", "观察异常波动，辅助值班优先级调整", "次", "#fb7b4d", alert_series),
            serialize_trend("检测次数趋势", "衡量视觉识别活跃度与场景复杂度", "次", "#2d8cff", detection_series),
            serialize_trend("巡检时长趋势", "追踪机器人投入时长与排班负荷", "分钟", "#19b97f", duration_series),
            serialize_trend("执行里程趋势", "按任务节奏估算巡检覆盖范围变化", "公里", "#7b6cff", mileage_series),
        ],
        "risk_weights": [
            {"label": date.strftime("%m-%d"), "value": risk_totals.get(date, 0)} for date in dates
        ],
    }


def ensure_demo_seed(*, force: bool = False) -> None:
    if not force and not settings.ENABLE_DEMO_SEED:
        return
    if not User.objects.filter(username="operator").exists():
        User.objects.create_user(
            username="operator",
            password="admin123456",
            first_name="值班员",
            last_name="A",
            email="operator@example.com",
        )

    robot, created = Robot.objects.get_or_create(
        code="ZSL-1A-07",
        defaults={
            "name": "南入口巡检机器人",
            "location": "太阳宫公园南入口",
            "area": "主通道南入口",
            "status": "online",
            "mode": "auto",
            "battery_level": 78,
            "network_strength": 92,
            "speaker_volume": 84,
            "patrol_duration_minutes": 30,
            "today_alerts": 12,
            "current_task_name": "公园主通道例行巡检",
            "firmware_version": "1.0.0",
            "camera_id": "front",
            "stream_id": "dog_ZSL-1A-07_front",
            "play_urls": {
                "flv": "https://39.107.250.69/live/dog_ZSL-1A-07_front.live.flv",
                "hls": "https://39.107.250.69/live/dog_ZSL-1A-07_front/hls.m3u8",
            },
        },
    )
    if created:
        robot.stream_id = "dog_ZSL-1A-07_front"
        robot.save(update_fields=["stream_id", "play_urls", "updated_at"])
    demo_map = MapData.objects.filter(name="太阳宫园区 V1", robot=robot).order_by("id").first()
    if demo_map is None:
        demo_map = MapData.objects.create(
            name="太阳宫园区 V1",
            robot=robot,
            resolution=0.05,
            description="演示预置地图，现场建图后可替换为真实地图",
        )
    demo_route = (
        PatrolRoute.objects.filter(
            name="南门-主步道-牡丹园-活动广场",
            robot=robot,
            map_data=demo_map,
        )
        .order_by("id")
        .first()
    )
    if demo_route is None:
        demo_route = PatrolRoute.objects.create(
            name="南门-主步道-牡丹园-活动广场",
            robot=robot,
            map_data=demo_map,
            waypoints=[
                {"x": 0.0, "y": 0.0, "yaw": 0.0, "name": "南门"},
                {"x": 2.0, "y": 1.0, "yaw": 0.0, "name": "主步道"},
                {"x": 4.0, "y": 2.0, "yaw": 0.0, "name": "牡丹园"},
                {"x": 6.0, "y": 3.0, "yaw": 0.0, "name": "活动广场"},
            ],
            waypoint_names=["南门", "主步道", "牡丹园", "活动广场"],
            description="演示预置路线，现场建图后应重新绑定航点",
        )
    task = PatrolTask.objects.filter(
        name="公园主通道早间巡检",
        robot=robot,
    ).order_by("id").first()
    created_task = False
    if task is None:
        task = PatrolTask.objects.create(
            name="公园主通道早间巡检",
            robot=robot,
            route_name=demo_route.name,
            scheduled_start=timezone.now() - timezone.timedelta(hours=2),
            scheduled_end=timezone.now() + timezone.timedelta(hours=1),
            status="running",
            completion_rate=68,
            route=demo_route,
        )
        created_task = True
    if not created_task and task.route_id is None:
        task.route = demo_route
        task.route_name = demo_route.name
        task.save(update_fields=["route", "route_name", "updated_at"])
    PatrolSchedule.objects.get_or_create(
        name="每日早间日常巡检",
        robot=robot,
        task_template=task,
        route=demo_route,
        map_data=demo_map,
        defaults={
            "schedule_type": "daily",
            "time_of_day": datetime_time(9, 0),
            "priority": 10,
            "enabled": True,
            "note": "演示预置计划：每天 09:00 执行日常巡检",
        },
    )


class LoginView(APIView):
    permission_classes = [permissions.AllowAny]

    def post(self, request):
        ensure_demo_seed()
        username = request.data.get("username", "")
        password = request.data.get("password", "")
        user = authenticate(username=username, password=password)
        if not user:
            return Response({"detail": "用户名或密码错误"}, status=status.HTTP_400_BAD_REQUEST)
        token, _ = Token.objects.get_or_create(user=user)
        return Response(
            {
                "token": token.key,
                "user": {
                    "username": user.username,
                    "display_name": f"{user.first_name}{user.last_name}".strip() or user.username,
                },
            }
        )


class LogoutView(APIView):
    def post(self, request):
        request.auth.delete()
        return Response({"detail": "已退出登录"})


class ProfileView(APIView):
    def get(self, request):
        user = request.user
        return Response(
            {
                "username": user.username,
                "display_name": f"{user.first_name}{user.last_name}".strip() or user.username,
            }
        )


class DashboardOverviewView(APIView):
    def get(self, request):
        ensure_demo_seed()
        robots = Robot.objects.all()
        events = InspectionEvent.objects.all()
        today = timezone.localdate()
        today_events = events.filter(detected_at__date=today)
        today_alert_count = today_events.count()
        latest_robot = robots.first()
        latest_event = events.first()
        # 今日巡检时长：取最新机器人当日遥测活跃时长（max-min(reported_at) 分钟），无则 0
        today_patrol_minutes = 0
        if latest_robot:
            today_telemetry = RobotTelemetry.objects.filter(
                robot=latest_robot, reported_at__date=today
            )
            span = today_telemetry.aggregate(
                low=Min("reported_at"), high=Max("reported_at")
            )
            if span["low"] and span["high"]:
                today_patrol_minutes = round((span["high"] - span["low"]).total_seconds() / 60, 1)
        return Response(
            {
                "summary": {
                    "online_robot_count": robots.filter(status="online").count(),
                    "today_alert_count": today_alert_count,
                    "pending_event_count": events.filter(status="pending").count(),
                    "resolved_event_count": events.filter(status="resolved").count(),
                    "completed_task_count": PatrolTask.objects.filter(status="completed").count(),
                },
                "header": {
                    "device_code": latest_robot.code if latest_robot else "--",
                    "current_mode": latest_robot.get_mode_display() if latest_robot else "--",
                    "current_location": latest_robot.location if latest_robot else "--",
                    "today_alerts": today_alert_count,
                    "today_patrol_minutes": today_patrol_minutes,
                },
                "live_event": EventSerializer(latest_event, context={"request": request}).data
                if latest_event
                else None,
                "latest_robot": RobotDetailSerializer(latest_robot, context={"request": request}).data
                if latest_robot
                else None,
                "event_distribution": list(
                    events.values("status").annotate(total=Count("id")).order_by("status")
                ),
            }
        )


class DashboardAnalyticsView(APIView):
    def get(self, request):
        return Response(build_analytics_payload())


class RobotListView(APIView):
    permission_classes = [permissions.AllowAny]

    def get(self, request):
        ensure_demo_seed()
        return Response(RobotSerializer(Robot.objects.all(), many=True).data)


class RobotDetailView(APIView):
    permission_classes = [permissions.AllowAny]

    def get(self, request, robot_id):
        ensure_demo_seed()
        robot = Robot.objects.get(id=robot_id)
        return Response(RobotDetailSerializer(robot, context={"request": request}).data)


def dispatch_robot_command(command: RobotCommand) -> tuple[dict, str]:
    endpoint = settings.ROBOT_CONTROL_ENDPOINTS.get(
        command.robot.code,
        settings.DEFAULT_ROBOT_CONTROL_ENDPOINT,
    )
    if not endpoint:
        return {}, "机器人未配置控制端点"

    payload = {
        "command_id": command.id,
        "robot_code": command.robot.code,
        "action": command.action,
        "payload": command.payload,
    }
    request = Request(
        endpoint,
        data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urlopen(request, timeout=3) as response:
            body = response.read().decode("utf-8")
            response_payload = json.loads(body) if body else {}
            return response_payload, ""
    except HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        return {}, f"板端返回 HTTP {exc.code}: {body}"
    except (URLError, TimeoutError, json.JSONDecodeError) as exc:
        return {}, str(exc)


class RobotCommandView(APIView):
    ACTION_TO_COMMAND_TYPE = {
        "takeover_enter": "teleop.takeover_enter",
        "takeover_exit": "teleop.takeover_exit",
        "stand_up": "teleop.stand_up",
        "lie_down": "teleop.lie_down",
        "shake_hand": "teleop.shake_hand",
        "two_leg_stand": "teleop.two_leg_stand",
        "speed_micro": "teleop.speed_micro",
        "speed_slow": "teleop.speed_slow",
        "speed_normal": "teleop.speed_normal",
        "speed_fast": "teleop.speed_fast",
        "move_forward": "teleop.move_forward",
        "move_backward": "teleop.move_backward",
        "move_left": "teleop.move_left",
        "move_right": "teleop.move_right",
        "turn_left": "teleop.turn_left",
        "turn_right": "teleop.turn_right",
        "move_velocity": "teleop.move_velocity",
        "move_stop": "teleop.move_stop",
        "passive": "teleop.passive",
        "skill": "teleop.skill",
        "skill_list": "teleop.skill_list",
        "skill_status": "teleop.skill_status",
        "skill_cancel": "teleop.skill_cancel",
        "person_follow_start": "teleop.person_follow_start",
        "person_follow_stop": "teleop.person_follow_stop",
        "person_follow_status": "teleop.person_follow_status",
        "charge_start": "charge.start",
        "charge_stop": "charge.stop",
        "motion_start": "motion.start",
        "motion_stop": "motion.stop",
        "audio_volume": "audio.volume",
    }

    def post(self, request, robot_id):
        ensure_demo_seed()
        robot = Robot.objects.get(id=robot_id)
        serializer = RobotCommandCreateSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        action = serializer.validated_data["action"]
        payload = serializer.validated_data.get("payload") or {}
        if action == "play_audio":
            audio_url = str(payload.get("audio_url", "")).strip()
            if not audio_url.startswith(("http://", "https://")):
                return Response(
                    {"detail": "audio_url 必须是机器狗可下载的 HTTP(S) 地址"},
                    status=status.HTTP_400_BAD_REQUEST,
                )
        command_type = self.ACTION_TO_COMMAND_TYPE.get(action)
        if command_type:
            if robot.effective_connection_status() != "online":
                return Response(
                    {"detail": "机器狗 Edge Agent 当前离线，无法远程控制。"},
                    status=status.HTTP_409_CONFLICT,
                )
            command = CommandService.create_robot_command(
                robot=robot,
                command_type=command_type,
                payload=payload,
                operator=request.user if request.user.is_authenticated else None,
                expiry_seconds=180 if action == "skill" else (30 if action in {"charge_start", "charge_stop", "motion_start", "motion_stop"} else 10),
            )
            return Response(RemoteCommandSerializer(command).data, status=status.HTTP_202_ACCEPTED)

        command = RobotCommand.objects.create(
            robot=robot,
            action=action,
            payload=payload,
        )

        if command.action == "play_audio":
            return Response(RobotCommandSerializer(command).data, status=status.HTTP_201_CREATED)

        response_payload, error_message = dispatch_robot_command(command)
        command.sent_at = timezone.now()
        command.response_payload = response_payload
        command.error_message = error_message
        command.status = "failed" if error_message else "sent"
        command.save(update_fields=["sent_at", "response_payload", "error_message", "status", "updated_at"])
        response_status = status.HTTP_201_CREATED if command.status == "sent" else status.HTTP_502_BAD_GATEWAY
        return Response(RobotCommandSerializer(command).data, status=response_status)


class RobotMcpTeleopView(APIView):
    """Authenticated HTTP API backing the public RoamerX MCP tools."""

    DIRECTION_ACTIONS = {
        "forward": "move_forward",
        "backward": "move_backward",
        "left": "move_left",
        "right": "move_right",
        "turn_left": "turn_left",
        "turn_right": "turn_right",
        "stop": "move_stop",
        "velocity": "move_velocity",
    }
    SPEED_ACTIONS = {
        "micro": "speed_micro",
        "low": "speed_slow",
        "medium": "speed_normal",
        "high": "speed_fast",
    }
    ACTIONS = {
        "stand_up": "stand_up",
        "prone": "lie_down",
        "passive": "passive",
        "motion_start": "motion_start",
        "motion_stop": "motion_stop",
    }
    MCP_VALUE_ALIASES = {
        "direction": {
            "前进": "forward", "前行": "forward", "后退": "backward", "后移": "backward",
            "左移": "left", "右移": "right", "左转": "turn_left", "向左转": "turn_left",
            "右转": "turn_right", "向右转": "turn_right", "停止": "stop", "停下": "stop",
            "速度": "velocity", "速度控制": "velocity",
        },
        "speed": {
            "微速": "micro", "微速档": "micro", "低速": "low", "低速档": "low",
            "中速": "medium", "中速档": "medium", "normal": "medium",
            "高速": "high", "高速档": "high",
        },
        "action": {
            "stand": "stand_up", "起立": "stand_up", "站立": "stand_up",
            "lie_down": "prone", "趴下": "prone", "匍匐": "prone",
            "damping": "passive", "阻尼": "passive", "软急停": "passive",
            "启动运控": "motion_start", "启动运动": "motion_start",
            "停止运控": "motion_stop", "停止运动": "motion_stop",
        },
    }
    PERSON_FOLLOW_COMMANDS = {
        "person-follow-start": "person_follow_start",
        "person-follow-stop": "person_follow_stop",
        "person-follow-status": "person_follow_status",
    }
    CENTER_TARGET = "center"
    PERSON_LABELS = {"person", "human", "人"}

    def post(self, request, robot_id, category):
        ensure_demo_seed()
        robot = get_object_or_404(Robot, id=robot_id)
        data = request.data if isinstance(request.data, dict) else {}
        if category == "skill-status":
            command_id = data.get("command_id")
            command = get_object_or_404(RemoteCommand, id=command_id, robot=robot, command_type="teleop.skill")
            return Response(RemoteCommandSerializer(command).data)
        if category == "person-detection-status":
            state = RobotPersonDetectionState.objects.filter(robot=robot).first()
            if state is None:
                return Response({"robot_id": robot.id, "available": False, "stale": True, "enabled": False, "detections": []})
            stale = _person_detection_is_stale(state)
            return Response({
                "robot_id": robot.id,
                "available": not stale,
                "stale": stale,
                "enabled": state.enabled,
                "frame_width": state.frame_width,
                "frame_height": state.frame_height,
                "detections": [] if stale else state.detections,
            })
        if category == "person-detection":
            enabled = data.get("enabled")
            if not isinstance(enabled, bool):
                return Response({"detail": "enabled 必须是布尔值"}, status=status.HTTP_400_BAD_REQUEST)
            state, _ = RobotPersonDetectionState.objects.get_or_create(robot=robot)
            state.enabled = enabled
            if not enabled:
                state.detections = []
                state.captured_at = timezone.now()
            state.save(update_fields=["enabled", "detections", "captured_at", "updated_at"])
            return Response({"robot_id": robot.id, "enabled": state.enabled})
        if category == "person-follow-start" and data.get("target"):
            target = str(data.get("target") or "").strip().lower()
            if target != self.CENTER_TARGET or str(data.get("track_id") or "").strip():
                return Response(
                    {"detail": "target 仅支持 center，且不能与 track_id 同时使用"},
                    status=status.HTTP_400_BAD_REQUEST,
                )
            track_id = self._center_person_track_id(robot)
            if not track_id:
                return Response(
                    {"detail": "没有可用于跟随的最新人员识别框"},
                    status=status.HTTP_409_CONFLICT,
                )
            data = {**data, "track_id": track_id}
        if robot.effective_connection_status() != "online":
            return Response({"detail": "机器狗 Edge Agent 当前离线，无法远程控制。"}, status=status.HTTP_409_CONFLICT)
        action, command = self._resolve_action(category, data)
        if not action:
            return Response(self._unsupported_parameter_response(category, data), status=status.HTTP_400_BAD_REQUEST)
        command_type = RobotCommandView.ACTION_TO_COMMAND_TYPE[action]
        remote = CommandService.create_robot_command(
            robot=robot,
            command_type=command_type,
            payload={"source": "mcp", **command},
            operator=request.user,
            expiry_seconds=180 if action == "skill" else 30,
        )
        return Response(RemoteCommandSerializer(remote).data, status=status.HTTP_202_ACCEPTED)

    def _resolve_action(self, category, data):
        if category == "direction":
            action = self.DIRECTION_ACTIONS.get(self._canonical_value("direction", data.get("direction")))
            return action, dict(data.get("command") or {})
        if category == "speed":
            return self.SPEED_ACTIONS.get(self._canonical_value("speed", data.get("level"))), {}
        if category == "action":
            return self.ACTIONS.get(self._canonical_value("action", data.get("action"))), {}
        if category == "skill":
            command = {key: data[key] for key in ("preset", "steps", "description") if key in data}
            return "skill", command
        if category == "skill-list":
            return "skill_list", {}
        if category == "skill-cancel":
            return "skill_cancel", {
                "command_id": data.get("command_id"),
            }
        follow_action = self.PERSON_FOLLOW_COMMANDS.get(category)
        if follow_action == "person_follow_start":
            track_id = str(data.get("track_id") or "").strip()
            return (follow_action, {"track_id": track_id}) if track_id else (None, {})
        if follow_action:
            return follow_action, {}
        return None, {}

    @classmethod
    def _canonical_value(cls, category, value):
        raw = str(value or "").strip().lower()
        return cls.MCP_VALUE_ALIASES.get(category, {}).get(raw, raw)

    @classmethod
    def _unsupported_parameter_response(cls, category, data):
        field, supported = {
            "direction": ("direction", sorted(cls.DIRECTION_ACTIONS)),
            "speed": ("level", sorted(cls.SPEED_ACTIONS)),
            "action": ("action", sorted(cls.ACTIONS)),
            "person-follow-start": ("track_id", ["non-empty track_id", "target=center"]),
        }.get(category, ("category", sorted({"direction", "speed", "action", "skill", "skill-list", "skill-cancel", "person-follow-start", "person-follow-stop", "person-follow-status"})))
        return {
            "detail": "不支持的 MCP 工具参数",
            "category": category,
            "parameter": field,
            "received": data.get(field),
            "supported_values": supported,
        }

    @classmethod
    def _center_person_track_id(cls, robot) -> str:
        """Choose the fresh person box closest to the camera-frame center.

        The cloud only resolves the target once.  The Edge Agent follows the
        resulting stable ``track_id`` from its local, high-rate snapshot.
        """
        state = RobotPersonDetectionState.objects.filter(robot=robot, enabled=True).first()
        if state is None or state.frame_width <= 0 or state.frame_height <= 0:
            return ""
        if _person_detection_is_stale(state):
            return ""

        frame_center_x = state.frame_width / 2
        frame_center_y = state.frame_height / 2
        candidates: list[tuple[float, float, float, str]] = []
        for detection in state.detections:
            if not isinstance(detection, dict):
                continue
            if str(detection.get("label") or "").strip().lower() not in cls.PERSON_LABELS:
                continue
            track_id = str(detection.get("track_id") or "").strip()
            bbox = detection.get("bbox")
            if not track_id or not isinstance(bbox, dict):
                continue
            try:
                x = float(bbox.get("x"))
                y = float(bbox.get("y"))
                width = float(bbox.get("width"))
                height = float(bbox.get("height"))
                confidence = float(detection.get("confidence") or 0)
            except (TypeError, ValueError):
                continue
            if width <= 0 or height <= 0:
                continue
            center_distance_squared = (x + width / 2 - frame_center_x) ** 2 + (
                y + height / 2 - frame_center_y
            ) ** 2
            # Stable tie breaks favour a higher-confidence, larger person box.
            candidates.append((center_distance_squared, -confidence, -(width * height), track_id))
        return min(candidates)[-1] if candidates else ""


class RobotChargingDockView(APIView):
    """Persist a robot's dock route and dispatch the two-stage docking task."""

    permission_classes = [permissions.AllowAny]

    def post(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        map_id = request.data.get("map_id") or robot.charging_map_id
        route_id = request.data.get("route_id") or robot.charging_route_id
        if not map_id or not route_id:
            return Response({"detail": "请先选择充电地图和两点回充路线"}, status=status.HTTP_400_BAD_REQUEST)
        map_data = get_object_or_404(MapData, pk=map_id)
        route = get_object_or_404(PatrolRoute, pk=route_id, robot=robot, map_data=map_data)
        if len(route.waypoints or []) != 2:
            return Response({"detail": "回充路线必须固定为两个点"}, status=status.HTTP_400_BAD_REQUEST)
        if robot.effective_connection_status() != "online":
            return Response({"detail": "机器狗 Edge Agent 当前离线"}, status=status.HTTP_409_CONFLICT)
        if robot.localization_status != "normal" or not robot.nav_ready:
            return Response({"detail": "定位或导航栈未就绪，不能开始回充"}, status=status.HTTP_409_CONFLICT)

        with transaction.atomic():
            robot = Robot.objects.select_for_update().get(pk=robot.pk)
            robot.charging_map = map_data
            robot.charging_route = route
            robot.save(update_fields=["charging_map", "charging_route", "updated_at"])
            now = timezone.now()
            task_name = f"一键回充 - {route.name}"
            task, _ = PatrolTask.objects.get_or_create(
                robot=robot,
                route=route,
                name=task_name,
                defaults={
                    "route_name": route.name,
                    "scheduled_start": now,
                    "scheduled_end": now + timedelta(hours=8),
                    "enabled": True,
                    "description": "机器人管理页面一键回充专用两点路线",
                    "created_by": request.user if request.user.is_authenticated else None,
                },
            )
            task.route_name = route.name
            task.scheduled_start = now
            task.scheduled_end = now + timedelta(hours=8)
            task.enabled = True
            task.save(update_fields=["route_name", "scheduled_start", "scheduled_end", "enabled", "updated_at"])
            try:
                # Creating an execution and its start command is one operation.
                # CommandService can reject the dispatch (for example, low
                # battery); do not commit a created execution with no command,
                # because it blocks every retry as ROBOT_BUSY.
                with transaction.atomic():
                    execution = TaskExecutionService.create_execution(
                        task, request.user if request.user.is_authenticated else None
                    )
                    command = CommandService.create(
                        execution,
                        "task.start",
                        request.user if request.user.is_authenticated else None,
                        command_options={
                            "docking": {
                                "enabled": True,
                                "final_waypoint_index": 1,
                                "charge_retries": 3,
                                "undock_seconds": 3,
                            }
                        },
                    )
            except TaskStateError as exc:
                return Response({"detail": str(exc)}, status=status.HTTP_409_CONFLICT)
        return Response(
            {
                "charging_config": {"map_id": map_data.id, "map_name": map_data.name, "route_id": route.id, "route_name": route.name},
                "execution": TaskExecutionSerializer(execution).data,
                "command": RemoteCommandSerializer(command).data,
            },
            status=status.HTTP_201_CREATED,
        )


class RobotRemoteCommandDetailView(APIView):
    def get(self, request, robot_id, command_id):
        command = get_object_or_404(
            RemoteCommand.objects.prefetch_related("events"),
            id=command_id,
            robot_id=robot_id,
        )
        return Response(RemoteCommandSerializer(command).data)


class RobotAudioRecordingCommandView(APIView):
    MAX_AUDIO_SIZE = 10 * 1024 * 1024
    EXTENSION_BY_TYPE = {
        "audio/webm": ".webm",
        "audio/ogg": ".ogg",
        "audio/mpeg": ".mp3",
        "audio/mp3": ".mp3",
        "audio/wav": ".wav",
        "audio/x-wav": ".wav",
        "audio/mp4": ".m4a",
        "audio/aac": ".aac",
    }
    ALLOWED_EXTENSIONS = {".webm", ".ogg", ".mp3", ".wav", ".m4a", ".aac"}

    def post(self, request, robot_id):
        ensure_demo_seed()
        robot = get_object_or_404(Robot, id=robot_id)
        uploaded_file = request.FILES.get("file")
        if uploaded_file is None:
            return Response({"detail": "录音文件不能为空"}, status=status.HTTP_400_BAD_REQUEST)
        if uploaded_file.size <= 0:
            return Response({"detail": "录音文件为空"}, status=status.HTTP_400_BAD_REQUEST)
        if uploaded_file.size > self.MAX_AUDIO_SIZE:
            return Response({"detail": "录音文件不能超过 10MB"}, status=status.HTTP_400_BAD_REQUEST)

        content_type = (uploaded_file.content_type or "").split(";", 1)[0].lower()
        extension = self.EXTENSION_BY_TYPE.get(content_type)
        if extension is None:
            original_name = uploaded_file.name or ""
            extension = "." + original_name.rsplit(".", 1)[-1].lower() if "." in original_name else ".webm"
        if extension not in self.ALLOWED_EXTENSIONS:
            return Response({"detail": "不支持的录音格式"}, status=status.HTTP_400_BAD_REQUEST)

        title = str(request.data.get("title") or request.data.get("audio_name") or "现场录音").strip()
        if not title:
            return Response({"detail": "录音标题不能为空"}, status=status.HTTP_400_BAD_REQUEST)
        if len(title) > 128:
            return Response({"detail": "录音标题不能超过 128 个字符"}, status=status.HTTP_400_BAD_REQUEST)

        category = None
        category_id = request.data.get("category")
        if category_id not in (None, "", "null"):
            category = get_object_or_404(SpeechCategory, pk=category_id)

        uploaded_file.name = f"recording{extension}"
        recording = RecordedAudio.objects.create(
            title=title,
            category=category,
            file=uploaded_file,
            content_type=content_type,
            file_size=uploaded_file.size,
            created_by=request.user if request.user.is_authenticated else None,
        )
        asr_service.process_recording(recording)
        recording.refresh_from_db()
        recording_data = RecordedAudioSerializer(recording, context={"request": request}).data
        audio_url = recording_data["audio_url"]
        play_now = str(request.data.get("play_now", "true")).strip().lower() not in {"0", "false", "no", "off"}
        command = None
        if play_now:
            command = RobotCommand.objects.create(
                robot=robot,
                action="play_audio",
                payload={
                    "audio_url": audio_url,
                    "audio_name": title,
                    "source": "dashboard_recording",
                    "recording_id": recording.id,
                    "content_type": recording.content_type,
                    "file_size": recording.file_size,
                },
            )
        return Response(
            {
                "audio_url": audio_url,
                "recording": recording_data,
                "command": RobotCommandSerializer(command).data if command else None,
            },
            status=status.HTTP_201_CREATED,
        )


class RobotStreamAudioCommandView(APIView):
    def post(self, request, robot_id):
        robot = get_object_or_404(Robot, id=robot_id)
        enabled = request.data.get("enabled")
        if not isinstance(enabled, bool):
            return Response({"detail": "enabled 必须为布尔值"}, status=status.HTTP_400_BAD_REQUEST)
        command = RobotCommand.objects.create(
            robot=robot,
            action="set_stream_audio",
            payload={"enabled": enabled, "source": "guard_live_audio"},
        )
        return Response(RobotCommandSerializer(command).data, status=status.HTTP_201_CREATED)


class RobotStreamAudioCommandDetailView(APIView):
    def get(self, request, robot_id, command_id):
        command = get_object_or_404(
            RobotCommand,
            id=command_id,
            robot_id=robot_id,
            action="set_stream_audio",
        )
        return Response(RobotCommandSerializer(command).data)


class RecordedAudioListView(APIView):
    def get(self, request):
        recordings = RecordedAudio.objects.select_related("category").all()
        return Response(RecordedAudioSerializer(recordings, many=True, context={"request": request}).data)


class RecordedAudioDetailView(APIView):
    def delete(self, request, pk):
        recording = get_object_or_404(RecordedAudio, pk=pk)
        stored_file = recording.file
        recording.delete()
        if stored_file:
            stored_file.delete(save=False)
        return Response(status=status.HTTP_204_NO_CONTENT)


class RobotRecordedAudioPlayView(APIView):
    def post(self, request, robot_id, recording_id):
        robot = get_object_or_404(Robot, id=robot_id)
        recording = get_object_or_404(RecordedAudio, id=recording_id)
        recording_data = RecordedAudioSerializer(recording, context={"request": request}).data
        command = RobotCommand.objects.create(
            robot=robot,
            action="play_audio",
            payload={
                "audio_url": recording_data["audio_url"],
                "audio_name": recording.title,
                "source": "dashboard_recording_library",
                "recording_id": recording.id,
                "content_type": recording.content_type,
                "file_size": recording.file_size,
            },
        )
        return Response(RobotCommandSerializer(command).data, status=status.HTTP_201_CREATED)


class SpeechTemplateListCreateView(APIView):
    def get(self, request):
        return Response(SpeechTemplateSerializer(SpeechTemplate.objects.all(), many=True).data)

    def post(self, request):
        serializer = SpeechTemplateSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        template = serializer.save(created_by=request.user if request.user.is_authenticated else None)
        return Response(SpeechTemplateSerializer(template).data, status=status.HTTP_201_CREATED)


class SpeechTemplateDetailView(APIView):
    def patch(self, request, pk):
        template = get_object_or_404(SpeechTemplate, pk=pk)
        serializer = SpeechTemplateSerializer(template, data=request.data, partial=True)
        serializer.is_valid(raise_exception=True)
        serializer.save()
        return Response(serializer.data)

    def delete(self, request, pk):
        get_object_or_404(SpeechTemplate, pk=pk).delete()
        return Response(status=status.HTTP_204_NO_CONTENT)


class AlertSkillBindingListView(APIView):
    def get(self, request):
        bindings = AlertSkillBinding.objects.select_related("template", "template__category")
        return Response(AlertSkillBindingSerializer(bindings, many=True).data)


class AlertSkillBindingDetailView(APIView):
    def patch(self, request, skill_key):
        binding = get_object_or_404(AlertSkillBinding, skill_key=skill_key)
        serializer = AlertSkillBindingSerializer(binding, data=request.data, partial=True)
        serializer.is_valid(raise_exception=True)
        serializer.save()
        return Response(serializer.data)


class AlertSkillPreviewView(APIView):
    def post(self, request, skill_key):
        serializer = AlertSkillPreviewSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        binding = get_object_or_404(
            AlertSkillBinding.objects.select_related("template"),
            skill_key=skill_key,
        )
        if binding.template is None:
            return Response({"detail": "请先为告警技能绑定播报模板"}, status=status.HTTP_400_BAD_REQUEST)

        robot = get_object_or_404(Robot, id=serializer.validated_data["robot_id"])
        if robot.connection_status != "online":
            return Response({"detail": "机器人离线，无法试播"}, status=status.HTTP_409_CONFLICT)

        try:
            saved_path, cache_hit = tts_service.synthesize_speech(binding.template.text)
        except IntegrityError:
            LOGGER.exception("alert skill preview synthesis failed skill=%s robot=%s", skill_key, robot.code)
            return Response({"detail": "试播语音生成失败，请稍后重试"}, status=status.HTTP_502_BAD_GATEWAY)

        command = RobotCommand.objects.create(
            robot=robot,
            action="play_audio",
            payload={
                "audio_url": build_public_media_url(request, saved_path),
                "audio_name": binding.template.name,
                "text": binding.template.text,
                "source": "dashboard_alert_skill_preview",
                "alert_skill": binding.skill_key,
                "dual_output": True,
                "content_type": "audio/mpeg",
                "tts_cache_hit": cache_hit,
                "preview": True,
            },
        )
        return Response(
            {
                "skill": AlertSkillBindingSerializer(binding).data,
                "command": RobotCommandSerializer(command).data,
            },
            status=status.HTTP_201_CREATED,
        )


class SpeechCategoryListCreateView(APIView):
    def get(self, request):
        return Response(SpeechCategorySerializer(SpeechCategory.objects.all(), many=True).data)

    def post(self, request):
        serializer = SpeechCategorySerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        category = serializer.save()
        return Response(SpeechCategorySerializer(category).data, status=status.HTTP_201_CREATED)


class SpeechCategoryDetailView(APIView):
    def patch(self, request, pk):
        category = get_object_or_404(SpeechCategory, pk=pk)
        serializer = SpeechCategorySerializer(category, data=request.data, partial=True)
        serializer.is_valid(raise_exception=True)
        serializer.save()
        return Response(serializer.data)

    def delete(self, request, pk):
        get_object_or_404(SpeechCategory, pk=pk).delete()
        return Response(status=status.HTTP_204_NO_CONTENT)


class SpeechSynthesisView(APIView):
    def post(self, request):
        serializer = SpeechSynthesisSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        text = serializer.validated_data["text"]
        try:
            saved_path, cache_hit = tts_service.synthesize_speech(text)
        except Exception:
            LOGGER.exception("speech synthesis failed")
            return Response({"detail": "语音生成失败，请稍后重试"}, status=status.HTTP_502_BAD_GATEWAY)
        return Response({"audio_url": build_public_media_url(request, saved_path), "cache_hit": cache_hit})


class RobotTTSCommandView(APIView):
    def post(self, request, robot_id):
        robot = get_object_or_404(Robot, id=robot_id)
        serializer = SpeechSynthesisSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        text = serializer.validated_data["text"]
        audio_name = serializer.validated_data.get("audio_name") or "实时文字喊话"
        try:
            saved_path, cache_hit = tts_service.synthesize_speech(text)
        except Exception:
            LOGGER.exception("robot speech synthesis failed robot=%s", robot.code)
            return Response({"detail": "语音生成失败，请稍后重试"}, status=status.HTTP_502_BAD_GATEWAY)

        audio_url = build_public_media_url(request, saved_path)
        command = RobotCommand.objects.create(
            robot=robot,
            action="play_audio",
            payload={
                "audio_url": audio_url,
                "audio_name": audio_name,
                "text": text,
                "source": "dashboard_tts",
                "output_target": "nx",
                "content_type": "audio/mpeg",
                "tts_cache_hit": cache_hit,
            },
        )
        return Response(
            {"audio_url": audio_url, "cache_hit": cache_hit, "command": RobotCommandSerializer(command).data},
            status=status.HTTP_201_CREATED,
        )


class DeviceCommandPollView(APIView):
    permission_classes = [IsAudioDeviceCredential]

    def get(self, request):
        requested_action = str(request.query_params.get("action") or "").strip()
        if requested_action and requested_action not in {"play_audio", "set_stream_audio"}:
            return Response({"detail": "unsupported command action"}, status=status.HTTP_400_BAD_REQUEST)
        requested_code = (
            request.query_params.get("robot_code")
            or request.headers.get("X-Device-Code")
            or ""
        ).strip()
        credential_robot = getattr(request, "device_robot", None)
        if credential_robot and requested_code and credential_robot.code != requested_code:
            return Response({"detail": "设备凭证与 robot_code 不匹配"}, status=status.HTTP_403_FORBIDDEN)
        robot = credential_robot
        if robot is None:
            if not requested_code:
                return Response({"detail": "robot_code is required"}, status=status.HTTP_400_BAD_REQUEST)
            robot = get_object_or_404(Robot, code=requested_code)

        with transaction.atomic():
            command = None
            if requested_action != "play_audio":
                command = (
                    RobotCommand.objects.select_for_update(skip_locked=True)
                    .filter(robot=robot, action="set_stream_audio", status="queued")
                    .order_by("-created_at")
                    .first()
                )
            if command is None and requested_action != "set_stream_audio":
                command = (
                    RobotCommand.objects.select_for_update(skip_locked=True)
                    .filter(robot=robot, action="play_audio", status="queued")
                    .order_by("-created_at")
                    .first()
                )
            if command is None:
                return Response(status=status.HTTP_204_NO_CONTENT)
            RobotCommand.objects.filter(
                robot=robot,
                action=command.action,
                status="queued",
                created_at__lt=command.created_at,
            ).update(
                status="superseded",
                error_message="已被更新的播报替换",
                updated_at=timezone.now(),
            )
            command.status = "sent"
            command.sent_at = timezone.now()
            command.save(update_fields=["status", "sent_at", "updated_at"])
        return Response(RobotCommandSerializer(command).data)


class DeviceCommandReportView(APIView):
    permission_classes = [IsAudioDeviceCredential]

    def post(self, request, command_id):
        status_value = str(request.data.get("status", "")).strip()
        if status_value not in {"running", "finished", "failed", "superseded"}:
            return Response(
                {"detail": "status must be running, finished, failed, or superseded"},
                status=status.HTTP_400_BAD_REQUEST,
            )
        command = get_object_or_404(
            RobotCommand.objects.select_related("robot"),
            id=command_id,
            action__in=["play_audio", "set_stream_audio"],
        )
        credential_robot = getattr(request, "device_robot", None)
        if credential_robot and credential_robot.id != command.robot_id:
            return Response({"detail": "设备凭证与命令所属机器人不匹配"}, status=status.HTTP_403_FORBIDDEN)

        response_payload = request.data.get("response_payload") or {}
        if not isinstance(response_payload, dict):
            return Response({"detail": "response_payload must be an object"}, status=status.HTTP_400_BAD_REQUEST)
        command.status = status_value
        command.response_payload = response_payload
        command.error_message = str(request.data.get("error_message") or "")
        command.save(update_fields=["status", "response_payload", "error_message", "updated_at"])
        return Response(RobotCommandSerializer(command).data)


class EventListView(APIView):
    def get(self, request):
        ensure_demo_seed()
        queryset = InspectionEvent.objects.select_related("robot").all()
        status_value = request.query_params.get("status")
        if status_value:
            queryset = queryset.filter(status=status_value)
        detected_from = request.query_params.get("detected_from", "").strip()
        detected_to = request.query_params.get("detected_to", "").strip()
        if detected_from:
            detected_from_value = parse_datetime(detected_from)
            if not detected_from_value:
                return Response({"detail": "开始时间参数无效"}, status=status.HTTP_400_BAD_REQUEST)
            if timezone.is_naive(detected_from_value):
                detected_from_value = timezone.make_aware(detected_from_value, timezone.get_current_timezone())
            queryset = queryset.filter(detected_at__gte=detected_from_value)
        if detected_to:
            detected_to_value = parse_datetime(detected_to)
            if not detected_to_value:
                return Response({"detail": "结束时间参数无效"}, status=status.HTTP_400_BAD_REQUEST)
            if timezone.is_naive(detected_to_value):
                detected_to_value = timezone.make_aware(detected_to_value, timezone.get_current_timezone())
            queryset = queryset.filter(detected_at__lte=detected_to_value)
        search_value = request.query_params.get("search", "").strip()
        if search_value:
            queryset = queryset.filter(
                Q(title__icontains=search_value)
                | Q(location__icontains=search_value)
                | Q(event_type__icontains=search_value)
                | Q(description__icontains=search_value)
                | Q(handling_notes__icontains=search_value)
                | Q(robot__name__icontains=search_value)
                | Q(robot__code__icontains=search_value)
            )
        ordering_value = request.query_params.get("ordering", "detected_desc")
        if ordering_value == "risk_desc":
            queryset = queryset.annotate(
                risk_rank=Case(
                    When(risk_level="high", then=3),
                    When(risk_level="medium", then=2),
                    When(risk_level="low", then=1),
                    default=0,
                    output_field=IntegerField(),
                )
            ).order_by("-risk_rank", "-detected_at")
        else:
            ordering_map = {
                "detected_desc": "-detected_at",
                "detected_asc": "detected_at",
                "confidence_desc": "-confidence",
            }
            ordering = ordering_map.get(ordering_value, "-detected_at")
            queryset = queryset.order_by(ordering, "-detected_at")
        try:
            page = max(int(request.query_params.get("page", 1)), 1)
            page_size = min(max(int(request.query_params.get("page_size", 10)), 1), 50)
        except ValueError:
            return Response({"detail": "分页参数无效"}, status=status.HTTP_400_BAD_REQUEST)

        total = queryset.count()
        start = (page - 1) * page_size
        end = start + page_size
        results = EventSerializer(queryset[start:end], many=True, context={"request": request}).data
        return Response(
            {
                "count": total,
                "page": page,
                "page_size": page_size,
                "has_next": end < total,
                "results": results,
            }
        )


def event_stream(request):
    token = request.GET.get("token", "")
    if not token or not Token.objects.filter(key=token).exists():
        return HttpResponseForbidden("invalid token")

    subscriber = event_broker.subscribe()

    def stream():
        try:
            yield from sse_stream(subscriber)
        finally:
            event_broker.unsubscribe(subscriber)

    response = StreamingHttpResponse(stream(), content_type="text/event-stream")
    response["Cache-Control"] = "no-cache"
    response["X-Accel-Buffering"] = "no"
    return response


class EventDetailView(APIView):
    def get(self, request, event_id):
        ensure_demo_seed()
        event = InspectionEvent.objects.select_related("robot").get(id=event_id)
        return Response(EventSerializer(event, context={"request": request}).data)


class EventHandleView(APIView):
    def post(self, request, event_id):
        ensure_demo_seed()
        event = InspectionEvent.objects.get(id=event_id)
        next_status = request.data.get("status", event.status)
        notes = request.data.get("handling_notes", "")
        review_result = request.data.get("review_result", event.review_result)
        if next_status not in dict(InspectionEvent.STATUS_CHOICES):
            return Response({"detail": "事件状态无效"}, status=status.HTTP_400_BAD_REQUEST)
        if review_result not in dict(InspectionEvent.REVIEW_RESULT_CHOICES):
            return Response({"detail": "复核结论无效"}, status=status.HTTP_400_BAD_REQUEST)
        event.status = next_status
        event.handling_notes = notes
        event.review_result = review_result
        event.handled_by = request.user
        event.handled_at = timezone.now()
        event.save(
            update_fields=[
                "status",
                "handling_notes",
                "review_result",
                "handled_by",
                "handled_at",
                "updated_at",
            ]
        )
        return Response(EventSerializer(event, context={"request": request}).data)


class TaskListView(APIView):
    def get(self, request):
        ensure_demo_seed()
        return Response(PatrolTaskSerializer(PatrolTask.objects.select_related("robot").all(), many=True).data)


class TelemetryIngestView(APIView):
    permission_classes = [IsAuthenticatedOrDeviceCredential]

    def post(self, request):
        ensure_demo_seed()
        serializer = TelemetryIngestSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        payload = serializer.validated_data
        if hasattr(request, "device_robot") and request.device_robot.code != payload["robot_code"]:
            return Response({"detail": "设备凭证与 robot_code 不匹配"}, status=status.HTTP_403_FORBIDDEN)
        existing_telemetry = RobotTelemetry.objects.filter(sequence_id=payload["sequence_id"]).first()
        if existing_telemetry:
            return Response(
                {
                    "detail": "重复上报已忽略",
                    "robot_id": existing_telemetry.robot_id,
                    "telemetry_id": existing_telemetry.id,
                    "duplicate": True,
                },
                status=status.HTTP_200_OK,
            )

        video = payload.get("video") or {}
        camera_id = video.get("camera_id", "front")
        stream_id = video.get("stream_id") or f"dog_{payload['robot_code']}_{camera_id}"
        robot, _ = Robot.objects.get_or_create(
            code=payload["robot_code"],
            defaults={
                "name": payload.get("robot_name") or payload["robot_code"],
                "location": payload["position"]["name"],
                "area": payload["position"]["name"],
                "camera_id": camera_id,
                "stream_id": stream_id,
            },
        )

        if payload.get("robot_name"):
            robot.name = payload["robot_name"]
        robot.location = payload["position"]["name"]
        robot.area = payload["position"]["name"]
        robot.battery_level = payload["power"]["battery_level"]
        robot.network_strength = payload["network"]["signal_strength"]
        robot.mode = payload["runtime"]["mode"]
        robot.status = payload["runtime"]["status"]
        robot.last_heartbeat_at = payload["reported_at"]
        robot.camera_id = camera_id
        robot.stream_id = stream_id
        robot.play_urls = video.get("play_urls") or robot.play_urls
        robot.today_alerts += len(payload.get("detections", []))
        robot.save()

        telemetry = RobotTelemetry.objects.create(
            robot=robot,
            sequence_id=payload["sequence_id"],
            position_name=payload["position"]["name"],
            latitude=payload["position"].get("latitude"),
            longitude=payload["position"].get("longitude"),
            heading=payload["motion"].get("heading"),
            speed=payload["motion"].get("speed"),
            battery_level=payload["power"]["battery_level"],
            network_strength=payload["network"]["signal_strength"],
            video=video,
            raw_payload=request.data,
            reported_at=payload["reported_at"],
        )

        frame_width = video.get("frame_width")
        frame_height = video.get("frame_height")
        queued_audio_command_ids = []
        for detection in payload.get("detections", []):
            bbox = detection.get("bbox") or {}
            event = InspectionEvent.objects.create(
                robot=robot,
                title=detection.get("label") or detection.get("type") or "AI识别事件",
                event_type=detection.get("type", "generic_detection"),
                location=payload["position"]["name"],
                detected_at=detection.get("event_time") or payload["reported_at"],
                confidence=round(float(detection.get("confidence", 0)) * 100, 2)
                if float(detection.get("confidence", 0)) <= 1
                else detection.get("confidence", 0),
                risk_level=detection.get("risk_level", "medium"),
                status="pending",
                snapshot_url=detection.get("snapshot_url", ""),
                description=f"板端识别上报: {detection.get('label') or detection.get('type')}",
                camera_id=detection.get("camera_id") or camera_id,
                stream_id=detection.get("stream_id") or stream_id,
                object_class=detection.get("object_class", ""),
                track_id=detection.get("track_id", ""),
                bbox_x=bbox.get("x"),
                bbox_y=bbox.get("y"),
                bbox_width=bbox.get("width"),
                bbox_height=bbox.get("height"),
                frame_width=frame_width,
                frame_height=frame_height,
                raw_detection=detection,
            )
            event_broker.publish(
                "inspection_event_created",
                {
                    "event": EventSerializer(event).data,
                    "robot": {
                        "id": robot.id,
                        "code": robot.code,
                        "name": robot.name,
                    },
                },
            )
            audio_command = queue_bicycle_departure_speech(request, robot, detection, event)
            if audio_command:
                queued_audio_command_ids.append(audio_command.id)

        return Response(
            {
                "detail": "上报成功",
                "robot_id": robot.id,
                "telemetry_id": telemetry.id,
                "duplicate": False,
                "audio_commands_queued": queued_audio_command_ids,
            },
            status=status.HTTP_201_CREATED,
        )


class DevicePersonDetectionView(APIView):
    permission_classes = [IsAuthenticatedOrDeviceCredential]

    def get(self, request):
        robot_code = str(request.query_params.get("robot_code") or "").strip()
        robot = get_object_or_404(Robot, code=robot_code)
        if hasattr(request, "device_robot") and request.device_robot.id != robot.id:
            return Response({"detail": "设备凭证与 robot_code 不匹配"}, status=status.HTTP_403_FORBIDDEN)
        state = RobotPersonDetectionState.objects.filter(robot=robot).first()
        return Response({"robot_code": robot.code, "enabled": bool(state and state.enabled)})

    def post(self, request):
        serializer = PersonDetectionIngestSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        payload = serializer.validated_data
        robot = get_object_or_404(Robot, code=payload["robot_code"])
        if hasattr(request, "device_robot") and request.device_robot.id != robot.id:
            return Response({"detail": "设备凭证与 robot_code 不匹配"}, status=status.HTTP_403_FORBIDDEN)
        fields = {
            "camera_id": payload["camera_id"],
            "frame_width": payload["frame_width"],
            "frame_height": payload["frame_height"],
            "captured_at": payload["captured_at"],
            "detections": payload["detections"],
            "updated_at": timezone.now(),
        }
        # SQLite does not provide row-level locks.  A conditional UPDATE keeps
        # the newest source frame and avoids holding a transaction while the
        # remote-control page polls at high frequency.
        if RobotPersonDetectionState.objects.filter(
            robot=robot,
            captured_at__lte=payload["captured_at"],
        ).update(**fields):
            return Response(
                {"detail": "实时检测帧已更新", "count": len(payload["detections"]), "accepted": True},
                status=status.HTTP_200_OK,
            )

        if RobotPersonDetectionState.objects.filter(robot=robot).exists():
            return Response(
                {
                    "detail": "已忽略乱序的旧检测帧",
                    "count": 0,
                    "accepted": False,
                },
                status=status.HTTP_200_OK,
            )

        try:
            RobotPersonDetectionState.objects.create(robot=robot, **fields)
        except Exception:
            # A concurrent first frame may have created the state.  Retry the
            # same conditional update, which still rejects an older frame.
            if not RobotPersonDetectionState.objects.filter(
                robot=robot,
                captured_at__lte=payload["captured_at"],
            ).update(**fields):
                return Response(
                    {"detail": "已忽略乱序的旧检测帧", "count": 0, "accepted": False},
                    status=status.HTTP_200_OK,
                )
        return Response(
            {"detail": "实时检测帧已更新", "count": len(payload["detections"]), "accepted": True},
            status=status.HTTP_200_OK,
        )


class RobotPersonDetectionView(APIView):
    def get(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        state = RobotPersonDetectionState.objects.filter(robot=robot).first()
        if state is None:
            return Response(
                {
                    "robot_id": robot.id,
                    "camera_id": robot.camera_id,
                    "available": False,
                    "stale": True,
                    "enabled": False,
                    "detections": [],
                }
            )
        stale = _person_detection_is_stale(state)
        return Response(
            {
                "robot_id": robot.id,
                "camera_id": state.camera_id,
                "available": not stale,
                "stale": stale,
                "enabled": state.enabled,
                "frame_width": state.frame_width,
                "frame_height": state.frame_height,
                "captured_at": state.captured_at,
                "received_at": state.updated_at,
                "detections": [] if stale else state.detections,
            }
        )

    def post(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        enabled = request.data.get("enabled")
        if not isinstance(enabled, bool):
            return Response({"detail": "enabled 必须是布尔值"}, status=status.HTTP_400_BAD_REQUEST)
        state, _ = RobotPersonDetectionState.objects.get_or_create(robot=robot)
        state.enabled = enabled
        if not enabled:
            state.detections = []
            state.captured_at = timezone.now()
        state.save(update_fields=["enabled", "detections", "captured_at", "updated_at"])
        return Response({"robot_id": robot.id, "enabled": state.enabled})


class MediaUploadView(APIView):
    permission_classes = [IsAuthenticatedOrDeviceCredential]

    def post(self, request):
        serializer = MediaUploadSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        payload = serializer.validated_data
        if hasattr(request, "device_robot") and request.device_robot.code != payload["robot_code"]:
            return Response({"detail": "设备凭证与 robot_code 不匹配"}, status=status.HTTP_403_FORBIDDEN)
        robot, _ = Robot.objects.get_or_create(
            code=payload["robot_code"],
            defaults={
                "name": payload["robot_code"],
                "location": "未知区域",
                "area": "未知区域",
            },
        )

        uploaded_file = payload["file"]
        expected_sha256 = payload.get("sha256") or ""
        if expected_sha256:
            digest = hashlib.sha256()
            for chunk in uploaded_file.chunks():
                digest.update(chunk)
            uploaded_file.seek(0)
            actual_sha256 = digest.hexdigest()
            if actual_sha256.lower() != expected_sha256.lower():
                return Response({"detail": "文件 sha256 校验失败"}, status=status.HTTP_400_BAD_REQUEST)
        else:
            actual_sha256 = ""

        asset = MediaAsset.objects.create(
            robot=robot,
            media_type=payload["media_type"],
            camera_id=payload.get("camera_id", ""),
            sequence_id=payload.get("sequence_id", ""),
            event_time=payload.get("event_time"),
            file=uploaded_file,
            sha256=actual_sha256,
            file_size=uploaded_file.size,
            event_id=payload.get("event_id"),
            task_execution_id=payload.get("task_execution_id"),
            content_type=getattr(uploaded_file, "content_type", "") or "",
        )
        media_url = settings.MEDIA_URL if settings.MEDIA_URL.startswith("/") else f"/{settings.MEDIA_URL}"
        media_path = f"{media_url.rstrip('/')}/{asset.file.name}"
        public_base_url = getattr(settings, "PUBLIC_BASE_URL", "")
        asset.url = f"{public_base_url}{media_path}" if public_base_url else request.build_absolute_uri(media_path)
        asset.save(update_fields=["url", "updated_at"])
        return Response(
            {"media_id": str(asset.media_id), "url": asset.url, "asset": MediaAssetSerializer(asset).data},
            status=status.HTTP_201_CREATED,
        )


@api_view(["GET"])
@permission_classes([permissions.AllowAny])
def health_check(_request):
    return Response({"status": "ok", "timestamp": timezone.now()})


class MapDataListView(APIView):
    """地图列表视图"""
    permission_classes = [permissions.AllowAny]

    def get(self, request):
        maps = MapData.objects.all()
        serializer = MapDataSerializer(maps, many=True, context={"request": request})
        return Response(serializer.data)

    def post(self, request):
        serializer = MapDataSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        serializer.save()
        return Response(serializer.data, status=status.HTTP_201_CREATED)


class MapSetListView(APIView):
    permission_classes = [permissions.AllowAny]

    def get(self, request):
        map_sets = MapSet.objects.select_related("robot").prefetch_related("members__map_data")
        return Response(MapSetSerializer(map_sets, many=True, context={"request": request}).data)


class MapDataDetailView(APIView):
    """地图详情视图"""
    permission_classes = [permissions.AllowAny]

    def get(self, request, pk):
        try:
            map_data = MapData.objects.get(pk=pk)
            serializer = MapDataSerializer(map_data, context={"request": request})
            return Response(serializer.data)
        except MapData.DoesNotExist:
            return Response({"detail": "地图不存在"}, status=status.HTTP_404_NOT_FOUND)

    def put(self, request, pk):
        try:
            map_data = MapData.objects.get(pk=pk)
            serializer = MapDataSerializer(map_data, data=request.data, partial=True)
            serializer.is_valid(raise_exception=True)
            serializer.save()
            return Response(serializer.data)
        except MapData.DoesNotExist:
            return Response({"detail": "地图不存在"}, status=status.HTTP_404_NOT_FOUND)

    def delete(self, request, pk):
        try:
            map_data = MapData.objects.get(pk=pk)
            if request_force_delete(request):
                result = _force_delete_map(map_data)
                return Response(
                    {
                        "detail": "地图及关联数据已强制删除。",
                        **result,
                    },
                    status=status.HTTP_200_OK,
                )
            if map_data.active:
                return Response(
                    {"detail": "当前活动地图不能删除，请先切换活动地图后再删除。"},
                    status=status.HTTP_409_CONFLICT,
                )
            references = _map_references(map_data)
            blocking = {name: count for name, count in references.items() if count}
            if blocking:
                detail = "地图已被引用，不能删除：" + "，".join(
                    f"{name} {count} 个" for name, count in blocking.items()
                )
                return Response({"detail": detail, "references": blocking}, status=status.HTTP_409_CONFLICT)
            files = [
                map_data.pgm_file,
                map_data.yaml_file,
                map_data.thumbnail,
                map_data.trajectory_file,
                map_data.mapping_trace,
                map_data.package_file,
            ]
            map_data.delete()
            for file_field in files:
                if file_field:
                    file_field.delete(save=False)
            return Response(status=status.HTTP_204_NO_CONTENT)
        except MapData.DoesNotExist:
            return Response({"detail": "地图不存在"}, status=status.HTTP_404_NOT_FOUND)
        except ProtectedError as exc:
            return Response(
                {"detail": str(exc.args[0]) if exc.args else "地图已被巡检任务或历史记录引用，不能删除。"},
                status=status.HTTP_409_CONFLICT,
            )


class MapDataDownloadView(APIView):
    """地图下载视图"""
    permission_classes = [permissions.AllowAny]

    def get(self, request, pk):
        try:
            map_data = MapData.objects.get(pk=pk)
            if map_data.package_file:
                response = FileResponse(map_data.package_file.open("rb"), content_type="application/zip")
                response["Content-Disposition"] = f'attachment; filename="{map_data.name}.zip"'
                return response
            import io
            import zipfile
            from django.http import HttpResponse

            zip_buffer = io.BytesIO()
            with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
                if map_data.pgm_file:
                    zip_file.write(map_data.pgm_file.path, 'map.pgm')
                if map_data.yaml_file:
                    zip_file.write(map_data.yaml_file.path, 'map.yaml')
                if map_data.thumbnail:
                    zip_file.write(map_data.thumbnail.path, 'preview.png')
                if map_data.trajectory_file:
                    zip_file.write(map_data.trajectory_file.path, 'map.txt')
                if map_data.mapping_trace:
                    zip_file.write(map_data.mapping_trace.path, 'mapping_trace.json')

            zip_buffer.seek(0)
            response = HttpResponse(zip_buffer, content_type='application/zip')
            response['Content-Disposition'] = f'attachment; filename={map_data.name}.zip'
            return response
        except MapData.DoesNotExist:
            return Response({"detail": "地图不存在"}, status=status.HTTP_404_NOT_FOUND)


class MapDataSetActiveView(APIView):
    """设为活动地图视图"""
    permission_classes = [permissions.AllowAny]

    def post(self, request, pk):
        try:
            map_data = MapData.objects.select_related("robot").get(pk=pk)
            MapData.objects.filter(robot=map_data.robot, active=True).update(active=False)
            map_data.active = True
            map_data.save(update_fields=["active", "updated_at"])
            command = None
            if map_data.robot:
                command = CommandService.create_robot_command(
                    robot=map_data.robot,
                    command_type="map.activate",
                    payload=_map_activation_payload(map_data, request),
                    operator=request.user if request.user.is_authenticated else None,
                    expiry_seconds=120,
                )
            serializer = MapDataSerializer(map_data, context={"request": request})
            data = serializer.data
            data["activation_command"] = RemoteCommandSerializer(command).data if command else None
            data["detail"] = (
                "活动地图已切换，并已向机器狗下发地图切换命令。"
                if command
                else "活动地图已切换；该地图未绑定机器人，未下发设备命令。"
            )
            return Response(data)
        except MapData.DoesNotExist:
            return Response({"detail": "地图不存在"}, status=status.HTTP_404_NOT_FOUND)


class MapDataManualCleanView(APIView):
    """Create and immediately activate a manually cleaned map revision."""
    permission_classes = [permissions.AllowAny]

    def post(self, request, pk):
        source = get_object_or_404(MapData.objects.select_related("robot"), pk=pk)
        if not source.robot:
            return Response({"detail": "地图未关联机器狗"}, status=status.HTTP_409_CONFLICT)
        if MapSetMember.objects.filter(map_data=source).exists():
            return Response({"detail": "地图集子图暂不支持单独人工擦除"}, status=status.HTTP_409_CONFLICT)
        if TaskExecution.objects.filter(robot=source.robot, state__in=TaskExecution.ACTIVE_STATES).exists():
            return Response({"detail": "机器人正在执行任务，不能切换地图"}, status=status.HTTP_409_CONFLICT)

        try:
            pgm_content, preview_content, erased_cells, strokes = _render_manual_cleanup(
                source,
                request.data.get("strokes"),
            )
        except (TypeError, ValueError) as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_400_BAD_REQUEST)

        source_description = _parse_map_description(source)
        base_map_id = source.edit_metadata.get("base_map_id") if isinstance(source.edit_metadata, dict) else None
        base_map_id = base_map_id or source.id
        source_map_dir = str(source_description.get("source_map_dir") or "")
        if not source_map_dir and source.name:
            source_map_dir = f"/home/dogrobot/runtime/nx-edge/data/jszr/map/{source.name}"
        edit_metadata = {
            "mode": "manual_cleanup",
            "base_map_id": base_map_id,
            "parent_map_id": source.id,
            "erased_cells": erased_cells,
            "strokes": strokes,
            "created_at": timezone.now().isoformat(),
        }
        description = {
            "source": "manual_cleanup",
            "source_map_dir": source_map_dir,
            "parent_map_id": source.id,
            "base_map_id": base_map_id,
            "route_hint": source_description.get("route_hint", ""),
        }
        name = str(request.data.get("name") or "").strip()
        if not name:
            name = f"{source.name}-人工清理-{timezone.localtime():%m%d-%H%M}"
        if len(name) > 128:
            return Response({"detail": "地图名称不能超过 128 个字符"}, status=status.HTTP_400_BAD_REQUEST)

        cleaned = None
        try:
            with transaction.atomic():
                cleaned = MapData.objects.create(
                    name=name,
                    robot=source.robot,
                    resolution=source.resolution,
                    width=source.width,
                    height=source.height,
                    origin=source.origin,
                    description=json.dumps(description, ensure_ascii=False),
                    parent_map=source,
                    edit_metadata=edit_metadata,
                )
                cleaned.pgm_file.save(f"{cleaned.id}_map.pgm", ContentFile(pgm_content), save=False)
                with source.yaml_file.open("rb") as yaml_stream:
                    cleaned.yaml_file.save(f"{cleaned.id}_map.yaml", ContentFile(yaml_stream.read()), save=False)
                if source.trajectory_file:
                    with source.trajectory_file.open("rb") as trace_stream:
                        cleaned.trajectory_file.save(f"{cleaned.id}_map.txt", ContentFile(trace_stream.read()), save=False)
                if source.mapping_trace:
                    with source.mapping_trace.open("rb") as trace_stream:
                        cleaned.mapping_trace.save(f"{cleaned.id}_mapping_trace.json", ContentFile(trace_stream.read()), save=False)
                cleaned.thumbnail.save(f"{cleaned.id}_preview.png", ContentFile(preview_content), save=False)
                cleaned.active = True
                cleaned.save()

                MapData.objects.filter(robot=source.robot, active=True).exclude(pk=cleaned.pk).update(active=False)
                migrated = {
                    "routes": PatrolRoute.objects.filter(map_data=source).update(map_data=cleaned),
                    "zones": Zone.objects.filter(map_data=source).update(map_data=cleaned),
                    "schedules": PatrolSchedule.objects.filter(map_data=source).update(map_data=cleaned),
                }
                command = CommandService.create_robot_command(
                    robot=source.robot,
                    command_type="map.activate",
                    payload=_map_activation_payload(cleaned, request),
                    operator=request.user if request.user.is_authenticated else None,
                    expiry_seconds=300,
                )
        except Exception:
            if cleaned:
                for field in (
                    cleaned.pgm_file,
                    cleaned.yaml_file,
                    cleaned.thumbnail,
                    cleaned.trajectory_file,
                    cleaned.mapping_trace,
                    cleaned.package_file,
                ):
                    if field:
                        field.delete(save=False)
            raise

        data = MapDataSerializer(cleaned, context={"request": request}).data
        data["activation_command"] = RemoteCommandSerializer(command).data
        data["migrated_references"] = migrated
        data["detail"] = "人工清理版已保存并下发机器狗，正在同步清理 PGM 与 PCD。"
        return Response(data, status=status.HTTP_201_CREATED)


class MapDataPreviewView(APIView):
    """PGM地图预览视图 - 将PGM文件转换为可预览的PNG图像"""
    permission_classes = [permissions.AllowAny]

    def _png_response(self, content):
        response = HttpResponse(content, content_type='image/png')
        response["Cache-Control"] = "no-store, no-cache, must-revalidate"
        return response

    def get(self, request, pk):
        try:
            map_data = MapData.objects.get(pk=pk)
            
            if map_data.thumbnail and request.query_params.get("raw") not in {"1", "true"}:
                with open(map_data.thumbnail.path, 'rb') as f:
                    content = f.read()
                return self._png_response(content)
            
            if not map_data.pgm_file:
                return self._png_response(self._generate_placeholder_image())
            
            pgm_path = map_data.pgm_file.path
            raw_preview = request.query_params.get("raw") in {"1", "true"}
            png_data = self._pgm_to_png(pgm_path, max_size=None if raw_preview else 1200)
            
            return self._png_response(png_data)
            
        except MapData.DoesNotExist:
            return self._png_response(self._generate_placeholder_image())
        except Exception as e:
            print(f"Error generating preview: {e}")
            return self._png_response(self._generate_placeholder_image())

    def _pgm_to_png(self, pgm_path, max_size=1200):
        """将PGM文件转换为浏览器兼容的RGB PNG格式。"""
        from PIL import Image

        with open(pgm_path, 'rb') as f:
            header = f.readline().decode('ascii').strip()
            
            while True:
                line = f.readline().decode('ascii').strip()
                if not line.startswith('#'):
                    break
            
            dims = line.split()
            width, height = int(dims[0]), int(dims[1])
            
            max_val = int(f.readline().decode('ascii').strip())
            
            if header == 'P5':
                raw_data = f.read()
            elif header == 'P2':
                raw_data = bytearray()
                for line in f:
                    for val in line.decode('ascii').split():
                        raw_data.append(int(val))
            else:
                return self._generate_placeholder_image()

        expected_size = width * height
        if len(raw_data) < expected_size:
            return self._generate_placeholder_image()

        image = Image.frombytes("L", (width, height), bytes(raw_data[:expected_size]))
        if max_val != 255:
            image = image.point(lambda value: int((value / max_val) * 255))

        if max_size:
            image.thumbnail((max_size, max_size), Image.Resampling.NEAREST)
        image = image.convert("RGB")

        output = io.BytesIO()
        image.save(output, format="PNG", optimize=True)
        return output.getvalue()

    def _create_png(self, width, height, raw_data, original_width, max_val):
        """创建PNG图像"""
        import zlib
        
        signature = b'\x89PNG\r\n\x1a\n'
        
        ihdr_data = (
            width.to_bytes(4, 'big') +
            height.to_bytes(4, 'big') +
            b'\x08' +
            b'\x00' +
            b'\x00' +
            b'\x00' +
            b'\x00' +
            b'\x00'
        )
        ihdr = self._create_chunk(b'IHDR', ihdr_data)
        
        scale = width / original_width
        filtered_data = bytearray()
        
        for y in range(height):
            filtered_data.append(0)
            
            for x in range(width):
                orig_x = int(x / scale)
                orig_y = int(y / scale)
                orig_idx = orig_y * original_width + orig_x
                
                if orig_idx < len(raw_data):
                    gray = raw_data[orig_idx]
                    # 直接转换，不反转颜色
                    pixel = int((gray / max_val) * 255)
                else:
                    pixel = 255
                
                filtered_data.append(pixel)
        
        compressed = zlib.compress(filtered_data)
        idat = self._create_chunk(b'IDAT', compressed)
        
        iend = self._create_chunk(b'IEND', b'')
        
        return signature + ihdr + idat + iend

    def _create_chunk(self, type_bytes, data):
        """创建PNG chunk"""
        length = len(data).to_bytes(4, 'big')
        crc_data = type_bytes + data
        crc = self._crc32(crc_data).to_bytes(4, 'big')
        return length + type_bytes + data + crc

    def _crc32(self, data):
        """计算CRC32"""
        crc = 0xffffffff
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc >> 1) ^ (0xedb88320 if (crc & 1) else 0)
        return crc ^ 0xffffffff

    def _generate_placeholder_image(self):
        """生成浏览器兼容的占位符图像。"""
        from PIL import Image

        image = Image.new("RGB", (32, 32), (238, 242, 247))
        output = io.BytesIO()
        image.save(output, format="PNG")
        return output.getvalue()


class MapDataMappingTraceView(APIView):
    permission_classes = [permissions.AllowAny]

    def get(self, request, pk):
        map_data = get_object_or_404(MapData, pk=pk)
        if map_data.mapping_trace:
            try:
                with map_data.mapping_trace.open("rb") as stream:
                    return Response(json.loads(stream.read().decode("utf-8")))
            except (OSError, UnicodeDecodeError, json.JSONDecodeError):
                return Response({"detail": "建图轨迹文件损坏"}, status=status.HTTP_422_UNPROCESSABLE_ENTITY)
        if map_data.trajectory_file:
            samples = []
            try:
                with map_data.trajectory_file.open("rb") as stream:
                    lines = stream.read().decode("utf-8").splitlines()
                for index, line in enumerate(lines):
                    if not line.strip() or line.lstrip().startswith("#"):
                        continue
                    fields = line.split()
                    if len(fields) < 3:
                        continue
                    samples.append({
                        "index": index,
                        "stamp": None,
                        "slam": {"x": float(fields[0]), "y": float(fields[1]), "yaw": float(fields[2])},
                        "rtk": {"valid": False, "reason": "not_recorded"},
                    })
            except (OSError, UnicodeDecodeError, ValueError):
                return Response({"detail": "旧版建图轨迹文件损坏"}, status=status.HTTP_422_UNPROCESSABLE_ENTITY)
            return Response({"format": "roamerx.mapping-trace.legacy", "frame_id": "map", "alignment_locked": False, "samples": samples})
        return Response({"format": "roamerx.mapping-trace.empty", "frame_id": "map", "alignment_locked": False, "samples": []})


class RobotMappingStatusView(APIView):
    """查询机器狗建图命令状态，包含连接状态和 edge_agent 真实建图状态机。"""
    permission_classes = [permissions.AllowAny]

    def get(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        command = (
            robot.remote_commands.filter(command_type__startswith="mapping.")
            .order_by("-issued_at")
            .first()
        )
        effective_connection_status = robot.effective_connection_status()
        latest_status = RobotStatusLatest.objects.filter(robot=robot).first()
        live_mapping = {}
        if latest_status and latest_status.raw_payload:
            live_mapping = latest_status.raw_payload.get("mapping") or {}
        live_progress = live_mapping.get("save_progress") or {}
        progress_is_current = not command
        if command and live_progress:
            try:
                progress_is_current = (
                    float(live_progress.get("updated_at_unix") or 0)
                    >= command.issued_at.timestamp() - 5
                )
            except (TypeError, ValueError):
                progress_is_current = False
        live_sample_is_current = bool(
            latest_status
            and (
                not command
                or progress_is_current
                or latest_status.sampled_at >= command.issued_at - timedelta(seconds=5)
            )
        )
        command_is_active = bool(
            command
            and command.status in {"created", "published", "accepted", "executing"}
        )
        # Edge Agent may publish its terminal mapping snapshot before the
        # command.result event reaches the cloud. Do not let the old command
        # status keep the indoor/outdoor controls locked in that brief (or
        # recovered) window.
        live_mapping_state = str(live_mapping.get("state") or "")
        live_mapping_terminal = bool(
            live_mapping
            and not live_mapping.get("process_alive")
            and live_mapping_state in {"exited", "completed", "failed", "cancelled"}
        )
        effective_command_status = command.status if command else "idle"
        if command_is_active and live_mapping_terminal:
            command_is_active = False
            effective_command_status = (
                "failed" if live_mapping_state == "failed"
                else "cancelled" if live_mapping_state == "cancelled"
                else "succeeded"
            )
        live_mapping_is_active = bool(
            live_mapping.get("process_alive")
            or live_mapping.get("state") not in {None, "idle"}
            or (live_progress and progress_is_current)
        )

        # 从 result_payload 提取 edge_agent 返回的真实 mapping state
        mapping_state = "idle"
        mapping_result = {}
        using_live_mapping = False
        if (
            live_mapping
            and live_sample_is_current
            and (not command_is_active or live_mapping_is_active)
        ):
            using_live_mapping = True
            mapping_result = live_mapping
            mapping_state = mapping_result.get("state", "idle")
        elif command and command.result_payload:
            mapping_result = command.result_payload
            mapping_state = mapping_result.get("state", "idle")
        elif command:
            status_map = {
                "created": "command_created",
                "published": "command_published",
                "accepted": "command_accepted",
                "rejected": "command_rejected",
                "timed_out": "command_timed_out",
                "succeeded": "idle",
                "failed": "command_failed",
            }
            mapping_state = status_map.get(command.status, command.status)

        latest_map = MapData.objects.filter(robot=robot).order_by("-created_at").first()
        robot_current_map = {}
        if latest_status and latest_status.raw_payload:
            robot_current_map = latest_status.raw_payload.get("current_map") or {}
        if not robot_current_map:
            robot_current_map = {
                "map_id": robot.current_map_id,
                "map_version": robot.current_map_version,
            }
        live_error_message = str(live_progress.get("error") or "")
        current_error_code = (
            "MAPPING_SAVE_FAILED"
            if using_live_mapping and live_error_message
            else ("" if using_live_mapping else (command.error_code if command else ""))
        )
        current_error_message = (
            live_error_message
            if using_live_mapping
            else (command.error_message if command else "")
        )
        return Response(
            {
                "robot_id": robot.id,
                "robot_code": robot.code,
                "connection_status": effective_connection_status,
                "raw_connection_status": robot.connection_status,
                "robot_status": robot.status,
                "agent_version": robot.agent_version or "",
                "current_map": robot_current_map,
                "current_map_id": robot.current_map_id,
                "current_map_version": robot.current_map_version,
                "command_type": command.command_type if command else None,
                "mapping_state": mapping_state,
                "command_status": effective_command_status,
                "command_id": str(command.id) if command else None,
                "issued_at": command.issued_at if command else None,
                "acknowledged_at": command.acknowledged_at if command else None,
                "finished_at": command.finished_at if command else None,
                "error_code": current_error_code,
                "error_message": current_error_message,
                "last_command_error_code": command.error_code if command else "",
                "last_command_error_message": command.error_message if command else "",
                "result": mapping_result,
                "map_name": mapping_result.get("map_name", ""),
                "files": mapping_result.get("files", {}),
                "map_dir": mapping_result.get("map_dir", ""),
                "latest_map": MapDataSerializer(latest_map, context={"request": request}).data
                if latest_map
                else None,
                "last_seen_at": robot.last_seen_at,
                "updated_at": robot.updated_at,
            }
        )


class RobotMappingCommandView(APIView):
    permission_classes = [permissions.AllowAny]
    command_type = ""
    expiry_seconds = 120

    def build_payload(self, request, robot: Robot) -> dict:
        return {}

    def post(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        command = CommandService.create_robot_command(
            robot=robot,
            command_type=self.command_type,
            payload=self.build_payload(request, robot),
            operator=request.user if request.user.is_authenticated else None,
            expiry_seconds=self.expiry_seconds,
        )
        return Response(
            {
                "command_id": str(command.id),
                "robot_id": robot.id,
                "robot_code": robot.code,
                "command_type": command.command_type,
                "status": command.status,
                "message": "建图命令已创建，等待 device worker 发布到 Edge Agent",
            },
            status=status.HTTP_202_ACCEPTED,
        )


class RobotMappingStartView(RobotMappingCommandView):
    command_type = "mapping.start"
    expiry_seconds = 120

    def build_payload(self, request, robot: Robot) -> dict:
        map_name = request.data.get("map_name") or f"{robot.name} 现场地图"
        mapping_type = request.data.get("mapping_type") or "indoor"
        scene_scope = request.data.get("scene_scope") or mapping_type
        if mapping_type not in {"indoor", "outdoor"}:
            raise ValidationError({"mapping_type": "必须是 indoor 或 outdoor"})
        if (mapping_type == "indoor") != (scene_scope == "indoor"):
            raise ValidationError({"scene_scope": "室内类型只能使用 indoor；室外类型只能使用 transition/outdoor"})
        return {
            "map_name": map_name,
            "route_hint": request.data.get("route_hint", ""),
            "operator_note": request.data.get("operator_note", ""),
            "record_rosbag": bool(request.data.get("record_rosbag", False)),
            "scene_scope": scene_scope,
            "mapping_type": mapping_type,
        }


class RobotMappingOriginStatusView(RobotMappingStatusView):
    """Origin status is part of the unified mapping snapshot; keep a focused REST alias."""


class RobotMappingOriginStartView(RobotMappingCommandView):
    command_type = "mapping.origin_start"
    expiry_seconds = 180

    def build_payload(self, request, robot: Robot) -> dict:
        scene_scope = request.data.get("scene_scope") or "outdoor"
        if scene_scope not in {"transition", "outdoor"}:
            raise ValidationError({"scene_scope": "锁定原点仅支持 transition 或 outdoor"})
        return {
            "map_name": request.data.get("map_name") or f"{robot.name} 室外地图",
            "route_hint": request.data.get("route_hint", ""),
            "mapping_type": "outdoor",
            "scene_scope": scene_scope,
            "mapping_session_id": request.data.get("mapping_session_id", ""),
            "record_rosbag": bool(request.data.get("record_rosbag", False)),
            "prepare_only": bool(request.data.get("prepare_only", False)),
        }


class RobotMappingOriginCancelView(RobotMappingCommandView):
    command_type = "mapping.origin_cancel"
    expiry_seconds = 60

    def build_payload(self, request, robot: Robot) -> dict:
        return {"mapping_session_id": request.data.get("mapping_session_id", "")}


class RobotMappingOriginExtractGlobalView(RobotMappingCommandView):
    command_type = "mapping.origin_extract_global"
    expiry_seconds = 120

    def build_payload(self, request, robot: Robot) -> dict:
        map_data = get_object_or_404(MapData, pk=request.data.get("map_id"), robot=robot)
        try:
            description = json.loads(map_data.description or "{}")
        except (TypeError, ValueError):
            description = {}
        origin_text = description.get("gnss_origin_yaml") or ""
        try:
            global_enu = yaml.safe_load(origin_text) if origin_text else {}
        except yaml.YAMLError as exc:
            raise ValidationError({"map_id": f"地图原点配置损坏: {exc}"}) from exc
        if not isinstance(global_enu, dict) or not global_enu.get("alignment_locked"):
            raise ValidationError({"map_id": "当前地图没有有效的锁定 ENU 原点"})
        return {
            "source_map_id": str(map_data.id),
            "source_map_name": map_data.name,
            "global_enu": global_enu,
        }


class RobotMappingSlamStartView(RobotMappingCommandView):
    command_type = "mapping.slam_start"
    expiry_seconds = 180

    def build_payload(self, request, robot: Robot) -> dict:
        mapping_type = request.data.get("mapping_type") or "indoor"
        if mapping_type not in {"indoor", "outdoor"}:
            raise ValidationError({"mapping_type": "必须是 indoor 或 outdoor"})
        scene_scope = request.data.get("scene_scope") or mapping_type
        if (mapping_type == "indoor") != (scene_scope == "indoor"):
            raise ValidationError({"scene_scope": "室内类型只能使用 indoor；室外类型只能使用 transition/outdoor"})
        return {
            "map_name": request.data.get("map_name") or f"{robot.name} 现场地图",
            "route_hint": request.data.get("route_hint", ""),
            "record_rosbag": bool(request.data.get("record_rosbag", False)),
            "mapping_type": mapping_type,
            "scene_scope": scene_scope,
            "mapping_session_id": request.data.get("mapping_session_id", ""),
        }


class RobotMappingBeginView(RobotMappingCommandView):
    command_type = "mapping.begin"
    expiry_seconds = 60

    def build_payload(self, request, robot: Robot) -> dict:
        return {
            "mapping_session_id": request.data.get("mapping_session_id", ""),
            "heading_check_confirmed": bool(request.data.get("heading_check_confirmed", False)),
        }


class RobotMappingSaveView(RobotMappingCommandView):
    command_type = "mapping.save"
    # Large outdoor map export includes keyframe filtering and submap building.
    expiry_seconds = 7200

    def build_payload(self, request, robot: Robot) -> dict:
        return {
            "map_name": request.data.get("map_name", ""),
            "mapping_session_id": request.data.get("mapping_session_id", ""),
        }


class RobotMappingCancelView(RobotMappingCommandView):
    command_type = "mapping.cancel"
    expiry_seconds = 60

    def build_payload(self, request, robot: Robot) -> dict:
        return {
            "reason": request.data.get("reason", "operator_cancel"),
            "mapping_session_id": request.data.get("mapping_session_id", ""),
        }


class RobotMappingSyncView(RobotMappingCommandView):
    """触发 Edge Agent 打包最近的地图文件并上传（不需要活跃建图会话）。"""
    command_type = "mapping.save"
    expiry_seconds = 300

    def build_payload(self, request, robot: Robot) -> dict:
        # 不传 map_name，让 Edge Agent 使用 session 目录名（如 20260626_215355）
        return {}


class RobotNavigationStatusView(APIView):
    """查询机器狗导航栈状态和最近导航控制命令。"""
    permission_classes = [permissions.AllowAny]

    def get(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        latest = RobotStatusLatest.objects.filter(robot=robot).first()
        command = robot.remote_commands.filter(
            command_type__in=[
                "nav.start", "nav.restart", "nav.recover", "nav.stop",
                "nav.initial_pose", "nav.relocalize",
            ]
        ).order_by("-issued_at").first()
        return Response(
            {
                "robot_id": robot.id,
                "robot_code": robot.code,
                "connection_status": robot.effective_connection_status(),
                "raw_connection_status": robot.connection_status,
                "current_map_id": robot.current_map_id,
                "current_map_version": robot.current_map_version,
                "localization_status": robot.localization_status,
                "ros_ready": robot.ros_ready,
                "nav_ready": robot.nav_ready,
                "status": RobotStatusSerializer(latest).data if latest else None,
                "command": {
                    "id": str(command.id),
                    "command_type": command.command_type,
                    "status": command.status,
                    "issued_at": command.issued_at,
                    "published_at": command.published_at,
                    "acknowledged_at": command.acknowledged_at,
                    "finished_at": command.finished_at,
                    "error_code": command.error_code,
                    "error_message": command.error_message,
                } if command else None,
            }
        )


class RobotNavigationCommandView(APIView):
    permission_classes = [permissions.AllowAny]
    command_type = ""
    expiry_seconds = 180

    def post(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        if self.command_type in {"nav.start", "nav.restart"} and robot.effective_connection_status() != "online":
            return Response(
                {"detail": "机器狗 Edge Agent 当前离线，无法启动导航栈。"},
                status=status.HTTP_409_CONFLICT,
            )
        command = CommandService.create_robot_command(
            robot=robot,
            command_type=self.command_type,
            payload={
                "reason": "route_planner_test",
                "map_id": str(request.data.get("map_id") or robot.current_map_id or ""),
                "map_version": request.data.get("map_version") or robot.current_map_version or "",
            },
            operator=request.user if request.user.is_authenticated else None,
            expiry_seconds=self.expiry_seconds,
        )
        return Response(RemoteCommandSerializer(command).data, status=status.HTTP_202_ACCEPTED)


class RobotNavigationInitialPoseView(APIView):
    permission_classes = [permissions.AllowAny]

    def post(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        if robot.effective_connection_status() != "online":
            return Response(
                {"detail": "机器狗 Edge Agent 当前离线，无法设置初始定位。"},
                status=status.HTTP_409_CONFLICT,
            )
        seed_source = str(request.data.get("seed_source") or "").strip()
        supplied = [request.data.get(field) is not None for field in ("x", "y", "yaw")]
        if not any(supplied) and seed_source not in {"mapping_start", "last_trusted", "rtk"}:
            return Response({"detail": "初始定位需要 x、y、yaw 或建图起点。"}, status=status.HTTP_400_BAD_REQUEST)
        if any(supplied) and not all(supplied):
            return Response({"detail": "初始定位需要同时提供数值 x、y、yaw。"}, status=status.HTTP_400_BAD_REQUEST)
        try:
            coordinates = (
                {field: float(request.data[field]) for field in ("x", "y", "yaw")}
                if all(supplied)
                else {}
            )
        except (TypeError, ValueError, KeyError):
            return Response({"detail": "初始定位需要同时提供数值 x、y、yaw。"}, status=status.HTTP_400_BAD_REQUEST)
        command = CommandService.create_robot_command(
            robot=robot,
            command_type="nav.initial_pose",
            payload={
                "reason": "manual_initial_pose",
                "frame_id": request.data.get("frame_id") or "map",
                "seed_source": seed_source,
                **coordinates,
                "map_id": str(request.data.get("map_id") or robot.current_map_id or ""),
                "map_version": request.data.get("map_version") or robot.current_map_version or "",
            },
            operator=request.user if request.user.is_authenticated else None,
            expiry_seconds=60,
        )
        return Response(RemoteCommandSerializer(command).data, status=status.HTTP_202_ACCEPTED)


class RobotNavigationStartView(RobotNavigationCommandView):
    command_type = "nav.start"


class RobotNavigationRestartView(RobotNavigationCommandView):
    command_type = "nav.restart"


class RobotNavigationStopView(RobotNavigationCommandView):
    command_type = "nav.stop"
    expiry_seconds = 60


class RobotNavigationProbeView(RobotNavigationCommandView):
    command_type = "nav.status"
    expiry_seconds = 60


class RobotNavigationRecoverView(RobotNavigationCommandView):
    command_type = "nav.recover"
    expiry_seconds = 180


class RobotNavigationRelocalizeView(RobotNavigationCommandView):
    command_type = "nav.relocalize"
    expiry_seconds = 180

    def build_payload(self, request, robot: Robot) -> dict:
        payload = {
            "reason": "operator_active_relocalization",
            "seed_source": request.data.get("seed_source") or "last_trusted",
            "map_id": str(request.data.get("map_id") or robot.current_map_id or ""),
            "map_version": request.data.get("map_version") or robot.current_map_version or "",
        }
        supplied = [request.data.get(field) is not None for field in ("x", "y", "yaw")]
        if any(supplied):
            if not all(supplied):
                raise ValueError("主动重定位需要同时提供 x、y、yaw")
            payload.update({
                "x": float(request.data["x"]),
                "y": float(request.data["y"]),
                "yaw": float(request.data["yaw"]),
            })
        return payload

    def post(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        if robot.effective_connection_status() != "online":
            return Response({"detail": "机器狗 Edge Agent 当前离线。"}, status=status.HTTP_409_CONFLICT)
        try:
            payload = self.build_payload(request, robot)
        except (TypeError, ValueError):
            return Response(
                {"detail": "主动重定位的 x、y、yaw 必须同时为数值。"},
                status=status.HTTP_400_BAD_REQUEST,
            )
        command = CommandService.create_robot_command(
            robot=robot,
            command_type=self.command_type,
            payload=payload,
            operator=request.user if request.user.is_authenticated else None,
            expiry_seconds=self.expiry_seconds,
        )
        return Response(RemoteCommandSerializer(command).data, status=status.HTTP_202_ACCEPTED)


class RobotSensorRestartView(APIView):
    permission_classes = [permissions.AllowAny]

    def post(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        if robot.effective_connection_status() != "online":
            return Response({"detail": "机器狗 Edge Agent 当前离线。"}, status=status.HTTP_409_CONFLICT)
        sensor = str(request.data.get("sensor") or "").strip().lower()
        if sensor not in {"lidar", "imu", "lidar_imu", "rtk"}:
            return Response({"detail": "仅支持重启雷达/IMU或 RTK。"}, status=status.HTTP_400_BAD_REQUEST)
        command = CommandService.create_robot_command(
            robot=robot,
            command_type="sensor.restart",
            payload={"sensor": sensor, "reason": "operator_sensor_recovery"},
            operator=request.user if request.user.is_authenticated else None,
            expiry_seconds=60,
        )
        return Response(RemoteCommandSerializer(command).data, status=status.HTTP_202_ACCEPTED)


class DeviceMapUploadView(APIView):
    """Edge Agent 上传现场建图结果。"""
    permission_classes = [permissions.AllowAny]

    def post(self, request):
        metadata_raw = request.data.get("metadata") or "{}"
        try:
            metadata = json.loads(metadata_raw) if isinstance(metadata_raw, str) else metadata_raw
        except json.JSONDecodeError:
            return Response({"detail": "metadata 不是合法 JSON"}, status=status.HTTP_400_BAD_REQUEST)

        robot_code = request.data.get("robot_code") or metadata.get("robot_code")
        robot = get_object_or_404(Robot, code=robot_code)
        map_name = request.data.get("map_name") or metadata.get("map_name") or f"{robot.name} 现场地图"
        package = request.FILES.get("map_package")
        if not package:
            return Response({"detail": "缺少 map_package 文件"}, status=status.HTTP_400_BAD_REQUEST)

        extracted: dict[str, bytes] = {}
        submap_files: dict[str, dict[str, bytes]] = {}
        map_set_manifest: dict = {}
        package_bytes = package.read()
        package_names: list[str] = []
        try:
            with zipfile.ZipFile(io.BytesIO(package_bytes)) as archive:
                package_names = archive.namelist()
                for name in archive.namelist():
                    if name in {"map.yaml", "map.pgm", "map_preview.png", "preview.png", "gnss_origin.yaml", "map.txt", "mapping_trace.json", "map_manifest.json", "trajectory_raw.csv", "trajectory_optimized.csv", "recording_manifest.yaml"}:
                        extracted[name] = archive.read(name)
                    elif name == "map_set/map_set_manifest.json":
                        map_set_manifest = json.loads(archive.read(name).decode("utf-8"))
                    elif name.startswith("map_set/"):
                        parts = name.split("/")
                        if len(parts) == 3 and parts[2] in {"map.yaml", "map.pgm", "map_preview.png", "submap.json"}:
                            submap_files.setdefault(parts[1], {})[parts[2]] = archive.read(name)
        except zipfile.BadZipFile:
            return Response({"detail": "map_package 不是合法 zip"}, status=status.HTTP_400_BAD_REQUEST)

        if "map.yaml" not in extracted or "map.pgm" not in extracted:
            return Response({"detail": "地图包必须包含 map.yaml 和 map.pgm"}, status=status.HTTP_400_BAD_REQUEST)

        yaml_metadata = _parse_simple_map_yaml(extracted["map.yaml"])
        width, height = _read_pgm_dimensions(extracted["map.pgm"])
        map_manifest = metadata.get("map_manifest") if isinstance(metadata.get("map_manifest"), dict) else {}
        if extracted.get("map_manifest.json"):
            try:
                parsed_manifest = json.loads(extracted["map_manifest.json"].decode("utf-8"))
                if isinstance(parsed_manifest, dict):
                    map_manifest = parsed_manifest
            except (UnicodeDecodeError, json.JSONDecodeError):
                pass
        if map_manifest.get("completeness") == "complete" and "map.pcd" not in package_names:
            return Response(
                {"detail": "完整地图包必须包含 map.pcd，当前上传包无法用于三维 NDT 导航"},
                status=status.HTTP_400_BAD_REQUEST,
            )
        description = {
            "source": "edge_mapping",
            "map_version": metadata.get("map_version", ""),
            "mapping_session_id": metadata.get("mapping_session_id", ""),
            "source_map_dir": metadata.get("source_map_dir", ""),
            "dynamic_filter": metadata.get("dynamic_filter", {}),
            "slam_health": metadata.get("slam_health", {}),
            "rescue": metadata.get("rescue", {}),
            "route_hint": metadata.get("route_hint", ""),
            "files": metadata.get("files", []),
            "image": yaml_metadata.get("image", ""),
            "gnss_origin_yaml": extracted.get("gnss_origin.yaml", b"").decode("utf-8", errors="ignore")[:16384],
            "map_manifest": map_manifest,
            "coordinate_mode": map_manifest.get("coordinate_mode") or metadata.get("coordinate_mode", ""),
            "scene_scope": map_manifest.get("scene_scope") or metadata.get("scene_scope", ""),
            "localization_mode": map_manifest.get("localization_mode") or metadata.get("localization_mode", ""),
            "origin_status": map_manifest.get("origin_status") or metadata.get("origin_status", ""),
            "package_files": package_names,
            "package_sha256": hashlib.sha256(package_bytes).hexdigest(),
            "mapping_metrics": metadata.get("mapping_metrics", {}),
        }
        auto_activate = bool(metadata.get("auto_activate", False))
        with transaction.atomic():
            map_data = MapData.objects.create(
                name=map_name,
                robot=robot,
                resolution=float(yaml_metadata.get("resolution") or metadata.get("resolution") or 0.05),
                width=width,
                height=height,
                origin=yaml_metadata.get("origin") or metadata.get("origin") or [],
                description=json.dumps(description, ensure_ascii=False),
                coordinate_mode=str(description.get("coordinate_mode") or ""),
                scene_scope=str(description.get("scene_scope") or ""),
                localization_mode=str(description.get("localization_mode") or ""),
                origin_status=str(description.get("origin_status") or ""),
                map_completeness=str(map_manifest.get("completeness") or ""),
                mapping_metrics=description.get("mapping_metrics") if isinstance(description.get("mapping_metrics"), dict) else {},
            )
            map_data.yaml_file.save(f"{map_data.id}_map.yaml", ContentFile(extracted["map.yaml"]), save=False)
            if extracted.get("map.txt"):
                map_data.trajectory_file.save(f"{map_data.id}_map.txt", ContentFile(extracted["map.txt"]), save=False)
            if extracted.get("mapping_trace.json"):
                map_data.mapping_trace.save(f"{map_data.id}_mapping_trace.json", ContentFile(extracted["mapping_trace.json"]), save=False)
            map_data.pgm_file.save(f"{map_data.id}_map.pgm", ContentFile(extracted["map.pgm"]), save=False)
            preview = extracted.get("map_preview.png") or extracted.get("preview.png")
            if preview:
                map_data.thumbnail.save(f"{map_data.id}_preview.png", ContentFile(preview), save=False)
            map_data.package_file.save(f"{map_data.id}_map_package.zip", ContentFile(package_bytes), save=False)
            map_data.save()
            map_set = None
            if map_set_manifest and submap_files:
                map_set = MapSet.objects.create(
                    name=f"{map_name} 地图集",
                    robot=robot,
                    version=str(metadata.get("map_version") or ""),
                    manifest=map_set_manifest,
                )
                source_dir = str(metadata.get("source_map_dir") or "").rstrip("/")
                for sequence, submap in enumerate(map_set_manifest.get("submaps") or [], start=1):
                    submap_id = str(submap.get("submap_id") or "")
                    files = submap_files.get(submap_id) or {}
                    if not submap_id or "map.yaml" not in files or "map.pgm" not in files:
                        continue
                    submap_yaml = _parse_simple_map_yaml(files["map.yaml"])
                    submap_width, submap_height = _read_pgm_dimensions(files["map.pgm"])
                    local_submap_dir = f"{source_dir}/map_set/{submap_id}" if source_dir else ""
                    submap_description = {
                        "source": "edge_mapping_submap",
                        "map_set_id": map_set.id,
                        "submap_id": submap_id,
                        "source_map_dir": local_submap_dir,
                        "metadata": submap,
                    }
                    member_map = MapData.objects.create(
                        name=f"{map_name} / {submap_id}",
                        robot=robot,
                        resolution=float(submap_yaml.get("resolution") or 0.05),
                        width=submap_width,
                        height=submap_height,
                        origin=submap_yaml.get("origin") or [],
                        description=json.dumps(submap_description, ensure_ascii=False),
                    )
                    member_map.yaml_file.save(f"{member_map.id}_{submap_id}.yaml", ContentFile(files["map.yaml"]), save=False)
                    member_map.pgm_file.save(f"{member_map.id}_{submap_id}.pgm", ContentFile(files["map.pgm"]), save=False)
                    if files.get("map_preview.png"):
                        member_map.thumbnail.save(f"{member_map.id}_{submap_id}.png", ContentFile(files["map_preview.png"]), save=False)
                    member_map.save()
                    MapSetMember.objects.create(
                        map_set=map_set,
                        map_data=member_map,
                        sequence=sequence,
                        submap_id=submap_id,
                        metadata=submap,
                    )
            command = None
            if auto_activate:
                MapData.objects.filter(robot=robot, active=True).exclude(pk=map_data.pk).update(active=False)
                map_data.active = True
                map_data.save(update_fields=["active", "updated_at"])
                command = CommandService.create_robot_command(
                    robot=robot,
                    command_type="map.activate",
                    payload=_map_activation_payload(map_data, request),
                    operator=None,
                    expiry_seconds=120,
                )
        data = MapDataSerializer(map_data, context={"request": request}).data
        data["map_set"] = MapSetSerializer(map_set, context={"request": request}).data if map_set else None
        data["activation_command"] = RemoteCommandSerializer(command).data if command else None
        data["detail"] = (
            "地图已上传、设为活动地图，并已向机器狗下发地图切换命令。"
            if command
            else "地图已上传。"
        )
        return Response(data, status=status.HTTP_201_CREATED)


class PatrolRouteListView(APIView):
    """巡逻路线列表视图"""
    permission_classes = [permissions.AllowAny]

    def get(self, request):
        routes = PatrolRoute.objects.all()
        serializer = PatrolRouteSerializer(routes, many=True)
        return Response(serializer.data)

    def post(self, request):
        serializer = PatrolRouteSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        serializer.save()
        return Response(serializer.data, status=status.HTTP_201_CREATED)


class PatrolRouteDetailView(APIView):
    """巡逻路线详情视图"""
    permission_classes = [permissions.AllowAny]

    def get(self, request, pk):
        try:
            route = PatrolRoute.objects.get(pk=pk)
            serializer = PatrolRouteSerializer(route)
            return Response(serializer.data)
        except PatrolRoute.DoesNotExist:
            return Response({"detail": "路线不存在"}, status=status.HTTP_404_NOT_FOUND)

    def put(self, request, pk):
        try:
            route = PatrolRoute.objects.get(pk=pk)
            serializer = PatrolRouteSerializer(route, data=request.data, partial=True)
            serializer.is_valid(raise_exception=True)
            serializer.save()
            return Response(serializer.data)
        except PatrolRoute.DoesNotExist:
            return Response({"detail": "路线不存在"}, status=status.HTTP_404_NOT_FOUND)

    def delete(self, request, pk):
        try:
            route = PatrolRoute.objects.get(pk=pk)
            route.delete()
            return Response(status=status.HTTP_204_NO_CONTENT)
        except PatrolRoute.DoesNotExist:
            return Response({"detail": "路线不存在"}, status=status.HTTP_404_NOT_FOUND)
        except ProtectedError:
            return Response(
                {"detail": "该路线已被巡检任务、巡检计划或执行记录引用，请先解绑后再删除。"},
                status=status.HTTP_409_CONFLICT,
            )


def validate_task_execution_readiness(task: PatrolTask) -> Response | None:
    CommandService.ensure_task_start_allowed(task.robot)
    if not task.enabled:
        return Response({"detail": "TASK_DISABLED"}, status=status.HTTP_409_CONFLICT)
    if task.route is None:
        return Response({"detail": "TASK_ROUTE_MISSING"}, status=status.HTTP_409_CONFLICT)
    if task.robot.effective_connection_status() != "online":
        return Response(
            {"detail": "机器狗 Edge Agent 当前离线，无法立即执行巡检任务。请先启动 Edge Agent 并确认设备在线。"},
            status=status.HTTP_409_CONFLICT,
        )
    if task.robot.localization_status != "normal" or not task.robot.nav_ready:
        return Response(
            {"detail": "机器狗定位或导航栈未就绪，请先在路径规划页面启动导航栈，并等待定位状态变为 normal、Nav2 ready 后再执行。"},
            status=status.HTTP_409_CONFLICT,
        )
    return None


def parse_loop_execution_context(data) -> tuple[bool, uuid.UUID | None, int]:
    loop_execution = data.get("loop_execution", False)
    if not isinstance(loop_execution, bool):
        raise ValueError("loop_execution 必须是布尔值")
    if not loop_execution:
        return False, None, 1
    raw_session_id = data.get("loop_session_id")
    try:
        loop_session_id = uuid.UUID(str(raw_session_id)) if raw_session_id else uuid.uuid4()
    except (TypeError, ValueError, AttributeError) as exc:
        raise ValueError("loop_session_id 必须是 UUID") from exc
    try:
        round_number = int(data.get("round_number", 1))
    except (TypeError, ValueError) as exc:
        raise ValueError("round_number 必须是正整数") from exc
    if round_number < 1:
        raise ValueError("round_number 必须是正整数")
    return True, loop_session_id, round_number


class PatrolRouteExecuteView(APIView):
    """直接执行一条已保存路线。"""
    permission_classes = [permissions.AllowAny]

    def post(self, request, pk):
        record_rosbag = request.data.get("record_rosbag", False)
        if not isinstance(record_rosbag, bool):
            return Response({"detail": "record_rosbag 必须是布尔值"}, status=status.HTTP_400_BAD_REQUEST)
        try:
            loop_execution, loop_session_id, round_number = parse_loop_execution_context(request.data)
        except ValueError as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_400_BAD_REQUEST)
        route = get_object_or_404(
            PatrolRoute.objects.select_related("robot", "map_data"),
            pk=pk,
        )
        CommandService.ensure_task_start_allowed(route.robot)
        now = timezone.now()
        task_name = f"路线快速执行 - {route.name}"
        task = PatrolTask.objects.filter(route=route, name=task_name).first()
        if task is None:
            task = PatrolTask.objects.create(
                name=task_name,
                robot=route.robot,
                route=route,
                route_name=route.name,
                scheduled_start=now,
                scheduled_end=now + timezone.timedelta(hours=1),
                enabled=True,
                description="路径规划页面直接执行路线自动创建",
                created_by=request.user if request.user.is_authenticated else None,
            )
        else:
            task.robot = route.robot
            task.route_name = route.name
            task.scheduled_start = now
            task.scheduled_end = now + timezone.timedelta(hours=1)
            task.enabled = True
            task.save(update_fields=["robot", "route_name", "scheduled_start", "scheduled_end", "enabled", "updated_at"])

        readiness_error = validate_task_execution_readiness(task)
        if readiness_error is not None:
            return readiness_error

        try:
            # Keep the execution and task.start command atomic. Without this,
            # a dispatch precheck failure leaves an active `created` execution
            # and makes the next operator attempt fail as ROBOT_BUSY.
            with transaction.atomic():
                execution = TaskExecutionService.create_execution(
                    task,
                    request.user if request.user.is_authenticated else None,
                    loop_session_id=loop_session_id,
                    round_number=round_number,
                )
                CommandService.create(
                    execution,
                    "task.start",
                    request.user if request.user.is_authenticated else None,
                    command_options={
                        "record_rosbag": record_rosbag,
                        "loop_execution": loop_execution,
                    },
                )
        except TaskStateError as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_409_CONFLICT)
        execution.refresh_from_db()
        return Response(TaskExecutionSerializer(execution).data, status=status.HTTP_201_CREATED)


class ZoneListView(APIView):
    """禁区列表视图"""
    permission_classes = [permissions.AllowAny]

    def get(self, request):
        zones = Zone.objects.all()
        serializer = ZoneSerializer(zones, many=True)
        return Response(serializer.data)

    def post(self, request):
        serializer = ZoneSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        serializer.save()
        return Response(serializer.data, status=status.HTTP_201_CREATED)


class ZoneDetailView(APIView):
    """禁区详情视图"""
    permission_classes = [permissions.AllowAny]

    def get(self, request, pk):
        try:
            zone = Zone.objects.get(pk=pk)
            serializer = ZoneSerializer(zone)
            return Response(serializer.data)
        except Zone.DoesNotExist:
            return Response({"detail": "禁区不存在"}, status=status.HTTP_404_NOT_FOUND)

    def put(self, request, pk):
        try:
            zone = Zone.objects.get(pk=pk)
            serializer = ZoneSerializer(zone, data=request.data, partial=True)
            serializer.is_valid(raise_exception=True)
            serializer.save()
            return Response(serializer.data)
        except Zone.DoesNotExist:
            return Response({"detail": "禁区不存在"}, status=status.HTTP_404_NOT_FOUND)

    def delete(self, request, pk):
        try:
            zone = Zone.objects.get(pk=pk)
            zone.delete()
            return Response(status=status.HTTP_204_NO_CONTENT)
        except Zone.DoesNotExist:
            return Response({"detail": "禁区不存在"}, status=status.HTTP_404_NOT_FOUND)


class TrackListView(APIView):
    """轨迹记录列表视图"""
    def get(self, request):
        tracks = Track.objects.all()
        serializer = TrackSerializer(tracks, many=True)
        return Response(serializer.data)

    def post(self, request):
        serializer = TrackSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        serializer.save()
        return Response(serializer.data, status=status.HTTP_201_CREATED)


class TrackDetailView(APIView):
    """轨迹记录详情视图"""
    def get(self, request, pk):
        try:
            track = Track.objects.get(pk=pk)
            serializer = TrackSerializer(track)
            return Response(serializer.data)
        except Track.DoesNotExist:
            return Response({"detail": "轨迹不存在"}, status=status.HTTP_404_NOT_FOUND)

    def delete(self, request, pk):
        try:
            track = Track.objects.get(pk=pk)
            track.delete()
            return Response(status=status.HTTP_204_NO_CONTENT)
        except Track.DoesNotExist:
            return Response({"detail": "轨迹不存在"}, status=status.HTTP_404_NOT_FOUND)


class RobotConnectionView(APIView):
    """Deprecated P0 endpoint: devices must establish an outbound Edge connection."""

    def post(self, request):
        return Response(
            {"detail": "该接口已停用：机器狗必须通过 Edge Agent 主动连接中心平台"},
            status=status.HTTP_410_GONE,
        )


class RobotMapDownloadView(APIView):
    """Deprecated P0 endpoint: the center must not SSH into robots."""

    def post(self, request):
        return Response(
            {"detail": "该接口已停用：P0 不允许中心平台通过 SSH 主动访问机器狗"},
            status=status.HTTP_410_GONE,
        )


class CalendarDayListCreateView(APIView):
    def get(self, request):
        queryset = CalendarDay.objects.all()
        serializer = CalendarDaySerializer(queryset, many=True)
        return Response(serializer.data)

    def post(self, request):
        serializer = CalendarDaySerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        item = serializer.save()
        return Response(CalendarDaySerializer(item).data, status=status.HTTP_201_CREATED)


class CalendarDayDetailView(APIView):
    def get(self, request, pk):
        item = get_object_or_404(CalendarDay, pk=pk)
        return Response(CalendarDaySerializer(item).data)

    def put(self, request, pk):
        item = get_object_or_404(CalendarDay, pk=pk)
        serializer = CalendarDaySerializer(item, data=request.data, partial=True)
        serializer.is_valid(raise_exception=True)
        item = serializer.save()
        return Response(CalendarDaySerializer(item).data)

    def delete(self, request, pk):
        get_object_or_404(CalendarDay, pk=pk).delete()
        return Response(status=status.HTTP_204_NO_CONTENT)


class PatrolScheduleListCreateView(APIView):
    def get(self, request):
        queryset = PatrolSchedule.objects.select_related("robot", "task_template", "route", "map_data").all()
        robot = request.query_params.get("robot")
        schedule_type = request.query_params.get("schedule_type")
        enabled = request.query_params.get("enabled")
        if robot:
            queryset = queryset.filter(robot_id=robot)
        if schedule_type:
            queryset = queryset.filter(schedule_type=schedule_type)
        if enabled in {"true", "false"}:
            queryset = queryset.filter(enabled=(enabled == "true"))
        return Response(PatrolScheduleSerializer(queryset, many=True).data)

    def post(self, request):
        serializer = PatrolScheduleSerializer(data=request.data, context={"request": request})
        serializer.is_valid(raise_exception=True)
        schedule = serializer.save()
        return Response(PatrolScheduleSerializer(schedule).data, status=status.HTTP_201_CREATED)


class PatrolScheduleDetailView(APIView):
    def get(self, request, pk):
        schedule = get_object_or_404(PatrolSchedule, pk=pk)
        return Response(PatrolScheduleSerializer(schedule).data)

    def put(self, request, pk):
        schedule = get_object_or_404(PatrolSchedule, pk=pk)
        serializer = PatrolScheduleSerializer(schedule, data=request.data, partial=True, context={"request": request})
        serializer.is_valid(raise_exception=True)
        schedule = serializer.save()
        return Response(PatrolScheduleSerializer(schedule).data)

    def delete(self, request, pk):
        get_object_or_404(PatrolSchedule, pk=pk).delete()
        return Response(status=status.HTTP_204_NO_CONTENT)


class PatrolScheduleEnableView(APIView):
    enabled = True

    def post(self, request, pk):
        schedule = get_object_or_404(PatrolSchedule, pk=pk)
        schedule.enabled = self.enabled
        schedule.save(update_fields=["enabled", "updated_at"])
        return Response(PatrolScheduleSerializer(schedule).data)


class PatrolScheduleDisableView(PatrolScheduleEnableView):
    enabled = False


class PatrolScheduleRunNowView(APIView):
    def post(self, request, pk):
        schedule = get_object_or_404(
            PatrolSchedule.objects.select_related("robot", "task_template", "route", "map_data"),
            pk=pk,
        )
        run = ScheduleService.trigger_now(schedule, operator=request.user if request.user.is_authenticated else None)
        return Response(ScheduleRunSerializer(run).data, status=status.HTTP_202_ACCEPTED)


class PatrolCalendarView(APIView):
    def get(self, request):
        today = timezone.localdate()
        start = parse_date(request.query_params.get("from", "")) or today
        end = parse_date(request.query_params.get("to", "")) or (start + timedelta(days=6))
        if end < start:
            return Response({"detail": "to 必须晚于或等于 from"}, status=status.HTTP_400_BAD_REQUEST)
        if (end - start).days > 62:
            return Response({"detail": "单次最多查询 63 天"}, status=status.HTTP_400_BAD_REQUEST)
        robot = request.query_params.get("robot")
        payload = ScheduleService.calendar_payload(start, end, robot_id=int(robot) if robot else None)
        return Response(payload)


class ScheduleRunListView(APIView):
    def get(self, request):
        queryset = ScheduleRun.objects.select_related(
            "schedule", "schedule__task_template", "schedule__route", "robot", "task_execution"
        ).all()
        start = parse_date(request.query_params.get("from", ""))
        end = parse_date(request.query_params.get("to", ""))
        status_filter = request.query_params.get("status")
        robot = request.query_params.get("robot")
        if start:
            queryset = queryset.filter(planned_start_at__date__gte=start)
        if end:
            queryset = queryset.filter(planned_start_at__date__lte=end)
        if status_filter:
            queryset = queryset.filter(status=status_filter)
        if robot:
            queryset = queryset.filter(robot_id=robot)
        return Response(ScheduleRunSerializer(queryset[:200], many=True).data)


class PatrolTaskListCreateView(APIView):
    permission_classes = [permissions.AllowAny]

    def get(self, request):
        queryset = PatrolTask.objects.select_related("robot", "route", "route__map_data").all()
        return Response(PatrolTaskSerializer(queryset, many=True).data)

    def post(self, request):
        serializer = PatrolTaskCreateSerializer(data=request.data, context={"request": request})
        serializer.is_valid(raise_exception=True)
        task = serializer.save()
        return Response(PatrolTaskSerializer(task).data, status=status.HTTP_201_CREATED)


class PatrolTaskDetailView(APIView):
    permission_classes = [permissions.AllowAny]

    def get(self, request, task_id):
        task = get_object_or_404(
            PatrolTask.objects.select_related("robot", "route", "route__map_data"),
            pk=task_id,
        )
        return Response(PatrolTaskSerializer(task).data)

    def put(self, request, task_id):
        task = get_object_or_404(PatrolTask, pk=task_id)
        serializer = PatrolTaskCreateSerializer(
            task,
            data=request.data,
            partial=True,
            context={"request": request},
        )
        serializer.is_valid(raise_exception=True)
        route = serializer.validated_data.get("route", task.route)
        task = serializer.save(route_name=route.name if route else task.route_name)
        return Response(PatrolTaskSerializer(task).data)

    def delete(self, request, task_id):
        task = get_object_or_404(PatrolTask, pk=task_id)
        try:
            if request_force_delete(request):
                counts = _force_delete_patrol_task(task)
                return Response(
                    {"detail": "巡检任务及关联数据已强制删除。", "deleted": counts},
                    status=status.HTTP_200_OK,
                )
            task.delete()
            return Response(status=status.HTTP_204_NO_CONTENT)
        except ProtectedError as exc:
            return Response(
                {
                    "detail": str(exc.args[0])
                    if exc.args
                    else "该巡检任务已有执行记录或日历计划引用，不能直接删除。请先删除相关计划；历史执行记录保留用于追溯。",
                    "references": _task_force_delete_counts(task),
                },
                status=status.HTTP_409_CONFLICT,
            )


class PatrolTaskExecuteView(APIView):
    permission_classes = [permissions.AllowAny]

    def post(self, request, task_id):
        record_rosbag = request.data.get("record_rosbag", False)
        if not isinstance(record_rosbag, bool):
            return Response({"detail": "record_rosbag 必须是布尔值"}, status=status.HTTP_400_BAD_REQUEST)
        try:
            loop_execution, loop_session_id, round_number = parse_loop_execution_context(request.data)
        except ValueError as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_400_BAD_REQUEST)
        task = get_object_or_404(
            PatrolTask.objects.select_related("robot", "route", "route__map_data"),
            pk=task_id,
        )
        readiness_error = validate_task_execution_readiness(task)
        if readiness_error is not None:
            return readiness_error
        try:
            operator = request.user if request.user.is_authenticated else None
            # See the route execution endpoint: a rejected command must roll
            # back the just-created execution rather than orphaning it.
            with transaction.atomic():
                execution = TaskExecutionService.create_execution(
                    task,
                    operator,
                    loop_session_id=loop_session_id,
                    round_number=round_number,
                )
                CommandService.create(
                    execution,
                    "task.start",
                    operator,
                    command_options={
                        "record_rosbag": record_rosbag,
                        "loop_execution": loop_execution,
                    },
                )
        except TaskStateError as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_409_CONFLICT)
        execution.refresh_from_db()
        return Response(TaskExecutionSerializer(execution).data, status=status.HTTP_201_CREATED)


class TaskExecutionDetailView(APIView):
    permission_classes = [permissions.AllowAny]

    def get(self, request, execution_id):
        execution = get_object_or_404(
            TaskExecution.objects.select_related("task", "robot", "route", "map_data").prefetch_related(
                "events", "commands__events"
            ),
            pk=execution_id,
        )
        return Response(TaskExecutionSerializer(execution).data)


class TaskExecutionCommandView(APIView):
    permission_classes = [permissions.AllowAny]
    command_type = ""

    def post(self, request, execution_id):
        serializer = TaskExecutionActionSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        execution = get_object_or_404(TaskExecution, pk=execution_id)
        try:
            operator = request.user if request.user.is_authenticated else None
            command = CommandService.create(execution, self.command_type, operator)
        except TaskStateError as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_409_CONFLICT)
        execution.refresh_from_db()
        data = TaskExecutionSerializer(execution).data
        data["created_command_id"] = str(command.id)
        return Response(data, status=status.HTTP_202_ACCEPTED)


class TaskExecutionPauseView(TaskExecutionCommandView):
    command_type = "task.pause"


class TaskExecutionResumeView(TaskExecutionCommandView):
    command_type = "task.resume"


class TaskExecutionCancelView(TaskExecutionCommandView):
    command_type = "task.cancel"


class TaskExecutionForceExitView(TaskExecutionCommandView):
    command_type = "task.force_exit"


class TaskExecutionTrajectoryView(APIView):
    permission_classes = [permissions.AllowAny]

    def get(self, request, execution_id):
        execution = get_object_or_404(TaskExecution, pk=execution_id)
        points = TrajectoryPoint.objects.filter(task_execution=execution).order_by("seq")
        return Response(
            {
                "task_execution_id": str(execution.id),
                "count": points.count(),
                "points": TrajectoryPointSerializer(points, many=True).data,
            }
        )


class RobotStatusView(APIView):
    def get(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        latest = RobotStatusLatest.objects.filter(robot=robot).first()
        return Response(
            {
                "robot_id": robot.id,
                "robot_code": robot.code,
                "connection_status": robot.effective_connection_status(),
                "raw_connection_status": robot.connection_status,
                "last_seen_at": robot.last_seen_at,
                "status": RobotStatusSerializer(latest).data if latest else None,
            }
        )


class RobotSessionListView(APIView):
    def get(self, request, robot_id):
        robot = get_object_or_404(Robot, pk=robot_id)
        sessions = RobotSession.objects.filter(robot=robot).order_by("-connected_at")[:100]
        return Response(RobotSessionSerializer(sessions, many=True).data)


class AlertTimelineView(APIView):
    def get(self, request, event_id):
        event = get_object_or_404(InspectionEvent, event_id=event_id)
        payload = AlertService.build_timeline(event)
        return Response(AlertTimelineSerializer(payload).data)
