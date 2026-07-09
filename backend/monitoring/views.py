import hashlib
import io
import json
import zipfile
from datetime import time as datetime_time, timedelta
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from django.conf import settings
from django.contrib.auth import authenticate, get_user_model
from django.core.files.base import ContentFile
from django.http import HttpResponse, HttpResponseForbidden, StreamingHttpResponse
from django.db import transaction
from django.db.models import Case, Count, IntegerField, Q, When
from django.db.models.deletion import ProtectedError
from django.shortcuts import get_object_or_404
from django.utils.dateparse import parse_date, parse_datetime
from django.utils import timezone
from rest_framework import permissions, status
from rest_framework.authtoken.models import Token
from rest_framework.decorators import api_view, permission_classes
from rest_framework.response import Response
from rest_framework.views import APIView

from .models import (
    InspectionEvent,
    CalendarDay,
    MediaAsset,
    PatrolTask,
    PatrolSchedule,
    Robot,
    RobotCommand,
    RobotSession,
    RobotStatusLatest,
    RobotTelemetry,
    TaskExecution,
    ScheduleRun,
    TrajectoryPoint,
    MapData,
    PatrolRoute,
    Zone,
    Track,
)
from .permissions import IsAuthenticatedOrDeviceCredential
from .realtime import event_broker, sse_stream
from .services.alert_service import AlertService
from .services.command_service import CommandService
from .services.schedule_service import ScheduleService
from .services.task_service import TaskExecutionService, TaskStateError
from .serializers import (
    EventSerializer,
    CalendarDaySerializer,
    MediaAssetSerializer,
    MediaUploadSerializer,
    PatrolTaskSerializer,
    PatrolTaskCreateSerializer,
    PatrolScheduleSerializer,
    RobotCommandCreateSerializer,
    RobotCommandSerializer,
    RobotDetailSerializer,
    RobotSerializer,
    RemoteCommandSerializer,
    TelemetryIngestSerializer,
    MapDataSerializer,
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

User = get_user_model()


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
    local_image_path = description.get("image", "") if isinstance(description, dict) else ""
    local_map_dir = local_image_path.rsplit("/", 1)[0] if local_image_path and "/" in local_image_path else ""
    if not local_map_dir and map_data.name:
        # Older uploaded maps did not store the edge-local image path in
        # description. Their name is the edge session directory, e.g.
        # 20260703_170635, so infer the local source directory from it.
        local_map_dir = f"/home/robot/.jszr/map/{map_data.name}"
        local_image_path = f"{local_map_dir}/map.pgm"
    map_version = f"legacy-mapdata-{map_data.id}"
    return {
        "map_id": str(map_data.id),
        "map_version": map_version,
        "map_name": map_data.name,
        "source_map_version": description.get("map_version", "") if isinstance(description, dict) else "",
        "local_map_dir": local_map_dir,
        "local_image_path": local_image_path,
        "pgm_url": request.build_absolute_uri(map_data.pgm_file.url) if map_data.pgm_file else "",
        "yaml_url": request.build_absolute_uri(map_data.yaml_file.url) if map_data.yaml_file else "",
        "resolution": map_data.resolution,
        "origin": map_data.origin,
        "width": map_data.width,
        "height": map_data.height,
    }


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
    files = [map_data.pgm_file, map_data.yaml_file, map_data.thumbnail]
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
    tasks = PatrolTask.objects.all()
    events = InspectionEvent.objects.all()

    risk_weight_map = {"high": 3, "medium": 2, "low": 1}
    risk_totals = {date: 0 for date in dates}
    for event in events.only("detected_at", "risk_level"):
        event_date = timezone.localtime(event.detected_at).date()
        if event_date in risk_totals:
            risk_totals[event_date] += risk_weight_map.get(event.risk_level, 1)

    labels = [date.strftime("%m-%d") for date in dates]
    alert_values = [1, 2, 0, 3, 2, 1, 4]
    detection_values = [5, 6, 4, 7, 6, 5, 8]
    duration_values = [26, 28, 24, 32, 27, 29, 30]
    mileage_values = [0, 0, 0, 0, 0, 0, 1.3]

    alert_series = [{"label": label, "value": value} for label, value in zip(labels, alert_values)]
    detection_series = [{"label": label, "value": value} for label, value in zip(labels, detection_values)]
    duration_series = [{"label": label, "value": value} for label, value in zip(labels, duration_values)]
    mileage_series = [{"label": label, "value": value} for label, value in zip(labels, mileage_values)]

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
                "note": "结合事件上报与遥测活跃度的综合识别次数",
            },
            {
                "title": "平均完成度",
                "value": f"{round(sum(task.completion_rate for task in tasks) / len(tasks)) if tasks else 0}%",
                "note": "任务执行进度持续稳定，适合持续追踪",
            },
            {
                "title": "值守响应",
                "value": "30 分钟",
                "note": f"当前在线设备 {Robot.objects.filter(status='online').count()} 台，处置链路保持畅通",
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


def ensure_demo_seed() -> None:
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
                "flv": "http://39.107.250.69:8090/live/dog_ZSL-1A-07_front.live.flv",
                "hls": "http://39.107.250.69:8090/live/dog_ZSL-1A-07_front/hls.m3u8",
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
    demo_route, _ = PatrolRoute.objects.get_or_create(
        name="南门-主步道-牡丹园-活动广场",
        robot=robot,
        map_data=demo_map,
        defaults={
            "waypoints": [
                {"x": 0.0, "y": 0.0, "yaw": 0.0, "name": "南门"},
                {"x": 2.0, "y": 1.0, "yaw": 0.0, "name": "主步道"},
                {"x": 4.0, "y": 2.0, "yaw": 0.0, "name": "牡丹园"},
                {"x": 6.0, "y": 3.0, "yaw": 0.0, "name": "活动广场"},
            ],
            "waypoint_names": ["南门", "主步道", "牡丹园", "活动广场"],
            "description": "演示预置路线，现场建图后应重新绑定航点",
        },
    )
    task, created_task = PatrolTask.objects.get_or_create(
        name="公园主通道早间巡检",
        robot=robot,
        defaults={
            "route_name": demo_route.name,
            "scheduled_start": timezone.now() - timezone.timedelta(hours=2),
            "scheduled_end": timezone.now() + timezone.timedelta(hours=1),
            "status": "running",
            "completion_rate": 68,
            "route": demo_route,
        },
    )
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
        latest_robot = robots.first()
        latest_event = events.first()
        return Response(
            {
                "summary": {
                    "online_robot_count": robots.filter(status="online").count(),
                    "today_alert_count": latest_robot.today_alerts if latest_robot else today_events.count(),
                    "pending_event_count": events.filter(status="pending").count(),
                    "resolved_event_count": events.filter(status="resolved").count(),
                    "completed_task_count": PatrolTask.objects.filter(status="completed").count(),
                },
                "header": {
                    "device_code": latest_robot.code if latest_robot else "--",
                    "current_mode": latest_robot.get_mode_display() if latest_robot else "--",
                    "current_location": latest_robot.location if latest_robot else "--",
                    "today_alerts": latest_robot.today_alerts if latest_robot else 0,
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
        "move_forward": "teleop.move_forward",
        "move_backward": "teleop.move_backward",
        "move_left": "teleop.move_left",
        "move_right": "teleop.move_right",
        "turn_left": "teleop.turn_left",
        "turn_right": "teleop.turn_right",
        "move_stop": "teleop.move_stop",
        "passive": "teleop.passive",
    }

    def post(self, request, robot_id):
        ensure_demo_seed()
        robot = Robot.objects.get(id=robot_id)
        serializer = RobotCommandCreateSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        action = serializer.validated_data["action"]
        payload = serializer.validated_data.get("payload") or {}
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
                expiry_seconds=5,
            )
            return Response(RemoteCommandSerializer(command).data, status=status.HTTP_202_ACCEPTED)

        command = RobotCommand.objects.create(
            robot=robot,
            action=action,
            payload=payload,
        )

        response_payload, error_message = dispatch_robot_command(command)
        command.sent_at = timezone.now()
        command.response_payload = response_payload
        command.error_message = error_message
        command.status = "failed" if error_message else "sent"
        command.save(update_fields=["sent_at", "response_payload", "error_message", "status", "updated_at"])

        response_status = status.HTTP_201_CREATED if command.status == "sent" else status.HTTP_502_BAD_GATEWAY
        return Response(RobotCommandSerializer(command).data, status=response_status)


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

        return Response(
            {"detail": "上报成功", "robot_id": robot.id, "telemetry_id": telemetry.id, "duplicate": False},
            status=status.HTTP_201_CREATED,
        )


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
            files = [map_data.pgm_file, map_data.yaml_file, map_data.thumbnail]
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
            
            if map_data.thumbnail:
                with open(map_data.thumbnail.path, 'rb') as f:
                    content = f.read()
                return self._png_response(content)
            
            if not map_data.pgm_file:
                return self._png_response(self._generate_placeholder_image())
            
            pgm_path = map_data.pgm_file.path
            png_data = self._pgm_to_png(pgm_path)
            
            return self._png_response(png_data)
            
        except MapData.DoesNotExist:
            return self._png_response(self._generate_placeholder_image())
        except Exception as e:
            print(f"Error generating preview: {e}")
            return self._png_response(self._generate_placeholder_image())

    def _pgm_to_png(self, pgm_path):
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

        max_size = 1200
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

        # 从 result_payload 提取 edge_agent 返回的真实 mapping state
        mapping_state = "idle"
        mapping_result = {}
        if command and command.result_payload:
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
        latest_status = RobotStatusLatest.objects.filter(robot=robot).first()
        robot_current_map = {}
        if latest_status and latest_status.raw_payload:
            robot_current_map = latest_status.raw_payload.get("current_map") or {}
        if not robot_current_map:
            robot_current_map = {
                "map_id": robot.current_map_id,
                "map_version": robot.current_map_version,
            }
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
                "command_status": command.status if command else "idle",
                "command_id": str(command.id) if command else None,
                "issued_at": command.issued_at if command else None,
                "acknowledged_at": command.acknowledged_at if command else None,
                "finished_at": command.finished_at if command else None,
                "error_code": command.error_code if command else "",
                "error_message": command.error_message if command else "",
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
        return {
            "map_name": map_name,
            "route_hint": request.data.get("route_hint", ""),
            "operator_note": request.data.get("operator_note", ""),
        }


class RobotMappingSaveView(RobotMappingCommandView):
    command_type = "mapping.save"
    expiry_seconds = 300

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
            command_type__in=["nav.start", "nav.restart", "nav.recover", "nav.stop", "nav.initial_pose"]
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
        try:
            x = float(request.data.get("x"))
            y = float(request.data.get("y"))
            yaw = float(request.data.get("yaw", 0))
        except (TypeError, ValueError):
            return Response({"detail": "初始定位需要数值 x、y、yaw。"}, status=status.HTTP_400_BAD_REQUEST)
        command = CommandService.create_robot_command(
            robot=robot,
            command_type="nav.initial_pose",
            payload={
                "reason": "manual_initial_pose",
                "frame_id": request.data.get("frame_id") or "map",
                "x": x,
                "y": y,
                "yaw": yaw,
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
        try:
            with zipfile.ZipFile(io.BytesIO(package.read())) as archive:
                for name in archive.namelist():
                    basename = name.rsplit("/", 1)[-1]
                    if basename in {"map.yaml", "map.pgm", "map_preview.png", "preview.png"}:
                        extracted[basename] = archive.read(name)
        except zipfile.BadZipFile:
            return Response({"detail": "map_package 不是合法 zip"}, status=status.HTTP_400_BAD_REQUEST)

        if "map.yaml" not in extracted or "map.pgm" not in extracted:
            return Response({"detail": "地图包必须包含 map.yaml 和 map.pgm"}, status=status.HTTP_400_BAD_REQUEST)

        yaml_metadata = _parse_simple_map_yaml(extracted["map.yaml"])
        width, height = _read_pgm_dimensions(extracted["map.pgm"])
        description = {
            "source": "edge_mapping",
            "map_version": metadata.get("map_version", ""),
            "mapping_session_id": metadata.get("mapping_session_id", ""),
            "source_map_dir": metadata.get("source_map_dir", ""),
            "dynamic_filter": metadata.get("dynamic_filter", {}),
            "route_hint": metadata.get("route_hint", ""),
            "files": metadata.get("files", []),
            "image": yaml_metadata.get("image", ""),
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
            )
            map_data.yaml_file.save(f"{map_data.id}_map.yaml", ContentFile(extracted["map.yaml"]), save=False)
            map_data.pgm_file.save(f"{map_data.id}_map.pgm", ContentFile(extracted["map.pgm"]), save=False)
            preview = extracted.get("map_preview.png") or extracted.get("preview.png")
            if preview:
                map_data.thumbnail.save(f"{map_data.id}_preview.png", ContentFile(preview), save=False)
            map_data.save()
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


class PatrolRouteExecuteView(APIView):
    """直接执行一条已保存路线。"""
    permission_classes = [permissions.AllowAny]

    def post(self, request, pk):
        route = get_object_or_404(
            PatrolRoute.objects.select_related("robot", "map_data"),
            pk=pk,
        )
        if route.robot.effective_connection_status() != "online":
            return Response(
                {"detail": "机器狗 Edge Agent 当前离线，无法立即执行路线。请先启动 Edge Agent 并确认设备在线。"},
                status=status.HTTP_409_CONFLICT,
            )
        if route.robot.localization_status != "normal" or not route.robot.nav_ready:
            return Response(
                {"detail": "机器狗定位或导航栈未就绪，请先在路径规划页面启动导航栈，并等待定位状态变为 normal、Nav2 ready 后再执行。"},
                status=status.HTTP_409_CONFLICT,
            )

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

        try:
            execution = TaskExecutionService.create_execution(
                task,
                request.user if request.user.is_authenticated else None,
            )
            CommandService.create(
                execution,
                "task.start",
                request.user if request.user.is_authenticated else None,
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
        task = get_object_or_404(
            PatrolTask.objects.select_related("robot", "route", "route__map_data"),
            pk=task_id,
        )
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
        try:
            operator = request.user if request.user.is_authenticated else None
            execution = TaskExecutionService.create_execution(task, operator)
            CommandService.create(execution, "task.start", operator)
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
