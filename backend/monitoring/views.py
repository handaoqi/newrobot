import hashlib
import io
import json
from uuid import uuid4
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from django.conf import settings
from django.contrib.auth import authenticate, get_user_model
from django.core.files.storage import default_storage
from django.http import HttpResponse, HttpResponseForbidden, StreamingHttpResponse
from django.db.models import Case, Count, IntegerField, Q, When
from django.utils.dateparse import parse_datetime
from django.utils import timezone
from rest_framework import permissions, status
from rest_framework.authtoken.models import Token
from rest_framework.decorators import api_view, permission_classes
from rest_framework.response import Response
from rest_framework.views import APIView

from .models import InspectionEvent, MediaAsset, PatrolTask, Robot, RobotCommand, RobotTelemetry, MapData, PatrolRoute, Zone, Track
from .realtime import event_broker, sse_stream
from .serializers import (
    EventSerializer,
    MediaAssetSerializer,
    MediaUploadSerializer,
    PatrolTaskSerializer,
    RobotCommandCreateSerializer,
    RobotCommandSerializer,
    RobotDetailSerializer,
    RobotSerializer,
    TelemetryIngestSerializer,
    MapDataSerializer,
    PatrolRouteSerializer,
    ZoneSerializer,
    TrackSerializer,
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
                "flv": "/live/dog_ZSL-1A-07_front.live.flv",
                "hls": "/live/dog_ZSL-1A-07_front/hls.m3u8",
            },
        },
    )
    if not robot.stream_id:
        robot.stream_id = "dog_ZSL-1A-07_front"
    robot.save(update_fields=["stream_id", "play_urls", "updated_at"])
    if not PatrolTask.objects.filter(name="公园主通道早间巡检", robot=robot).exists():
        PatrolTask.objects.create(
            name="公园主通道早间巡检",
            robot=robot,
            route_name="南门-主路-中心广场",
            scheduled_start=timezone.now() - timezone.timedelta(hours=2),
            scheduled_end=timezone.now() + timezone.timedelta(hours=1),
            status="running",
            completion_rate=68,
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
    def get(self, request):
        ensure_demo_seed()
        return Response(RobotSerializer(Robot.objects.all(), many=True).data)


class RobotDetailView(APIView):
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
                return Response({"detail": "audio_url 必须是可下载的 HTTP(S) 地址"}, status=status.HTTP_400_BAD_REQUEST)

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


class RobotAudioRecordingCommandView(APIView):
    def post(self, request, robot_id):
        ensure_demo_seed()
        robot = Robot.objects.get(id=robot_id)
        uploaded_file = request.FILES.get("file")
        if uploaded_file is None:
            return Response({"detail": "录音文件不能为空"}, status=status.HTTP_400_BAD_REQUEST)
        if uploaded_file.size <= 0:
            return Response({"detail": "录音文件为空"}, status=status.HTTP_400_BAD_REQUEST)
        if uploaded_file.size > 10 * 1024 * 1024:
            return Response({"detail": "录音文件不能超过 10MB"}, status=status.HTTP_400_BAD_REQUEST)

        content_type = (uploaded_file.content_type or "").lower()
        extension_by_type = {
            "audio/webm": ".webm",
            "audio/ogg": ".ogg",
            "audio/mpeg": ".mp3",
            "audio/mp3": ".mp3",
            "audio/wav": ".wav",
            "audio/x-wav": ".wav",
            "audio/mp4": ".m4a",
        }
        extension = extension_by_type.get(content_type)
        if extension is None:
            original_name = uploaded_file.name or ""
            extension = "." + original_name.rsplit(".", 1)[-1].lower() if "." in original_name else ".webm"
        if extension not in {".webm", ".ogg", ".mp3", ".wav", ".m4a", ".aac"}:
            return Response({"detail": "不支持的录音格式"}, status=status.HTTP_400_BAD_REQUEST)

        relative_path = timezone.now().strftime("command-audio/%Y/%m/%d/")
        file_name = f"{uuid4().hex}{extension}"
        saved_path = default_storage.save(relative_path + file_name, uploaded_file)
        audio_url = request.build_absolute_uri(settings.MEDIA_URL + saved_path)
        command = RobotCommand.objects.create(
            robot=robot,
            action="play_audio",
            payload={
                "audio_url": audio_url,
                "audio_name": request.data.get("audio_name") or "现场录音",
                "source": "dashboard_recording",
                "content_type": content_type,
                "file_size": uploaded_file.size,
            },
        )
        return Response(
            {"audio_url": audio_url, "command": RobotCommandSerializer(command).data},
            status=status.HTTP_201_CREATED,
        )


class DeviceCommandPollView(APIView):
    permission_classes = [permissions.AllowAny]

    def get(self, request):
        robot_code = (
            request.query_params.get("robot_code")
            or request.headers.get("X-Device-Code")
            or ""
        ).strip()
        if not robot_code:
            return Response({"detail": "robot_code is required"}, status=status.HTTP_400_BAD_REQUEST)

        try:
            robot = Robot.objects.get(code=robot_code)
        except Robot.DoesNotExist:
            return Response({"detail": "robot not found"}, status=status.HTTP_404_NOT_FOUND)

        command = (
            RobotCommand.objects.filter(robot=robot, action="play_audio", status="queued")
            .order_by("created_at")
            .first()
        )
        if command is None:
            return Response(status=status.HTTP_204_NO_CONTENT)

        command.status = "sent"
        command.sent_at = timezone.now()
        command.save(update_fields=["status", "sent_at", "updated_at"])
        return Response(RobotCommandSerializer(command).data)


class DeviceCommandReportView(APIView):
    permission_classes = [permissions.AllowAny]

    def post(self, request, command_id):
        status_value = str(request.data.get("status", "")).strip()
        if status_value not in {"running", "finished", "failed"}:
            return Response({"detail": "status must be running, finished, or failed"}, status=status.HTTP_400_BAD_REQUEST)

        try:
            command = RobotCommand.objects.select_related("robot").get(id=command_id, action="play_audio")
        except RobotCommand.DoesNotExist:
            return Response({"detail": "command not found"}, status=status.HTTP_404_NOT_FOUND)

        device_code = (request.headers.get("X-Device-Code") or request.data.get("robot_code") or "").strip()
        if device_code and device_code != command.robot.code:
            return Response({"detail": "robot_code mismatch"}, status=status.HTTP_403_FORBIDDEN)

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
        event.save(update_fields=["status", "handling_notes", "review_result", "updated_at"])
        return Response(EventSerializer(event, context={"request": request}).data)


class TaskListView(APIView):
    def get(self, request):
        ensure_demo_seed()
        return Response(PatrolTaskSerializer(PatrolTask.objects.select_related("robot").all(), many=True).data)


class TelemetryIngestView(APIView):
    permission_classes = [permissions.AllowAny]

    def post(self, request):
        ensure_demo_seed()
        serializer = TelemetryIngestSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        payload = serializer.validated_data
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
    permission_classes = [permissions.AllowAny]

    def post(self, request):
        serializer = MediaUploadSerializer(data=request.data)
        serializer.is_valid(raise_exception=True)
        payload = serializer.validated_data
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
        )
        asset.url = settings.MEDIA_URL + asset.file.name
        asset.save(update_fields=["url", "updated_at"])
        return Response({"url": asset.url, "asset": MediaAssetSerializer(asset).data}, status=status.HTTP_201_CREATED)


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
            map_data.delete()
            return Response(status=status.HTTP_204_NO_CONTENT)
        except MapData.DoesNotExist:
            return Response({"detail": "地图不存在"}, status=status.HTTP_404_NOT_FOUND)


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
            map_data = MapData.objects.get(pk=pk)
            MapData.objects.filter(active=True).update(active=False)
            map_data.active = True
            map_data.save()
            serializer = MapDataSerializer(map_data, context={"request": request})
            return Response(serializer.data)
        except MapData.DoesNotExist:
            return Response({"detail": "地图不存在"}, status=status.HTTP_404_NOT_FOUND)


class MapDataPreviewView(APIView):
    """PGM地图预览视图 - 将PGM文件转换为可预览的PNG图像"""
    permission_classes = [permissions.AllowAny]

    def get(self, request, pk):
        try:
            map_data = MapData.objects.get(pk=pk)
            
            if map_data.thumbnail:
                with open(map_data.thumbnail.path, 'rb') as f:
                    content = f.read()
                return HttpResponse(content, content_type='image/png')
            
            if not map_data.pgm_file:
                return HttpResponse(self._generate_placeholder_image(), content_type='image/png')
            
            pgm_path = map_data.pgm_file.path
            png_data = self._pgm_to_png(pgm_path)
            
            return HttpResponse(png_data, content_type='image/png')
            
        except MapData.DoesNotExist:
            return HttpResponse(self._generate_placeholder_image(), content_type='image/png')
        except Exception as e:
            print(f"Error generating preview: {e}")
            return HttpResponse(self._generate_placeholder_image(), content_type='image/png')

    def _pgm_to_png(self, pgm_path):
        """将PGM文件转换为PNG格式"""
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
        
        max_size = 300
        scale = min(max_size / width, max_size / height, 1.0)
        scaled_width = int(width * scale)
        scaled_height = int(height * scale)
        
        return self._create_png(scaled_width, scaled_height, raw_data, width, max_val)

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
        """生成占位符图像"""
        import zlib
        
        signature = b'\x89PNG\r\n\x1a\n'
        
        ihdr_data = b'\x00\x00\x00\x01\x00\x00\x00\x01\x08\x00\x00\x00\x00'
        ihdr = self._create_chunk(b'IHDR', ihdr_data)
        
        filtered_data = b'\x00\x80'
        compressed = zlib.compress(filtered_data)
        idat = self._create_chunk(b'IDAT', compressed)
        
        iend = self._create_chunk(b'IEND', b'')
        
        return signature + ihdr + idat + iend


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
    permission_classes = [permissions.AllowAny]

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
    permission_classes = [permissions.AllowAny]

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
    """机器狗连接视图 - 连接机器狗并获取地图"""
    permission_classes = [permissions.AllowAny]

    def post(self, request):
        """连接机器狗"""
        try:
            import paramiko
            from pathlib import Path
            import os

            ip = request.data.get('ip')
            username = request.data.get('username')
            password = request.data.get('password')

            if not all([ip, username, password]):
                return Response(
                    {"error": "请提供完整的连接信息"},
                    status=status.HTTP_400_BAD_REQUEST
                )

            # 创建SSH连接
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(ip, username=username, password=password, timeout=10)

            # 获取地图目录列表
            stdin, stdout, stderr = ssh.exec_command(
                'find ~/.jszr/map -maxdepth 1 -type d | sort'
            )
            output = stdout.read().decode('utf-8')
            directories = [line.strip() for line in output.strip().split('\n') if line.strip()]

            # 获取每个目录的地图信息
            maps = []
            for dir_path in directories:
                if not dir_path or dir_path.endswith('.jszr/map'):
                    continue

                map_name = os.path.basename(dir_path)
                map_info = {
                    'name': map_name,
                    'path': dir_path,
                    'files': []
                }

                # 列出目录中的文件
                stdin, stdout, stderr = ssh.exec_command(f'ls -lh "{dir_path}"')
                output = stdout.read().decode('utf-8')
                for line in output.strip().split('\n'):
                    if line:
                        parts = line.split()
                        if len(parts) >= 9:
                            size = parts[4]
                            filename = ' '.join(parts[8:])
                            map_info['files'].append({
                                'name': filename,
                                'size': size
                            })

                maps.append(map_info)

            ssh.close()

            return Response({
                'connected': True,
                'robot_ip': ip,
                'maps': maps,
                'total': len(maps)
            })

        except paramiko.AuthenticationException:
            return Response(
                {"error": "认证失败，请检查用户名和密码"},
                status=status.HTTP_401_UNAUTHORIZED
            )
        except paramiko.SSHException as e:
            return Response(
                {"error": f"SSH连接失败: {str(e)}"},
                status=status.HTTP_500_INTERNAL_SERVER_ERROR
            )
        except Exception as e:
            return Response(
                {"error": f"连接失败: {str(e)}"},
                status=status.HTTP_500_INTERNAL_SERVER_ERROR
            )


class RobotMapDownloadView(APIView):
    """从机器狗下载地图"""
    permission_classes = [permissions.AllowAny]

    def post(self, request):
        """下载地图文件"""
        try:
            import paramiko
            from pathlib import Path
            import os
            import shutil
            from datetime import datetime

            ip = request.data.get('ip')
            username = request.data.get('username')
            password = request.data.get('password')
            map_path = request.data.get('map_path')  # 狗上的地图路径
            map_name = request.data.get('map_name')  # 地图名称

            if not all([ip, username, password, map_path, map_name]):
                return Response(
                    {"error": "缺少必要参数"},
                    status=status.HTTP_400_BAD_REQUEST
                )

            # 创建SSH连接
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(ip, username=username, password=password, timeout=30)

            # 创建本地地图存储目录
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            local_map_dir = Path(settings.MEDIA_ROOT) / 'maps' / map_name
            local_map_dir.mkdir(parents=True, exist_ok=True)

            # 获取SFTP连接
            sftp = ssh.open_sftp()

            # 下载地图文件（跳过大的pcd文件）
            downloaded_files = []
            skipped_files = []

            # 先列出所有文件
            stdin, stdout, stderr = ssh.exec_command(f'ls "{map_path}"')
            files = stdout.read().decode('utf-8').strip().split('\n')

            for filename in files:
                if not filename:
                    continue

                remote_path = f'{map_path}/{filename}'
                local_file_path = local_map_dir / filename

                try:
                    # 跳过大的pcd文件
                    if filename.endswith('.pcd'):
                        stdin, stdout, stderr = ssh.exec_command(f'stat -c%s "{remote_path}"')
                        file_size = int(stdout.read().decode('utf-8').strip())

                        if file_size > 50 * 1024 * 1024:  # 大于50MB的文件跳过
                            skipped_files.append(filename)
                            continue

                    # 下载文件
                    sftp.get(remote_path, str(local_file_path))
                    downloaded_files.append(filename)

                except Exception as e:
                    print(f'下载文件失败 {filename}: {e}')
                    continue

            sftp.close()
            ssh.close()

            # 保存到数据库
            from .models import MapData

            # 检查是否已存在
            if MapData.objects.filter(name=map_name).exists():
                map_data = MapData.objects.get(name=map_name)
            else:
                # 获取yaml配置
                yaml_path = local_map_dir / 'map.yaml'
                resolution = 0.05
                if yaml_path.exists():
                    try:
                        with open(yaml_path, 'r', encoding='utf-8') as f:
                            content = f.read()
                            for line in content.split('\n'):
                                if 'resolution:' in line:
                                    resolution = float(line.split(':')[1].strip())
                    except:
                        pass

                map_data = MapData.objects.create(
                    name=map_name,
                    resolution=resolution,
                    description=f'从机器狗 {ip} 下载的地图'
                )

            # 更新文件路径
            pgm_file = local_map_dir / 'map.pgm'
            yaml_file = local_map_dir / 'map.yaml'

            if pgm_file.exists():
                map_data.pgm_file.name = f'maps/{map_name}/map.pgm'
            if yaml_file.exists():
                map_data.yaml_file.name = f'maps/{map_name}/map.yaml'

            map_data.save()

            # 计算文件大小
            total_size = sum(
                f.stat().st_size
                for f in local_map_dir.iterdir()
                if f.is_file()
            )

            return Response({
                'success': True,
                'message': f'地图下载完成',
                'map': {
                    'id': map_data.id,
                    'name': map_data.name,
                    'downloaded_files': downloaded_files,
                    'skipped_files': skipped_files,
                    'total_size': f'{total_size / (1024 * 1024):.2f} MB'
                }
            })

        except paramiko.AuthenticationException:
            return Response(
                {"error": "认证失败"},
                status=status.HTTP_401_UNAUTHORIZED
            )
        except paramiko.SSHException as e:
            return Response(
                {"error": f"SSH连接失败: {str(e)}"},
                status=status.HTTP_500_INTERNAL_SERVER_ERROR
            )
        except Exception as e:
            return Response(
                {"error": f"下载失败: {str(e)}"},
                status=status.HTTP_500_INTERNAL_SERVER_ERROR
            )
