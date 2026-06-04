import hashlib
import json
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from django.conf import settings
from django.contrib.auth import authenticate, get_user_model
from django.http import HttpResponseForbidden, StreamingHttpResponse
from django.db.models import Case, Count, IntegerField, Q, When
from django.utils.dateparse import parse_datetime
from django.utils import timezone
from rest_framework import permissions, status
from rest_framework.authtoken.models import Token
from rest_framework.decorators import api_view, permission_classes
from rest_framework.response import Response
from rest_framework.views import APIView

from .models import InspectionEvent, MediaAsset, PatrolTask, Robot, RobotCommand, RobotTelemetry
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
                "flv": "http://127.0.0.1:8080/live/dog_ZSL-1A-07_front.live.flv",
                "hls": "http://127.0.0.1:8080/live/dog_ZSL-1A-07_front/hls.m3u8",
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
        command = RobotCommand.objects.create(
            robot=robot,
            action=serializer.validated_data["action"],
            payload=serializer.validated_data.get("payload") or {},
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
        asset.url = request.build_absolute_uri(settings.MEDIA_URL + asset.file.name)
        asset.save(update_fields=["url", "updated_at"])
        return Response({"url": asset.url, "asset": MediaAssetSerializer(asset).data}, status=status.HTTP_201_CREATED)


@api_view(["GET"])
@permission_classes([permissions.AllowAny])
def health_check(_request):
    return Response({"status": "ok", "timestamp": timezone.now()})

# Create your views here.
