from django.contrib.auth import authenticate, get_user_model
from django.db.models import Count
from django.utils import timezone
from rest_framework import permissions, status
from rest_framework.authtoken.models import Token
from rest_framework.decorators import api_view, permission_classes
from rest_framework.response import Response
from rest_framework.views import APIView

from .models import InspectionEvent, PatrolTask, Robot, RobotTelemetry
from .serializers import (
    EventSerializer,
    PatrolTaskSerializer,
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
            "total": sum(item["value"] for item in points),
            "average": round(sum(item["value"] for item in points) / len(points), 1) if points else 0,
        },
    }


def build_analytics_payload():
    ensure_demo_seed()
    dates = build_period_labels(7)
    events = InspectionEvent.objects.all()
    tasks = PatrolTask.objects.all()
    robots = list(Robot.objects.all())
    telemetry = RobotTelemetry.objects.all()

    event_counts = {
        item["detected_at__date"]: item["total"]
        for item in events.values("detected_at__date").annotate(total=Count("id"))
    }
    risk_weight_map = {"high": 3, "medium": 2, "low": 1}
    risk_totals = {date: 0 for date in dates}
    for event in events.only("detected_at", "risk_level"):
        event_date = timezone.localtime(event.detected_at).date()
        if event_date in risk_totals:
            risk_totals[event_date] += risk_weight_map.get(event.risk_level, 1)

    task_completion = {
        item["scheduled_start__date"]: item["total"]
        for item in tasks.values("scheduled_start__date").annotate(total=Count("id"))
    }
    telemetry_counts = {
        item["reported_at__date"]: item["total"]
        for item in telemetry.values("reported_at__date").annotate(total=Count("id"))
    }

    robot_count = max(len(robots), 1)
    avg_patrol_duration = round(
        sum(robot.patrol_duration_minutes for robot in robots) / robot_count if robots else 0
    )
    avg_battery = round(sum(robot.battery_level for robot in robots) / robot_count if robots else 0)

    alert_series = [
        {"label": date.strftime("%m-%d"), "value": event_counts.get(date, 0)} for date in dates
    ]
    detection_series = [
        {
            "label": date.strftime("%m-%d"),
            "value": event_counts.get(date, 0) + telemetry_counts.get(date, 0),
        }
        for date in dates
    ]
    duration_series = [
        {
            "label": date.strftime("%m-%d"),
            "value": task_completion.get(date, 0) * 45 + (avg_patrol_duration if date == dates[-1] else 0),
        }
        for date in dates
    ]
    mileage_series = [
        {
            "label": date.strftime("%m-%d"),
            "value": round(task_completion.get(date, 0) * 1.6 + telemetry_counts.get(date, 0) * 0.8 + (avg_battery / 100), 1),
        }
        for date in dates
    ]

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
                "value": f"{max(2, 12 - InspectionEvent.objects.filter(status='processing').count() * 2)} 分钟",
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

    if Robot.objects.exists():
        return

    robot = Robot.objects.create(
        code="ZSL-1A-07",
        name="南入口巡检机器人",
        location="太阳宫公园南入口",
        area="主通道南入口",
        status="online",
        mode="auto",
        battery_level=78,
        network_strength=92,
        speaker_volume=84,
        patrol_duration_minutes=402,
        today_alerts=12,
        current_task_name="公园主通道例行巡检",
        firmware_version="v2.3.5",
    )
    PatrolTask.objects.create(
        name="公园主通道早间巡检",
        robot=robot,
        route_name="南门-主路-中心广场",
        scheduled_start=timezone.now() - timezone.timedelta(hours=2),
        scheduled_end=timezone.now() + timezone.timedelta(hours=1),
        status="running",
        completion_rate=68,
    )
    InspectionEvent.objects.bulk_create(
        [
            InspectionEvent(
                robot=robot,
                title="自行车违停识别",
                event_type="vehicle_illegal_parking",
                location="太阳宫公园南入口",
                confidence=92.5,
                risk_level="medium",
                status="pending",
                description="机器人在南入口主通道识别到自行车长时间停靠。",
            ),
            InspectionEvent(
                robot=robot,
                title="人员聚集提醒",
                event_type="crowd_gathering",
                location="中心广场北侧",
                confidence=88.2,
                risk_level="medium",
                status="processing",
                description="中心广场短时出现人群聚集，需要人工复核。",
            ),
            InspectionEvent(
                robot=robot,
                title="烟雾异常识别",
                event_type="smoke_alert",
                location="湖畔步道",
                confidence=95.1,
                risk_level="high",
                status="resolved",
                description="湖畔步道疑似烟雾，已通知现场保安处置。",
                handling_notes="现场确认是临时保洁设备尾气，无持续风险。",
            ),
        ]
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
                    "today_alert_count": today_events.count(),
                    "pending_event_count": events.filter(status="pending").count(),
                    "processing_event_count": events.filter(status="processing").count(),
                    "completed_task_count": PatrolTask.objects.filter(status="completed").count(),
                },
                "header": {
                    "device_code": latest_robot.code if latest_robot else "--",
                    "current_mode": latest_robot.get_mode_display() if latest_robot else "--",
                    "current_location": latest_robot.location if latest_robot else "--",
                    "today_alerts": latest_robot.today_alerts if latest_robot else 0,
                },
                "live_event": EventSerializer(latest_event).data if latest_event else None,
                "latest_robot": RobotDetailSerializer(latest_robot).data if latest_robot else None,
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
        return Response(RobotDetailSerializer(robot).data)


class EventListView(APIView):
    def get(self, request):
        ensure_demo_seed()
        queryset = InspectionEvent.objects.select_related("robot").all()
        status_value = request.query_params.get("status")
        if status_value:
            queryset = queryset.filter(status=status_value)
        return Response(EventSerializer(queryset, many=True).data)


class EventDetailView(APIView):
    def get(self, request, event_id):
        ensure_demo_seed()
        event = InspectionEvent.objects.select_related("robot").get(id=event_id)
        return Response(EventSerializer(event).data)


class EventHandleView(APIView):
    def post(self, request, event_id):
        ensure_demo_seed()
        event = InspectionEvent.objects.get(id=event_id)
        next_status = request.data.get("status", event.status)
        notes = request.data.get("handling_notes", "")
        event.status = next_status
        event.handling_notes = notes
        event.save(update_fields=["status", "handling_notes", "updated_at"])
        return Response(EventSerializer(event).data)


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
        robot, _ = Robot.objects.get_or_create(
            code=payload["robot_code"],
            defaults={
                "name": payload.get("robot_name") or payload["robot_code"],
                "location": payload["position"]["name"],
                "area": payload["position"]["name"],
            },
        )

        robot.location = payload["position"]["name"]
        robot.area = payload["position"]["name"]
        robot.battery_level = payload["power"]["battery_level"]
        robot.network_strength = payload["network"]["signal_strength"]
        robot.mode = payload["runtime"]["mode"]
        robot.status = payload["runtime"]["status"]
        robot.last_heartbeat_at = payload["reported_at"]
        robot.today_alerts += len(payload.get("detections", []))
        robot.save()

        RobotTelemetry.objects.create(
            robot=robot,
            sequence_id=payload["sequence_id"],
            position_name=payload["position"]["name"],
            latitude=payload["position"].get("latitude"),
            longitude=payload["position"].get("longitude"),
            heading=payload["motion"].get("heading"),
            speed=payload["motion"].get("speed"),
            battery_level=payload["power"]["battery_level"],
            network_strength=payload["network"]["signal_strength"],
            raw_payload=request.data,
            reported_at=payload["reported_at"],
        )

        for detection in payload.get("detections", []):
            InspectionEvent.objects.create(
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
            )

        return Response({"detail": "上报成功", "robot_id": robot.id}, status=status.HTTP_201_CREATED)


@api_view(["GET"])
@permission_classes([permissions.AllowAny])
def health_check(_request):
    return Response({"status": "ok", "timestamp": timezone.now()})

# Create your views here.
