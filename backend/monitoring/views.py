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
