from rest_framework import serializers

from .models import InspectionEvent, PatrolTask, Robot


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
        ]


class EventSerializer(serializers.ModelSerializer):
    robot_code = serializers.CharField(source="robot.code", read_only=True)
    robot_name = serializers.CharField(source="robot.name", read_only=True)
    risk_label = serializers.CharField(source="get_risk_level_display", read_only=True)
    status_label = serializers.CharField(source="get_status_display", read_only=True)

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
            "description",
            "handling_notes",
            "robot_code",
            "robot_name",
        ]


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
    detections = serializers.ListField(child=serializers.DictField(), required=False, default=list)
