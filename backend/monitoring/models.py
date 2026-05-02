from django.db import models
from django.utils import timezone


class BaseTimestampModel(models.Model):
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        abstract = True


class Robot(BaseTimestampModel):
    STATUS_CHOICES = [
        ("online", "在线"),
        ("offline", "离线"),
        ("warning", "告警"),
        ("charging", "充电中"),
    ]
    MODE_CHOICES = [
        ("auto", "AI自主巡检"),
        ("manual", "人工接管"),
        ("standby", "待命"),
        ("returning", "返航"),
    ]

    code = models.CharField(max_length=32, unique=True)
    name = models.CharField(max_length=64)
    location = models.CharField(max_length=128)
    area = models.CharField(max_length=128)
    status = models.CharField(max_length=16, choices=STATUS_CHOICES, default="online")
    mode = models.CharField(max_length=16, choices=MODE_CHOICES, default="auto")
    battery_level = models.PositiveSmallIntegerField(default=100)
    network_strength = models.PositiveSmallIntegerField(default=100)
    speaker_volume = models.PositiveSmallIntegerField(default=84)
    patrol_duration_minutes = models.PositiveIntegerField(default=0)
    today_alerts = models.PositiveIntegerField(default=0)
    current_task_name = models.CharField(max_length=128, blank=True)
    firmware_version = models.CharField(max_length=32, default="v1.0.0")
    last_heartbeat_at = models.DateTimeField(default=timezone.now)

    class Meta:
        ordering = ["code"]

    def __str__(self) -> str:
        return f"{self.code} - {self.name}"


class PatrolTask(BaseTimestampModel):
    STATUS_CHOICES = [
        ("pending", "待执行"),
        ("running", "执行中"),
        ("completed", "已完成"),
        ("paused", "已暂停"),
    ]

    name = models.CharField(max_length=128)
    robot = models.ForeignKey(Robot, related_name="tasks", on_delete=models.CASCADE)
    route_name = models.CharField(max_length=128)
    scheduled_start = models.DateTimeField()
    scheduled_end = models.DateTimeField()
    status = models.CharField(max_length=16, choices=STATUS_CHOICES, default="pending")
    completion_rate = models.PositiveSmallIntegerField(default=0)

    class Meta:
        ordering = ["-scheduled_start"]

    def __str__(self) -> str:
        return self.name


class InspectionEvent(BaseTimestampModel):
    STATUS_CHOICES = [
        ("pending", "待处理"),
        ("processing", "处理中"),
        ("resolved", "已完成"),
        ("false_alarm", "误报"),
    ]
    RISK_CHOICES = [
        ("high", "高"),
        ("medium", "中"),
        ("low", "低"),
    ]

    robot = models.ForeignKey(Robot, related_name="events", on_delete=models.CASCADE)
    title = models.CharField(max_length=128)
    event_type = models.CharField(max_length=64)
    location = models.CharField(max_length=128)
    detected_at = models.DateTimeField(default=timezone.now)
    confidence = models.DecimalField(max_digits=5, decimal_places=2, default=0)
    risk_level = models.CharField(max_length=16, choices=RISK_CHOICES, default="medium")
    status = models.CharField(max_length=16, choices=STATUS_CHOICES, default="pending")
    snapshot_url = models.URLField(blank=True)
    description = models.TextField(blank=True)
    handling_notes = models.TextField(blank=True)

    class Meta:
        ordering = ["-detected_at"]

    def __str__(self) -> str:
        return self.title


class RobotTelemetry(BaseTimestampModel):
    robot = models.ForeignKey(Robot, related_name="telemetry_records", on_delete=models.CASCADE)
    sequence_id = models.CharField(max_length=64)
    position_name = models.CharField(max_length=128)
    latitude = models.DecimalField(max_digits=10, decimal_places=6, null=True, blank=True)
    longitude = models.DecimalField(max_digits=10, decimal_places=6, null=True, blank=True)
    heading = models.DecimalField(max_digits=6, decimal_places=2, null=True, blank=True)
    speed = models.DecimalField(max_digits=6, decimal_places=2, null=True, blank=True)
    battery_level = models.PositiveSmallIntegerField(default=0)
    network_strength = models.PositiveSmallIntegerField(default=0)
    raw_payload = models.JSONField(default=dict, blank=True)
    reported_at = models.DateTimeField(default=timezone.now)

    class Meta:
        ordering = ["-reported_at"]

    def __str__(self) -> str:
        return f"{self.robot.code}#{self.sequence_id}"

# Create your models here.
