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
    camera_id = models.CharField(max_length=32, default="front")
    stream_id = models.CharField(max_length=96, blank=True)
    play_urls = models.JSONField(default=dict, blank=True)

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
        ("resolved", "已处理"),
    ]
    REVIEW_RESULT_CHOICES = [
        ("confirmed", "确认违规"),
        ("suspected", "怀疑"),
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
    review_result = models.CharField(max_length=16, choices=REVIEW_RESULT_CHOICES, default="confirmed")
    snapshot_url = models.URLField(blank=True)
    description = models.TextField(blank=True)
    handling_notes = models.TextField(blank=True)
    camera_id = models.CharField(max_length=32, blank=True)
    stream_id = models.CharField(max_length=96, blank=True)
    object_class = models.CharField(max_length=64, blank=True)
    track_id = models.CharField(max_length=64, blank=True)
    bbox_x = models.IntegerField(null=True, blank=True)
    bbox_y = models.IntegerField(null=True, blank=True)
    bbox_width = models.IntegerField(null=True, blank=True)
    bbox_height = models.IntegerField(null=True, blank=True)
    frame_width = models.PositiveIntegerField(null=True, blank=True)
    frame_height = models.PositiveIntegerField(null=True, blank=True)
    raw_detection = models.JSONField(default=dict, blank=True)

    class Meta:
        ordering = ["-detected_at"]

    def __str__(self) -> str:
        return self.title


class RobotTelemetry(BaseTimestampModel):
    robot = models.ForeignKey(Robot, related_name="telemetry_records", on_delete=models.CASCADE)
    sequence_id = models.CharField(max_length=64, unique=True)
    position_name = models.CharField(max_length=128)
    latitude = models.DecimalField(max_digits=10, decimal_places=6, null=True, blank=True)
    longitude = models.DecimalField(max_digits=10, decimal_places=6, null=True, blank=True)
    heading = models.DecimalField(max_digits=6, decimal_places=2, null=True, blank=True)
    speed = models.DecimalField(max_digits=6, decimal_places=2, null=True, blank=True)
    battery_level = models.PositiveSmallIntegerField(default=0)
    network_strength = models.PositiveSmallIntegerField(default=0)
    video = models.JSONField(default=dict, blank=True)
    raw_payload = models.JSONField(default=dict, blank=True)
    reported_at = models.DateTimeField(default=timezone.now)

    class Meta:
        ordering = ["-reported_at"]

    def __str__(self) -> str:
        return f"{self.robot.code}#{self.sequence_id}"


class MediaAsset(BaseTimestampModel):
    MEDIA_TYPE_CHOICES = [
        ("snapshot", "抓拍图"),
        ("clip", "视频片段"),
    ]

    robot = models.ForeignKey(Robot, related_name="media_assets", on_delete=models.CASCADE)
    media_type = models.CharField(max_length=16, choices=MEDIA_TYPE_CHOICES)
    camera_id = models.CharField(max_length=32, blank=True)
    sequence_id = models.CharField(max_length=64, blank=True)
    event_time = models.DateTimeField(null=True, blank=True)
    file = models.FileField(upload_to="device-media/%Y/%m/%d/")
    url = models.URLField(blank=True)
    sha256 = models.CharField(max_length=64, blank=True)
    file_size = models.PositiveIntegerField(default=0)

    class Meta:
        ordering = ["-created_at"]

    def __str__(self) -> str:
        return f"{self.robot.code} {self.media_type} {self.sequence_id}"


class RobotCommand(BaseTimestampModel):
    ACTION_CHOICES = [
        ("shake_hand", "握手"),
        ("stand_up", "站立"),
        ("lie_down", "趴下"),
        ("move_stop", "停止移动"),
        ("passive", "软急停"),
    ]
    STATUS_CHOICES = [
        ("queued", "待发送"),
        ("sent", "已发送"),
        ("failed", "发送失败"),
    ]

    robot = models.ForeignKey(Robot, related_name="commands", on_delete=models.CASCADE)
    action = models.CharField(max_length=32, choices=ACTION_CHOICES)
    payload = models.JSONField(default=dict, blank=True)
    status = models.CharField(max_length=16, choices=STATUS_CHOICES, default="queued")
    response_payload = models.JSONField(default=dict, blank=True)
    error_message = models.TextField(blank=True)
    sent_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        ordering = ["-created_at"]

    def __str__(self) -> str:
        return f"{self.robot.code} {self.action} {self.status}"

# Create your models here.
