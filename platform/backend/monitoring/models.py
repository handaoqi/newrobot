import uuid

from django.conf import settings
from django.db import models
from django.db.models import Q
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
    connection_status = models.CharField(
        max_length=16,
        choices=[("unknown", "未知"), ("online", "在线"), ("offline", "离线")],
        default="unknown",
    )
    agent_version = models.CharField(max_length=64, blank=True)
    capabilities = models.JSONField(default=list, blank=True)
    localization_status = models.CharField(
        max_length=24,
        choices=[
            ("unknown", "未知"),
            ("initializing", "初始化"),
            ("relocalizing", "重定位"),
            ("relocalized", "重定位成功"),
            ("normal", "正常"),
            ("lost", "丢失"),
        ],
        default="unknown",
    )
    ros_ready = models.BooleanField(default=False)
    nav_ready = models.BooleanField(default=False)
    control_mode = models.CharField(
        max_length=24,
        choices=[
            ("unknown", "未知"),
            ("autonomous", "自主"),
            ("manual_assist", "人工辅助"),
            ("manual_takeover", "人工接管"),
            ("emergency_stop", "急停"),
        ],
        default="unknown",
    )
    current_map_id = models.CharField(max_length=128, blank=True)
    current_map_version = models.CharField(max_length=64, blank=True)
    charging_map = models.ForeignKey(
        "MapData", related_name="charging_robots", on_delete=models.SET_NULL,
        null=True, blank=True,
    )
    charging_route = models.ForeignKey(
        "PatrolRoute", related_name="charging_robots", on_delete=models.SET_NULL,
        null=True, blank=True,
    )
    last_state_version = models.BigIntegerField(default=0)
    last_seen_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        ordering = ["code"]

    def __str__(self) -> str:
        return f"{self.code} - {self.name}"

    def effective_connection_status(self, offline_after_seconds: int = 180) -> str:
        if self.connection_status != "online":
            return self.connection_status
        if not self.last_seen_at:
            return "offline"
        if timezone.now() - self.last_seen_at > timezone.timedelta(seconds=offline_after_seconds):
            return "offline"
        return "online"


class RobotPersonDetectionState(BaseTimestampModel):
    """Latest live detection boxes; high-rate frames must not become alert records."""

    robot = models.OneToOneField(
        Robot,
        related_name="person_detection_state",
        on_delete=models.CASCADE,
    )
    camera_id = models.CharField(max_length=32, default="front")
    frame_width = models.PositiveIntegerField(default=0)
    frame_height = models.PositiveIntegerField(default=0)
    captured_at = models.DateTimeField(default=timezone.now)
    detections = models.JSONField(default=list, blank=True)
    enabled = models.BooleanField(default=False)

    class Meta:
        ordering = ["robot__code"]


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
    route = models.ForeignKey(
        "PatrolRoute",
        related_name="task_templates",
        on_delete=models.PROTECT,
        null=True,
        blank=True,
    )
    enabled = models.BooleanField(default=True)
    record_rosbag = models.BooleanField(default=False)
    description = models.TextField(blank=True)
    created_by = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        related_name="created_patrol_tasks",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )

    class Meta:
        ordering = ["-scheduled_start"]

    def __str__(self) -> str:
        return self.name


class CalendarDay(BaseTimestampModel):
    DAY_TYPE_CHOICES = [
        ("holiday", "节假日"),
        ("workday", "调休日"),
        ("event_day", "特殊活动日"),
    ]

    date = models.DateField(unique=True)
    name = models.CharField(max_length=128)
    day_type = models.CharField(max_length=16, choices=DAY_TYPE_CHOICES, default="holiday")
    enabled = models.BooleanField(default=True)
    note = models.TextField(blank=True)

    class Meta:
        ordering = ["date"]

    def __str__(self) -> str:
        return f"{self.date} {self.name}"


class PatrolSchedule(BaseTimestampModel):
    SCHEDULE_TYPE_CHOICES = [
        ("daily", "日常计划"),
        ("holiday", "节假日计划"),
        ("once", "一次性计划"),
    ]

    name = models.CharField(max_length=128)
    robot = models.ForeignKey(Robot, related_name="patrol_schedules", on_delete=models.CASCADE)
    task_template = models.ForeignKey(PatrolTask, related_name="calendar_schedules", on_delete=models.PROTECT)
    route = models.ForeignKey("PatrolRoute", related_name="calendar_schedules", on_delete=models.PROTECT)
    map_data = models.ForeignKey("MapData", related_name="calendar_schedules", on_delete=models.PROTECT)
    schedule_type = models.CharField(max_length=16, choices=SCHEDULE_TYPE_CHOICES, default="daily")
    time_of_day = models.TimeField()
    weekdays = models.JSONField(default=list, blank=True)
    run_date = models.DateField(null=True, blank=True)
    priority = models.IntegerField(default=10)
    enabled = models.BooleanField(default=True)
    last_triggered_at = models.DateTimeField(null=True, blank=True)
    created_by = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        related_name="created_patrol_schedules",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    note = models.TextField(blank=True)

    class Meta:
        ordering = ["-enabled", "time_of_day", "-priority", "name"]
        indexes = [
            models.Index(fields=["enabled", "schedule_type", "time_of_day"], name="patrol_sched_due_idx"),
            models.Index(fields=["robot", "enabled"], name="patrol_sched_robot_idx"),
        ]

    def __str__(self) -> str:
        return self.name


class ScheduleRun(BaseTimestampModel):
    STATUS_CHOICES = [
        ("created", "已创建"),
        ("dispatched", "已下发"),
        ("skipped", "已跳过"),
        ("failed", "失败"),
    ]

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    schedule = models.ForeignKey(PatrolSchedule, related_name="runs", on_delete=models.CASCADE)
    robot = models.ForeignKey(Robot, related_name="schedule_runs", on_delete=models.CASCADE)
    planned_start_at = models.DateTimeField()
    triggered_at = models.DateTimeField(null=True, blank=True)
    status = models.CharField(max_length=16, choices=STATUS_CHOICES, default="created")
    task_execution = models.ForeignKey(
        "TaskExecution",
        related_name="schedule_runs",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    remote_command = models.ForeignKey(
        "RemoteCommand",
        related_name="schedule_runs",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    skip_reason = models.CharField(max_length=64, blank=True)
    error_message = models.TextField(blank=True)

    class Meta:
        ordering = ["-planned_start_at"]
        indexes = [
            models.Index(fields=["robot", "-planned_start_at"], name="sched_run_robot_time_idx"),
            models.Index(fields=["status", "-planned_start_at"], name="sched_run_status_time_idx"),
        ]
        constraints = [
            models.UniqueConstraint(
                fields=["schedule", "planned_start_at"],
                name="uniq_schedule_planned_start",
            )
        ]

    def __str__(self) -> str:
        return f"{self.schedule.name} @ {self.planned_start_at}"


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
    event_id = models.UUIDField(default=uuid.uuid4, unique=True, db_index=True)
    task_execution = models.ForeignKey(
        "TaskExecution",
        related_name="inspection_events",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    map_data = models.ForeignKey(
        "MapData",
        related_name="inspection_events",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    map_id = models.CharField(max_length=128, blank=True)
    map_version = models.CharField(max_length=64, blank=True)
    frame_id = models.CharField(max_length=32, default="map")
    position_x = models.DecimalField(max_digits=12, decimal_places=4, null=True, blank=True)
    position_y = models.DecimalField(max_digits=12, decimal_places=4, null=True, blank=True)
    position_yaw = models.DecimalField(max_digits=10, decimal_places=5, null=True, blank=True)
    source_component = models.CharField(max_length=64, blank=True)
    source_code = models.CharField(max_length=64, blank=True)
    model_version = models.CharField(max_length=64, blank=True)
    snapshot_asset = models.ForeignKey(
        "MediaAsset",
        related_name="snapshot_events",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    clip_asset = models.ForeignKey(
        "MediaAsset",
        related_name="clip_events",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    handled_by = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        related_name="handled_inspection_events",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    handled_at = models.DateTimeField(null=True, blank=True)

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
    video = models.JSONField(default=dict, blank=True)
    raw_payload = models.JSONField(default=dict, blank=True)
    reported_at = models.DateTimeField(default=timezone.now)
    session_id = models.CharField(max_length=64, default="legacy")
    message_id = models.UUIDField(null=True, blank=True, db_index=True)
    task_execution = models.ForeignKey(
        "TaskExecution",
        related_name="telemetry_records",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    frame_id = models.CharField(max_length=32, blank=True)
    map_id = models.CharField(max_length=128, blank=True)
    map_version = models.CharField(max_length=64, blank=True)
    x = models.DecimalField(max_digits=12, decimal_places=4, null=True, blank=True)
    y = models.DecimalField(max_digits=12, decimal_places=4, null=True, blank=True)
    z = models.DecimalField(max_digits=12, decimal_places=4, null=True, blank=True)
    yaw = models.DecimalField(max_digits=10, decimal_places=5, null=True, blank=True)
    localization_status = models.CharField(max_length=24, blank=True)

    class Meta:
        ordering = ["-reported_at"]
        indexes = [
            models.Index(fields=["reported_at"], name="robottelemetry_reported_idx"),
        ]
        constraints = [
            models.UniqueConstraint(
                fields=["robot", "session_id", "sequence_id"],
                name="uniq_robot_session_telemetry_sequence",
            )
        ]

    def __str__(self) -> str:
        return f"{self.robot.code}#{self.sequence_id}"


class RobotTelemetryDailySummary(BaseTimestampModel):
    """Daily business metrics retained after detailed telemetry expires."""

    robot = models.ForeignKey(Robot, related_name="telemetry_daily_summaries", on_delete=models.CASCADE)
    day = models.DateField()
    sample_count = models.PositiveIntegerField(default=0)
    first_reported_at = models.DateTimeField()
    last_reported_at = models.DateTimeField()
    active_seconds = models.PositiveBigIntegerField(default=0)
    distance_km = models.DecimalField(max_digits=14, decimal_places=6, default=0)
    first_latitude = models.DecimalField(max_digits=10, decimal_places=6, null=True, blank=True)
    first_longitude = models.DecimalField(max_digits=10, decimal_places=6, null=True, blank=True)
    last_latitude = models.DecimalField(max_digits=10, decimal_places=6, null=True, blank=True)
    last_longitude = models.DecimalField(max_digits=10, decimal_places=6, null=True, blank=True)

    class Meta:
        ordering = ["-day", "robot_id"]
        indexes = [
            models.Index(fields=["day"], name="telemetry_summary_day_idx"),
        ]
        constraints = [
            models.UniqueConstraint(fields=["robot", "day"], name="uniq_robot_daily_telemetry"),
        ]

    def __str__(self) -> str:
        return f"{self.robot.code}@{self.day}"


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
    media_id = models.UUIDField(default=uuid.uuid4, unique=True, db_index=True)
    event_id = models.UUIDField(null=True, blank=True, db_index=True)
    task_execution = models.ForeignKey(
        "TaskExecution",
        related_name="media_assets",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    content_type = models.CharField(max_length=128, blank=True)
    uploaded_at = models.DateTimeField(default=timezone.now)

    class Meta:
        ordering = ["-created_at"]

    def __str__(self) -> str:
        return f"{self.robot.code} {self.media_type} {self.sequence_id}"


class RobotCommand(BaseTimestampModel):
    ACTION_CHOICES = [
        ("shake_hand", "握手"),
        ("stand_up", "站立"),
        ("lie_down", "趴下"),
        ("crawl_forward", "匍匐前进"),
        ("speed_micro", "微速档"),
        ("speed_slow", "低速档"),
        ("speed_normal", "中速档"),
        ("speed_fast", "高速档"),
        ("move_forward", "前进"),
        ("move_backward", "后退"),
        ("move_left", "左移"),
        ("move_right", "右移"),
        ("turn_left", "左转"),
        ("turn_right", "右转"),
        ("move_velocity", "跟随速度"),
        ("takeover_enter", "进入远程接管"),
        ("takeover_exit", "退出远程接管"),
        ("move_stop", "停止移动"),
        ("passive", "软急停"),
        ("jump", "原地跳"),
        ("front_jump", "向前跳"),
        ("backflip", "后空翻"),
        ("two_leg_stand", "双腿站立"),
        ("cancel_two_leg_stand", "取消双腿站立"),
        ("attitude_control", "姿态控制"),
        ("play_audio", "播放音频"),
        ("charge_start", "开始充电"),
        ("charge_stop", "断开充电"),
        ("motion_start", "启动运控"),
        ("motion_stop", "停止运控"),
        ("audio_volume", "调节音量"),
        ("skill", "执行遥控技能"),
        ("skill_status", "查询遥控技能"),
        ("skill_cancel", "取消遥控技能"),
    ]
    STATUS_CHOICES = [
        ("queued", "待发送"),
        ("sent", "已发送"),
        ("running", "执行中"),
        ("finished", "已完成"),
        ("failed", "发送失败"),
        ("expired", "已过期"),
        ("superseded", "已被新命令替换"),
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


class SpeechCategory(BaseTimestampModel):
    name = models.CharField(max_length=64, unique=True)

    class Meta:
        ordering = ["id"]

    def __str__(self) -> str:
        return self.name


class SpeechTemplate(BaseTimestampModel):
    name = models.CharField(max_length=64, unique=True)
    text = models.CharField(max_length=500)
    category = models.ForeignKey(
        SpeechCategory,
        related_name="templates",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    created_by = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        related_name="speech_templates",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )

    class Meta:
        ordering = ["id"]

    def __str__(self) -> str:
        return self.name


class AlertSkillBinding(BaseTimestampModel):
    SKILL_CHOICES = [
        ("bicycle_alert", "自行车告警"),
        ("obstacle_detected", "发现障碍"),
        ("avoidance", "避障"),
        ("dissuasion", "劝阻"),
        ("low_battery_return_charge", "低电量停车告警"),
    ]

    skill_key = models.CharField(max_length=32, choices=SKILL_CHOICES, unique=True)
    template = models.ForeignKey(
        SpeechTemplate,
        related_name="alert_skill_bindings",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    enabled = models.BooleanField(default=True)

    class Meta:
        ordering = ["id"]

    def __str__(self) -> str:
        return self.get_skill_key_display()


class RecordedAudio(BaseTimestampModel):
    ASR_STATUS_CHOICES = [
        ("pending", "识别中"),
        ("completed", "识别完成"),
        ("failed", "识别失败"),
    ]

    title = models.CharField(max_length=128)
    category = models.ForeignKey(
        SpeechCategory,
        related_name="recordings",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    file = models.FileField(upload_to="recorded-audio/%Y/%m/%d/")
    content_type = models.CharField(max_length=128, blank=True)
    file_size = models.PositiveIntegerField(default=0)
    transcript = models.TextField(blank=True)
    asr_status = models.CharField(max_length=16, choices=ASR_STATUS_CHOICES, default="pending")
    asr_error = models.CharField(max_length=255, blank=True)
    created_by = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        related_name="recorded_audios",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )

    class Meta:
        ordering = ["-created_at"]

    def __str__(self) -> str:
        return self.title


class MapData(BaseTimestampModel):
    """地图数据模型"""
    name = models.CharField(max_length=128, verbose_name="地图名称")
    robot = models.ForeignKey(Robot, on_delete=models.SET_NULL, null=True, blank=True, verbose_name="关联机器人", related_name="maps")
    pgm_file = models.FileField(upload_to='maps/', verbose_name="PGM地图文件", null=True, blank=True)
    yaml_file = models.FileField(upload_to='maps/', verbose_name="YAML配置文件", null=True, blank=True)
    thumbnail = models.ImageField(upload_to='maps/thumbnails/', null=True, blank=True, verbose_name="缩略图")
    trajectory_file = models.FileField(upload_to='maps/traces/', null=True, blank=True, verbose_name="建图轨迹")
    mapping_trace = models.FileField(upload_to='maps/traces/', null=True, blank=True, verbose_name="建图定位轨迹")
    package_file = models.FileField(upload_to='maps/packages/', null=True, blank=True, verbose_name="完整地图包")
    resolution = models.FloatField(default=0.05, verbose_name="分辨率(m/像素)")
    width = models.IntegerField(default=0, verbose_name="宽度(像素)")
    height = models.IntegerField(default=0, verbose_name="高度(像素)")
    origin = models.JSONField(default=list, verbose_name="原点坐标 [x, y, theta]")
    active = models.BooleanField(default=False, verbose_name="是否为活动地图")
    description = models.TextField(blank=True, verbose_name="地图描述")
    parent_map = models.ForeignKey(
        "self",
        related_name="derived_maps",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
        verbose_name="来源地图",
    )
    edit_metadata = models.JSONField(default=dict, blank=True, verbose_name="地图编辑记录")
    coordinate_mode = models.CharField(max_length=32, blank=True, verbose_name="坐标模式")
    scene_scope = models.CharField(max_length=32, blank=True, verbose_name="场景范围")
    localization_mode = models.CharField(max_length=32, blank=True, verbose_name="定位模式")
    origin_status = models.CharField(max_length=32, blank=True, verbose_name="原点状态")
    map_completeness = models.CharField(max_length=32, blank=True, verbose_name="地图完整度")
    mapping_metrics = models.JSONField(default=dict, blank=True, verbose_name="建图指标")

    class Meta:
        verbose_name = "地图数据"
        verbose_name_plural = "地图数据"
        ordering = ["-created_at"]

    def __str__(self) -> str:
        return self.name


class MapSet(BaseTimestampModel):
    """A globally ordered set of overlapping local navigation maps."""

    name = models.CharField(max_length=128)
    robot = models.ForeignKey(Robot, on_delete=models.SET_NULL, null=True, blank=True, related_name="map_sets")
    version = models.CharField(max_length=64, blank=True)
    manifest = models.JSONField(default=dict, blank=True)
    active = models.BooleanField(default=False)

    class Meta:
        ordering = ["-created_at"]

    def __str__(self) -> str:
        return self.name


class MapSetMember(BaseTimestampModel):
    map_set = models.ForeignKey(MapSet, on_delete=models.CASCADE, related_name="members")
    map_data = models.OneToOneField(MapData, on_delete=models.CASCADE, related_name="map_set_member")
    sequence = models.PositiveIntegerField()
    submap_id = models.CharField(max_length=64)
    metadata = models.JSONField(default=dict, blank=True)

    class Meta:
        ordering = ["sequence"]
        constraints = [models.UniqueConstraint(fields=["map_set", "sequence"], name="unique_map_set_sequence")]


class PatrolRoute(BaseTimestampModel):
    """巡逻路线模型"""
    name = models.CharField(max_length=128, unique=True, verbose_name="路线名称")
    map_data = models.ForeignKey(MapData, on_delete=models.CASCADE, verbose_name="关联地图", related_name="routes")
    map_set = models.ForeignKey(MapSet, on_delete=models.SET_NULL, null=True, blank=True, related_name="routes")
    robot = models.ForeignKey(Robot, on_delete=models.CASCADE, verbose_name="关联机器人", related_name="routes")
    waypoints = models.JSONField(default=list, verbose_name="途经点坐标数组")  # [[x1,y1],[x2,y2],...]
    waypoint_names = models.JSONField(default=list, verbose_name="途经点名称数组")
    description = models.TextField(blank=True, verbose_name="路线描述")
    scene_scope = models.CharField(max_length=32, blank=True, default="indoor", verbose_name="场景范围")
    global_controller = models.CharField(
        max_length=32,
        blank=True,
        default="theta_star",
        verbose_name="全局控制器",
    )
    record_rosbag = models.BooleanField(default=False, verbose_name="录制导航调试包")

    class Meta:
        verbose_name = "巡逻路线"
        verbose_name_plural = "巡逻路线"
        ordering = ["-created_at"]

    def __str__(self) -> str:
        return self.name


class Zone(BaseTimestampModel):
    """禁区模型"""
    ZONE_TYPE_CHOICES = [
        ("forbidden", "禁入区"),
        ("warning", "警告区"),
        ("restricted", "限行区"),
    ]

    name = models.CharField(max_length=128, verbose_name="禁区名称")
    map_data = models.ForeignKey(MapData, on_delete=models.CASCADE, verbose_name="关联地图", related_name="zones")
    zone_type = models.CharField(max_length=16, choices=ZONE_TYPE_CHOICES, default="forbidden", verbose_name="禁区类型")
    polygon = models.JSONField(default=list, verbose_name="多边形坐标点 [[x1,y1],[x2,y2],...]")
    description = models.TextField(blank=True, verbose_name="禁区描述")
    active = models.BooleanField(default=True, verbose_name="是否启用")
    speed_limit_mps = models.FloatField(null=True, blank=True, verbose_name="限速值(m/s)")
    warning_distance_m = models.FloatField(default=0.5, verbose_name="提前警告距离(m)")

    class Meta:
        verbose_name = "禁区"
        verbose_name_plural = "禁区"
        ordering = ["-created_at"]

    def __str__(self) -> str:
        return self.name


class MapNavigationBoundary(BaseTimestampModel):
    """Versioned navigation geofence for one map.

    ``revision`` is the latest saved draft while ``active_revision`` is the
    last revision acknowledged by the Edge Agent. Keeping the two separate
    prevents an unfinished edit from silently changing a running robot.
    """

    APPLY_STATUS_CHOICES = [
        ("unconfigured", "未配置"),
        ("draft", "草稿"),
        ("applying", "应用中"),
        ("active", "已生效"),
        ("failed", "应用失败"),
    ]

    map_data = models.OneToOneField(
        MapData,
        related_name="navigation_boundary",
        on_delete=models.CASCADE,
    )
    outer_polygon = models.JSONField(default=list, blank=True)
    safety_margin_m = models.FloatField(default=0.2)
    revision = models.PositiveIntegerField(default=0)
    active_revision = models.PositiveIntegerField(default=0)
    active_payload = models.JSONField(default=dict, blank=True)
    apply_status = models.CharField(
        max_length=16,
        choices=APPLY_STATUS_CHOICES,
        default="unconfigured",
    )
    apply_error = models.TextField(blank=True)
    apply_command = models.ForeignKey(
        "RemoteCommand",
        related_name="boundary_applications",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )

    class Meta:
        ordering = ["map_data_id"]


class Track(BaseTimestampModel):
    """轨迹记录模型"""
    robot = models.ForeignKey(Robot, on_delete=models.CASCADE, verbose_name="关联机器人", related_name="tracks")
    map_data = models.ForeignKey(MapData, on_delete=models.SET_NULL, null=True, blank=True, verbose_name="关联地图", related_name="tracks")
    route = models.ForeignKey(PatrolRoute, on_delete=models.SET_NULL, null=True, blank=True, verbose_name="关联路线", related_name="tracks")
    task = models.ForeignKey(PatrolTask, on_delete=models.SET_NULL, null=True, blank=True, verbose_name="关联任务", related_name="tracks")
    path = models.JSONField(default=list, verbose_name="轨迹路径 [[x1,y1,timestamp1],[x2,y2,timestamp2],...]")
    start_time = models.DateTimeField(verbose_name="开始时间")
    end_time = models.DateTimeField(null=True, blank=True, verbose_name="结束时间")
    distance = models.FloatField(default=0, verbose_name="行驶距离(米)")
    duration = models.IntegerField(default=0, verbose_name="持续时间(秒)")
    description = models.TextField(blank=True, verbose_name="轨迹描述")

    class Meta:
        verbose_name = "轨迹记录"
        verbose_name_plural = "轨迹记录"
        ordering = ["-start_time"]

    def __str__(self) -> str:
        return f"{self.robot.code} - {self.start_time.strftime('%Y-%m-%d %H:%M:%S')}"


class RobotCredential(BaseTimestampModel):
    STATUS_CHOICES = [
        ("active", "有效"),
        ("revoked", "已吊销"),
        ("expired", "已过期"),
    ]

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    robot = models.ForeignKey(Robot, related_name="credentials", on_delete=models.CASCADE)
    credential_id = models.CharField(max_length=128, unique=True)
    secret_hash = models.CharField(max_length=256, blank=True)
    certificate_fingerprint = models.CharField(max_length=128, blank=True)
    status = models.CharField(max_length=16, choices=STATUS_CHOICES, default="active")
    issued_at = models.DateTimeField(default=timezone.now)
    expires_at = models.DateTimeField(null=True, blank=True)
    revoked_at = models.DateTimeField(null=True, blank=True)


class RobotSession(BaseTimestampModel):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    robot = models.ForeignKey(Robot, related_name="sessions", on_delete=models.CASCADE)
    session_id = models.UUIDField(unique=True)
    transport = models.CharField(
        max_length=8,
        choices=[("mqtt", "MQTT"), ("wss", "WSS")],
        default="mqtt",
    )
    connected_at = models.DateTimeField(default=timezone.now)
    last_heartbeat_at = models.DateTimeField(default=timezone.now)
    disconnected_at = models.DateTimeField(null=True, blank=True)
    disconnect_reason = models.CharField(max_length=128, blank=True)
    agent_version = models.CharField(max_length=64, blank=True)
    boot_id = models.CharField(max_length=128, blank=True)
    remote_ip = models.GenericIPAddressField(null=True, blank=True)
    capabilities = models.JSONField(default=list, blank=True)
    metadata = models.JSONField(default=dict, blank=True)

    class Meta:
        ordering = ["-connected_at"]
        indexes = [models.Index(fields=["robot", "-connected_at"], name="robot_session_recent_idx")]


class TaskExecution(BaseTimestampModel):
    ACTIVE_STATES = [
        "created",
        "dispatching",
        "accepted",
        "running",
        "pausing",
        "paused",
        "resuming",
        "cancelling",
        "interrupted",
    ]
    STATE_CHOICES = [
        ("created", "已创建"),
        ("dispatching", "下发中"),
        ("accepted", "已接受"),
        ("running", "执行中"),
        ("pausing", "暂停中"),
        ("paused", "已暂停"),
        ("resuming", "继续中"),
        ("cancelling", "终止中"),
        ("completed", "已完成"),
        ("failed", "失败"),
        ("cancelled", "已终止"),
        ("timed_out", "超时"),
        ("interrupted", "中断待对账"),
        ("rejected", "已拒绝"),
    ]

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    task = models.ForeignKey(PatrolTask, related_name="executions", on_delete=models.PROTECT)
    robot = models.ForeignKey(Robot, related_name="task_executions", on_delete=models.PROTECT)
    route = models.ForeignKey(PatrolRoute, related_name="executions", on_delete=models.SET_NULL, null=True, blank=True)
    map_data = models.ForeignKey(MapData, related_name="task_executions", on_delete=models.SET_NULL, null=True, blank=True)
    route_snapshot = models.JSONField(default=dict)
    loop_session_id = models.UUIDField(null=True, blank=True, db_index=True)
    round_number = models.PositiveIntegerField(default=1)
    # New loop executions receive a stable session/round key. Historical rows
    # stay NULL so an index can be added without rewriting duplicate history.
    loop_dispatch_key = models.CharField(max_length=80, null=True, blank=True, unique=True, editable=False)
    state = models.CharField(max_length=24, choices=STATE_CHOICES, default="created")
    state_version = models.BigIntegerField(default=0)
    current_waypoint_index = models.PositiveIntegerField(null=True, blank=True)
    current_waypoint_id = models.CharField(max_length=128, blank=True)
    completed_waypoints = models.PositiveIntegerField(default=0)
    total_waypoints = models.PositiveIntegerField(default=0)
    distance_remaining_m = models.DecimalField(max_digits=12, decimal_places=3, null=True, blank=True)
    estimated_time_remaining_s = models.PositiveIntegerField(null=True, blank=True)
    started_at = models.DateTimeField(null=True, blank=True)
    finished_at = models.DateTimeField(null=True, blank=True)
    accepted_at = models.DateTimeField(null=True, blank=True)
    paused_at = models.DateTimeField(null=True, blank=True)
    failure_code = models.CharField(max_length=64, blank=True)
    failure_message = models.TextField(blank=True)
    created_by = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        related_name="created_task_executions",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    last_edge_event_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        ordering = ["-created_at"]
        indexes = [
            models.Index(fields=["robot", "state"], name="task_exec_robot_state_idx"),
            models.Index(fields=["-created_at"], name="task_exec_recent_idx"),
        ]
        constraints = [
            models.CheckConstraint(condition=Q(state_version__gte=0), name="task_exec_state_version_gte_0"),
            models.UniqueConstraint(
                fields=["robot"],
                condition=Q(
                    state__in=[
                        "created",
                        "dispatching",
                        "accepted",
                        "running",
                        "pausing",
                        "paused",
                        "resuming",
                        "cancelling",
                        "interrupted",
                    ]
                ),
                name="uniq_active_task_execution_per_robot",
            ),
        ]


class TaskExecutionEvent(BaseTimestampModel):
    task_execution = models.ForeignKey(TaskExecution, related_name="events", on_delete=models.CASCADE)
    state = models.CharField(max_length=24, choices=TaskExecution.STATE_CHOICES)
    state_version = models.BigIntegerField()
    event_type = models.CharField(max_length=64)
    occurred_at = models.DateTimeField()
    received_at = models.DateTimeField(default=timezone.now)
    message_id = models.UUIDField(null=True, blank=True, unique=True)
    reason_code = models.CharField(max_length=64, blank=True)
    reason_message = models.TextField(blank=True)
    payload = models.JSONField(default=dict, blank=True)

    class Meta:
        ordering = ["state_version"]
        constraints = [
            models.UniqueConstraint(
                fields=["task_execution", "state_version"],
                name="uniq_task_execution_state_version",
            )
        ]


class RemoteCommand(BaseTimestampModel):
    TYPE_CHOICES = [
        ("task.start", "启动任务"),
        ("task.pause", "暂停任务"),
        ("task.resume", "继续任务"),
        ("task.resume_forward", "恢复前向"),
        ("task.cancel", "终止任务"),
        ("task.force_exit", "强制退出并清理任务"),
        ("task.recover.v1", "任务自愈"),
        ("mapping.start", "开始建图"),
        ("mapping.save", "停止并保存地图"),
        ("mapping.cancel", "取消建图"),
        ("mapping.status", "查询建图状态"),
        ("mapping.origin_start", "锁定 ENU 原点"),
        ("mapping.origin_cancel", "取消原点锁定"),
        ("mapping.origin_extract_global", "提取全局 ENU"),
        ("mapping.slam_start", "启动 SLAM 预热"),
        ("mapping.begin", "确认并开始正式建图"),
        ("nav.status", "查询导航状态"),
        ("nav.start", "启动导航栈"),
        ("nav.restart", "重启导航栈"),
        ("nav.recover", "恢复导航栈"),
        ("nav.stop", "停止导航栈"),
        ("nav.initial_pose", "设置初始定位"),
        ("nav.single_goal", "单点导航"),
        ("nav.relocalize", "主动重定位"),
        ("diagnostics.log_config", "配置诊断日志"),
        ("map.activate", "切换活动地图"),
        ("map.boundary_apply", "应用导航边界"),
        ("map.optimize", "离线回环优化"),
        ("sensor.restart", "重启传感器"),
        ("charge.start", "开始充电"),
        ("charge.stop", "断开充电"),
        ("motion.start", "启动运控"),
        ("motion.stop", "停止运控"),
        ("audio.volume", "调节扬声器音量"),
        ("teleop.takeover_enter", "进入远程接管"),
        ("teleop.takeover_exit", "退出远程接管"),
        ("teleop.stand_up", "站立"),
        ("teleop.lie_down", "趴下"),
        ("teleop.shake_hand", "打招呼"),
        ("teleop.two_leg_stand", "双腿站立"),
        ("teleop.crawl_forward", "匍匐前进"),
        ("teleop.speed_micro", "微速档"),
        ("teleop.speed_slow", "低速档"),
        ("teleop.speed_normal", "中速档"),
        ("teleop.speed_fast", "高速档"),
        ("teleop.move_forward", "前进"),
        ("teleop.move_backward", "后退"),
        ("teleop.move_left", "左移"),
        ("teleop.move_right", "右移"),
        ("teleop.turn_left", "左转"),
        ("teleop.turn_right", "右转"),
        ("teleop.move_velocity", "跟随速度"),
        ("teleop.move_stop", "停止移动"),
        ("teleop.passive", "软急停"),
        ("teleop.skill", "执行遥控技能"),
        ("teleop.skill_list", "列出遥控技能"),
        ("teleop.skill_status", "查询遥控技能"),
        ("teleop.skill_cancel", "取消遥控技能"),
        ("teleop.person_follow_start", "启动人员跟随"),
        ("teleop.person_follow_stop", "停止人员跟随"),
        ("teleop.person_follow_status", "查询人员跟随"),
    ]
    STATUS_CHOICES = [
        ("created", "已创建"),
        ("published", "已发布"),
        ("accepted", "已接受"),
        ("rejected", "已拒绝"),
        ("executing", "执行中"),
        ("succeeded", "成功"),
        ("failed", "失败"),
        ("cancelled", "已取消"),
        ("timed_out", "超时"),
        ("expired", "已过期"),
    ]

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    robot = models.ForeignKey(Robot, related_name="remote_commands", on_delete=models.PROTECT)
    task_execution = models.ForeignKey(
        TaskExecution,
        related_name="commands",
        on_delete=models.CASCADE,
        null=True,
        blank=True,
    )
    command_type = models.CharField(max_length=32, choices=TYPE_CHOICES)
    payload = models.JSONField(default=dict)
    status = models.CharField(max_length=16, choices=STATUS_CHOICES, default="created")
    operator = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        related_name="issued_remote_commands",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    trace_id = models.UUIDField(default=uuid.uuid4, db_index=True)
    issued_at = models.DateTimeField(default=timezone.now)
    expires_at = models.DateTimeField()
    published_at = models.DateTimeField(null=True, blank=True)
    acknowledged_at = models.DateTimeField(null=True, blank=True)
    started_at = models.DateTimeField(null=True, blank=True)
    finished_at = models.DateTimeField(null=True, blank=True)
    ack_reason_code = models.CharField(max_length=64, blank=True)
    ack_reason_message = models.TextField(blank=True)
    result_payload = models.JSONField(default=dict, blank=True)
    error_code = models.CharField(max_length=64, blank=True)
    error_message = models.TextField(blank=True)
    retry_count = models.PositiveIntegerField(default=0)

    class Meta:
        ordering = ["-issued_at"]
        indexes = [
            models.Index(fields=["robot", "-issued_at"], name="remote_cmd_robot_recent_idx"),
            models.Index(fields=["task_execution", "-issued_at"], name="remote_cmd_exec_recent_idx"),
        ]
        constraints = [
            models.CheckConstraint(condition=Q(expires_at__gt=models.F("issued_at")), name="remote_cmd_expiry_after_issue")
        ]


class CommandEvent(BaseTimestampModel):
    command = models.ForeignKey(RemoteCommand, related_name="events", on_delete=models.CASCADE)
    event_type = models.CharField(max_length=32)
    event_at = models.DateTimeField(default=timezone.now)
    source = models.CharField(max_length=16, choices=[("center", "中心"), ("edge", "边缘"), ("broker", "Broker")])
    message_id = models.UUIDField(null=True, blank=True, unique=True)
    payload = models.JSONField(default=dict, blank=True)

    class Meta:
        ordering = ["event_at"]
        indexes = [models.Index(fields=["command", "event_at"], name="command_event_time_idx")]


class PatrolLoopSession(BaseTimestampModel):
    ACTIVE_STATES = [
        "starting",
        "running",
        "resting",
        "observing",
        "recovering",
        "paused",
        "stopping",
    ]
    TERMINAL_STATES = ["completed", "failed", "cancelled", "low_battery_stopped"]
    STATE_CHOICES = [
        ("starting", "准备启动"),
        ("running", "循环执行中"),
        ("resting", "轮间休息"),
        ("observing", "异常观察中"),
        ("recovering", "自愈中"),
        ("paused", "人工暂停"),
        ("stopping", "停止中"),
        ("completed", "已完成"),
        ("failed", "失败"),
        ("cancelled", "已停止"),
        ("low_battery_stopped", "低电量终止"),
    ]

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    robot = models.ForeignKey(Robot, related_name="patrol_loop_sessions", on_delete=models.PROTECT)
    task = models.ForeignKey("PatrolTask", related_name="loop_sessions", on_delete=models.PROTECT)
    route_snapshot = models.JSONField(default=dict)
    state = models.CharField(max_length=32, choices=STATE_CHOICES, default="starting")
    state_version = models.BigIntegerField(default=0)
    duration_seconds = models.PositiveIntegerField()
    rest_seconds = models.PositiveIntegerField(default=0)
    started_at = models.DateTimeField(default=timezone.now)
    ends_at = models.DateTimeField()
    finished_at = models.DateTimeField(null=True, blank=True)
    current_round = models.PositiveIntegerField(default=0)
    current_execution = models.ForeignKey(
        TaskExecution,
        related_name="owning_loop_sessions",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    next_action_at = models.DateTimeField(default=timezone.now, db_index=True)
    observation_started_at = models.DateTimeField(null=True, blank=True)
    recovery_episode_id = models.UUIDField(null=True, blank=True)
    recovery_reason_code = models.CharField(max_length=64, blank=True)
    recovery_reason_message = models.TextField(blank=True)
    recovery_attempt = models.PositiveSmallIntegerField(default=0)
    recovery_max_attempts = models.PositiveSmallIntegerField(default=10)
    manual_paused = models.BooleanField(default=False)
    last_error = models.TextField(blank=True)
    metadata = models.JSONField(default=dict, blank=True)
    created_by = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        related_name="created_patrol_loop_sessions",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )

    class Meta:
        ordering = ["-created_at"]
        indexes = [
            models.Index(fields=["state", "next_action_at"], name="loop_state_due_idx"),
            models.Index(fields=["robot", "-created_at"], name="loop_robot_recent_idx"),
        ]
        constraints = [
            models.CheckConstraint(condition=Q(duration_seconds__gt=0), name="loop_duration_gt_0"),
            models.CheckConstraint(condition=Q(recovery_attempt__lte=10), name="loop_recovery_attempt_lte_10"),
            models.UniqueConstraint(
                fields=["robot"],
                condition=Q(
                    state__in=[
                        "starting",
                        "running",
                        "resting",
                        "observing",
                        "recovering",
                        "paused",
                        "stopping",
                    ]
                ),
                name="uniq_active_patrol_loop_per_robot",
            ),
        ]


class PatrolLoopEvent(BaseTimestampModel):
    loop_session = models.ForeignKey(
        PatrolLoopSession,
        related_name="events",
        on_delete=models.CASCADE,
    )
    task_execution = models.ForeignKey(
        TaskExecution,
        related_name="loop_events",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    event_type = models.CharField(max_length=64)
    state = models.CharField(max_length=32, choices=PatrolLoopSession.STATE_CHOICES)
    occurred_at = models.DateTimeField(default=timezone.now)
    reason_code = models.CharField(max_length=64, blank=True)
    reason_message = models.TextField(blank=True)
    recovery_attempt = models.PositiveSmallIntegerField(default=0)
    idempotency_key = models.CharField(max_length=160, unique=True)
    payload = models.JSONField(default=dict, blank=True)

    class Meta:
        ordering = ["occurred_at", "id"]
        indexes = [models.Index(fields=["loop_session", "-occurred_at"], name="loop_event_recent_idx")]


class RobotLowBatteryEpisode(BaseTimestampModel):
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    robot = models.ForeignKey(Robot, related_name="low_battery_episodes", on_delete=models.CASCADE)
    episode_key = models.CharField(max_length=128, unique=True)
    active = models.BooleanField(default=True)
    battery_percent = models.PositiveSmallIntegerField()
    threshold_percent = models.PositiveSmallIntegerField(default=20)
    rearm_percent = models.PositiveSmallIntegerField(default=25)
    source = models.CharField(max_length=32, default="edge_alert")
    triggered_at = models.DateTimeField(default=timezone.now)
    cleared_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        ordering = ["-triggered_at"]
        constraints = [
            models.UniqueConstraint(
                fields=["robot"],
                condition=Q(active=True),
                name="uniq_active_low_battery_episode",
            )
        ]


class DebugLogSession(BaseTimestampModel):
    STATUS_CHOICES = [
        ("starting", "启动中"),
        ("active", "已启用"),
        ("stopped", "已停止"),
        ("expired", "已到期"),
        ("failed", "失败"),
    ]

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    robot = models.ForeignKey(Robot, related_name="debug_log_sessions", on_delete=models.CASCADE)
    modules = models.JSONField(default=list)
    sample_hz = models.FloatField(default=1.0)
    status = models.CharField(max_length=16, choices=STATUS_CHOICES, default="starting")
    started_at = models.DateTimeField(default=timezone.now)
    expires_at = models.DateTimeField()
    stopped_at = models.DateTimeField(null=True, blank=True)
    created_by = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        related_name="debug_log_sessions",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )

    class Meta:
        ordering = ["-started_at"]
        indexes = [models.Index(fields=["robot", "-expires_at"], name="debug_log_robot_exp_idx")]


class SystemLog(BaseTimestampModel):
    LEVEL_CHOICES = [
        ("DEBUG", "DEBUG"),
        ("INFO", "INFO"),
        ("WARNING", "WARNING"),
        ("ERROR", "ERROR"),
    ]
    MODULE_CHOICES = [
        ("localization", "定位"),
        ("navigation", "导航"),
        ("avoidance", "避障"),
        ("relocalization", "主动重定位"),
        ("waypoint", "航点调整"),
        ("planner", "路径规划"),
        ("boundary", "导航边界"),
        ("system", "系统"),
    ]

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    robot = models.ForeignKey(Robot, related_name="system_logs", on_delete=models.CASCADE)
    occurred_at = models.DateTimeField(default=timezone.now)
    received_at = models.DateTimeField(default=timezone.now)
    level = models.CharField(max_length=8, choices=LEVEL_CHOICES)
    module = models.CharField(max_length=24, choices=MODULE_CHOICES)
    event_code = models.CharField(max_length=96)
    message = models.CharField(max_length=500)
    source = models.CharField(max_length=64, default="center")
    data = models.JSONField(default=dict, blank=True)
    trace_id = models.UUIDField(null=True, blank=True, db_index=True)
    route = models.ForeignKey(
        PatrolRoute,
        related_name="system_logs",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    waypoint_id = models.CharField(max_length=128, blank=True)
    round_number = models.PositiveIntegerField(null=True, blank=True)
    nav_goal_generation = models.PositiveIntegerField(null=True, blank=True)
    localization_generation = models.PositiveIntegerField(null=True, blank=True)
    dedupe_key = models.CharField(max_length=160, blank=True)
    task_execution = models.ForeignKey(
        TaskExecution,
        related_name="system_logs",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    command = models.ForeignKey(
        RemoteCommand,
        related_name="system_logs",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    map_data = models.ForeignKey(
        MapData,
        related_name="system_logs",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    waypoint_index = models.IntegerField(null=True, blank=True)
    x = models.FloatField(null=True, blank=True)
    y = models.FloatField(null=True, blank=True)
    yaw = models.FloatField(null=True, blank=True)
    repeat_count = models.PositiveIntegerField(default=1)

    class Meta:
        ordering = ["-occurred_at", "-id"]
        indexes = [
            models.Index(fields=["robot", "-occurred_at"], name="syslog_robot_time_idx"),
            models.Index(fields=["robot", "level", "-occurred_at"], name="syslog_level_time_idx"),
            models.Index(fields=["task_execution", "-occurred_at"], name="syslog_task_time_idx"),
            models.Index(fields=["robot", "trace_id", "-occurred_at"], name="syslog_trace_time_idx"),
            models.Index(fields=["robot", "module", "dedupe_key", "-occurred_at"], name="syslog_dedupe_idx"),
        ]


class InboundMessage(models.Model):
    message_id = models.UUIDField(primary_key=True)
    robot = models.ForeignKey(Robot, related_name="inbound_messages", on_delete=models.CASCADE)
    session_id = models.UUIDField()
    message_type = models.CharField(max_length=64)
    sequence = models.BigIntegerField(null=True, blank=True)
    topic = models.CharField(max_length=256)
    received_at = models.DateTimeField(default=timezone.now)
    processed_at = models.DateTimeField(null=True, blank=True)
    process_status = models.CharField(
        max_length=16,
        choices=[("pending", "待处理"), ("processed", "已处理"), ("failed", "失败"), ("ignored", "忽略")],
        default="pending",
    )
    error_message = models.TextField(blank=True)
    raw_payload = models.JSONField(default=dict)

    class Meta:
        indexes = [
            models.Index(fields=["process_status", "received_at"], name="inbound_status_time_idx"),
        ]


class RobotStatusLatest(models.Model):
    robot = models.OneToOneField(Robot, related_name="latest_status", primary_key=True, on_delete=models.CASCADE)
    state_version = models.BigIntegerField(default=0)
    sampled_at = models.DateTimeField()
    received_at = models.DateTimeField(default=timezone.now)
    frame_id = models.CharField(max_length=32, default="map")
    map_id = models.CharField(max_length=128, blank=True)
    map_version = models.CharField(max_length=64, blank=True)
    x = models.DecimalField(max_digits=12, decimal_places=4, null=True, blank=True)
    y = models.DecimalField(max_digits=12, decimal_places=4, null=True, blank=True)
    z = models.DecimalField(max_digits=12, decimal_places=4, null=True, blank=True)
    yaw = models.DecimalField(max_digits=10, decimal_places=5, null=True, blank=True)
    speed_mps = models.DecimalField(max_digits=10, decimal_places=5, null=True, blank=True)
    localization_status = models.CharField(max_length=24, default="unknown")
    localization_source_status = models.PositiveSmallIntegerField(null=True, blank=True)
    localization_quality = models.JSONField(default=dict, blank=True)
    power_available = models.BooleanField(default=False)
    battery_percent = models.PositiveSmallIntegerField(null=True, blank=True)
    charging = models.BooleanField(null=True, blank=True)
    network_type = models.CharField(max_length=32, blank=True)
    signal_percent = models.PositiveSmallIntegerField(null=True, blank=True)
    ros_ready = models.BooleanField(default=False)
    nav_ready = models.BooleanField(default=False)
    emergency_stop = models.BooleanField(default=False)
    control_mode = models.CharField(max_length=24, default="unknown")
    task_execution = models.ForeignKey(
        TaskExecution,
        related_name="latest_robot_statuses",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    raw_payload = models.JSONField(default=dict, blank=True)


class TrajectoryPoint(models.Model):
    robot = models.ForeignKey(Robot, related_name="trajectory_points", on_delete=models.CASCADE)
    task_execution = models.ForeignKey(TaskExecution, related_name="trajectory_points", on_delete=models.CASCADE)
    seq = models.BigIntegerField()
    sampled_at = models.DateTimeField()
    received_at = models.DateTimeField(default=timezone.now)
    frame_id = models.CharField(max_length=32, default="map")
    map_id = models.CharField(max_length=128, blank=True)
    map_version = models.CharField(max_length=64, blank=True)
    x = models.DecimalField(max_digits=12, decimal_places=4)
    y = models.DecimalField(max_digits=12, decimal_places=4)
    yaw = models.DecimalField(max_digits=10, decimal_places=5)
    speed_mps = models.DecimalField(max_digits=10, decimal_places=5, null=True, blank=True)
    localization_status = models.CharField(max_length=24)
    batch_id = models.UUIDField()

    class Meta:
        ordering = ["seq"]
        constraints = [
            models.UniqueConstraint(
                fields=["robot", "task_execution", "seq"],
                name="uniq_robot_execution_trajectory_seq",
            ),
            models.CheckConstraint(condition=Q(seq__gte=0), name="trajectory_seq_gte_0"),
        ]
        indexes = [
            models.Index(fields=["task_execution", "sampled_at"], name="trajectory_exec_time_idx"),
            models.Index(fields=["robot", "sampled_at"], name="trajectory_robot_time_idx"),
        ]


class TrajectoryBatchReceipt(models.Model):
    batch_id = models.UUIDField(primary_key=True)
    robot = models.ForeignKey(Robot, related_name="trajectory_receipts", on_delete=models.CASCADE)
    task_execution = models.ForeignKey(TaskExecution, related_name="trajectory_receipts", on_delete=models.CASCADE)
    first_seq = models.BigIntegerField()
    last_seq = models.BigIntegerField()
    point_count = models.PositiveIntegerField()
    received_at = models.DateTimeField(default=timezone.now)
    duplicate = models.BooleanField(default=False)

    class Meta:
        constraints = [
            models.CheckConstraint(condition=Q(last_seq__gte=models.F("first_seq")), name="trajectory_batch_valid_range")
        ]


class DevelopmentAgentState(BaseTimestampModel):
    robot = models.OneToOneField(
        Robot,
        related_name="development_agent_state",
        primary_key=True,
        on_delete=models.CASCADE,
    )
    status = models.CharField(max_length=16, default="offline")
    agent_version = models.CharField(max_length=64, blank=True)
    codex_binary = models.CharField(max_length=512, blank=True)
    workspaces = models.JSONField(default=list, blank=True)
    last_seen_at = models.DateTimeField(default=timezone.now)


class DevelopmentConversationState(BaseTimestampModel):
    MODE_CHOICES = [
        ("execute", "执行模式"),
        ("plan", "规划对话模式"),
    ]

    robot = models.OneToOneField(
        Robot,
        related_name="development_conversation_state",
        primary_key=True,
        on_delete=models.CASCADE,
    )
    mode = models.CharField(max_length=16, choices=MODE_CHOICES, default="execute")
    workspace = models.CharField(max_length=64, default="robot-main")
    model = models.CharField(max_length=64, default="gpt-5.6-terra")


class DevelopmentTask(BaseTimestampModel):
    STATUS_CHOICES = [
        ("created", "待下发"),
        ("published", "已下发"),
        ("running", "执行中"),
        ("cancelling", "取消中"),
        ("succeeded", "已完成"),
        ("failed", "失败"),
        ("cancelled", "已取消"),
        ("timed_out", "超时"),
        ("rejected", "已拒绝"),
    ]
    TERMINAL_STATES = {"succeeded", "failed", "cancelled", "timed_out", "rejected"}
    ACTIVE_STATES = {"created", "published", "running", "cancelling"}

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    robot = models.ForeignKey(Robot, related_name="development_tasks", on_delete=models.PROTECT)
    workspace = models.CharField(max_length=64)
    model = models.CharField(max_length=64, default="gpt-5.6-terra")
    execution_mode = models.CharField(
        max_length=16,
        choices=[("plan", "规划对话"), ("execute", "执行任务")],
        default="execute",
    )
    prompt = models.TextField()
    status = models.CharField(max_length=16, choices=STATUS_CHOICES, default="created")
    operator = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        related_name="development_tasks",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    published_at = models.DateTimeField(null=True, blank=True)
    started_at = models.DateTimeField(null=True, blank=True)
    finished_at = models.DateTimeField(null=True, blank=True)
    cancel_requested_at = models.DateTimeField(null=True, blank=True)
    cancel_published_at = models.DateTimeField(null=True, blank=True)
    exit_code = models.IntegerField(null=True, blank=True)
    error_message = models.TextField(blank=True)
    last_message = models.TextField(blank=True)
    codex_thread_id = models.CharField(max_length=128, blank=True)

    class Meta:
        ordering = ["-created_at"]
        indexes = [
            models.Index(fields=["robot", "status"], name="dev_task_robot_status_idx"),
            models.Index(fields=["-created_at"], name="dev_task_recent_idx"),
        ]


class DevelopmentTaskEvent(models.Model):
    task = models.ForeignKey(DevelopmentTask, related_name="events", on_delete=models.CASCADE)
    sequence = models.PositiveBigIntegerField()
    event_type = models.CharField(max_length=32)
    stream = models.CharField(max_length=16, blank=True)
    text = models.TextField(blank=True)
    payload = models.JSONField(default=dict, blank=True)
    occurred_at = models.DateTimeField(default=timezone.now)
    received_at = models.DateTimeField(default=timezone.now)

    class Meta:
        ordering = ["sequence"]
        constraints = [
            models.UniqueConstraint(fields=["task", "sequence"], name="uniq_dev_task_event_sequence")
        ]


class VoiceRecognitionEvent(BaseTimestampModel):
    """Text-only audit trail for each development voice-recognition attempt."""

    OUTCOME_CHOICES = [
        ("accepted", "已创建开发任务"),
        ("armed", "已唤醒，等待指令"),
        ("busy", "已有开发任务执行中"),
        ("ignored", "未命中唤醒词"),
        ("no_speech", "未识别到有效语音"),
    ]

    robot = models.ForeignKey(Robot, related_name="voice_recognition_events", on_delete=models.CASCADE)
    task = models.ForeignKey(
        DevelopmentTask,
        related_name="voice_recognition_events",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    transcript = models.TextField(blank=True)
    command = models.TextField(blank=True)
    asr_engine = models.CharField(max_length=64, blank=True)
    outcome = models.CharField(max_length=16, choices=OUTCOME_CHOICES)

    class Meta:
        ordering = ["-created_at"]
        indexes = [
            models.Index(fields=["robot", "-created_at"], name="voice_rec_robot_time_idx"),
        ]


class ValidationRecording(BaseTimestampModel):
    """Immutable rosbag/MCAP input registered for replay validation."""

    FORMAT_CHOICES = [("mcap", "MCAP"), ("sqlite3", "rosbag2 SQLite3")]
    STATE_CHOICES = [
        ("registered", "已登记"),
        ("uploading", "上传中"),
        ("ready", "可检查"),
        ("invalid", "无效"),
    ]

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    robot = models.ForeignKey(
        Robot, related_name="validation_recordings", on_delete=models.SET_NULL, null=True, blank=True
    )
    task_execution = models.ForeignKey(
        TaskExecution,
        related_name="validation_recordings",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    label = models.CharField(max_length=160)
    storage_format = models.CharField(max_length=16, choices=FORMAT_CHOICES, default="mcap")
    state = models.CharField(max_length=16, choices=STATE_CHOICES, default="registered")
    object_key = models.CharField(max_length=512, blank=True)
    sha256 = models.CharField(max_length=64, blank=True)
    size_bytes = models.PositiveBigIntegerField(default=0)
    duration_seconds = models.FloatField(default=0)
    start_time_ns = models.BigIntegerField(default=0)
    end_time_ns = models.BigIntegerField(default=0)
    topic_manifest = models.JSONField(default=list, blank=True)
    recording_manifest = models.JSONField(default=dict, blank=True)
    source_path_hint = models.CharField(max_length=512, blank=True)
    upload_id = models.CharField(max_length=512, blank=True)
    uploaded_at = models.DateTimeField(null=True, blank=True)
    invalid_reason = models.TextField(blank=True)

    class Meta:
        ordering = ["-created_at"]
        indexes = [
            models.Index(fields=["state", "-created_at"], name="val_record_state_idx"),
            models.Index(fields=["robot", "-created_at"], name="val_record_robot_idx"),
        ]


class ValidationProfile(BaseTimestampModel):
    MODE_CHOICES = [("bag_replay", "Bag 重算"), ("matrix_scenario", "MATRiX/UE 场景仿真")]

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    name = models.CharField(max_length=128)
    version = models.PositiveIntegerField(default=1)
    mode = models.CharField(max_length=24, choices=MODE_CHOICES, default="bag_replay")
    enabled = models.BooleanField(default=True)
    description = models.TextField(blank=True)
    required_topics = models.JSONField(default=list, blank=True)
    replay_topics = models.JSONField(default=list, blank=True)
    output_topics = models.JSONField(default=list, blank=True)
    thresholds = models.JSONField(default=dict, blank=True)
    runner_config = models.JSONField(default=dict, blank=True)

    class Meta:
        ordering = ["name", "-version"]
        constraints = [
            models.UniqueConstraint(fields=["name", "version"], name="uniq_validation_profile_ver")
        ]


class ValidationRunner(BaseTimestampModel):
    KIND_CHOICES = [("cpu", "CPU Replay Runner"), ("matrix", "MATRiX/UE GPU Runner")]
    STATE_CHOICES = [("offline", "离线"), ("idle", "空闲"), ("busy", "忙碌"), ("disabled", "停用")]

    id = models.CharField(primary_key=True, max_length=64)
    display_name = models.CharField(max_length=128, blank=True)
    kind = models.CharField(max_length=16, choices=KIND_CHOICES, default="cpu")
    state = models.CharField(max_length=16, choices=STATE_CHOICES, default="offline")
    capabilities = models.JSONField(default=list, blank=True)
    version = models.CharField(max_length=128, blank=True)
    last_seen_at = models.DateTimeField(null=True, blank=True)
    private_address = models.CharField(max_length=256, blank=True)

    class Meta:
        ordering = ["id"]


class ValidationJob(BaseTimestampModel):
    MODE_CHOICES = ValidationProfile.MODE_CHOICES
    STATE_CHOICES = [
        ("queued", "排队中"),
        ("staging", "准备中"),
        ("running", "运行中"),
        ("analyzing", "分析中"),
        ("uploading", "上传结果"),
        ("cancelling", "取消中"),
        ("completed", "已完成"),
        ("cancelled", "已取消"),
        ("infra_error", "基础设施失败"),
    ]
    VERDICT_CHOICES = [
        ("NOT_EVALUATED", "未判定"),
        ("PASS", "通过"),
        ("WARN", "告警"),
        ("FAIL", "失败"),
    ]
    TERMINAL_STATES = {"completed", "cancelled", "infra_error"}

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    recording = models.ForeignKey(
        ValidationRecording, related_name="validation_jobs", on_delete=models.PROTECT
    )
    profile = models.ForeignKey(ValidationProfile, related_name="validation_jobs", on_delete=models.PROTECT)
    baseline_job = models.ForeignKey(
        "self", related_name="comparison_jobs", on_delete=models.SET_NULL, null=True, blank=True
    )
    mode = models.CharField(max_length=24, choices=MODE_CHOICES, default="bag_replay")
    state = models.CharField(max_length=24, choices=STATE_CHOICES, default="queued")
    verdict = models.CharField(max_length=16, choices=VERDICT_CHOICES, default="NOT_EVALUATED")
    progress_percent = models.PositiveSmallIntegerField(default=0)
    runner = models.ForeignKey(
        ValidationRunner, related_name="validation_jobs", on_delete=models.SET_NULL, null=True, blank=True
    )
    requested_by = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        related_name="validation_jobs",
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
    )
    idempotency_key = models.CharField(max_length=128, blank=True)
    attempt_count = models.PositiveIntegerField(default=0)
    lease_token = models.UUIDField(null=True, blank=True)
    lease_expires_at = models.DateTimeField(null=True, blank=True)
    requested_config = models.JSONField(default=dict, blank=True)
    resolved_config = models.JSONField(default=dict, blank=True)
    summary = models.JSONField(default=dict, blank=True)
    error_code = models.CharField(max_length=64, blank=True)
    error_message = models.TextField(blank=True)
    stack_git_sha = models.CharField(max_length=64, blank=True)
    stack_image_digest = models.CharField(max_length=160, blank=True)
    live_bridge_url = models.CharField(max_length=512, blank=True)
    queued_at = models.DateTimeField(default=timezone.now)
    started_at = models.DateTimeField(null=True, blank=True)
    finished_at = models.DateTimeField(null=True, blank=True)
    cancel_requested_at = models.DateTimeField(null=True, blank=True)

    class Meta:
        ordering = ["-created_at"]
        indexes = [
            models.Index(fields=["state", "queued_at"], name="val_job_queue_idx"),
            models.Index(fields=["runner", "state"], name="val_job_runner_idx"),
        ]
        constraints = [
            models.UniqueConstraint(
                fields=["requested_by", "idempotency_key"],
                condition=~Q(idempotency_key=""),
                name="uniq_validation_idempotency",
            ),
            models.CheckConstraint(
                condition=Q(progress_percent__gte=0) & Q(progress_percent__lte=100),
                name="val_job_progress_range",
            ),
        ]


class ValidationAttempt(BaseTimestampModel):
    job = models.ForeignKey(ValidationJob, related_name="attempts", on_delete=models.CASCADE)
    number = models.PositiveIntegerField()
    runner = models.ForeignKey(ValidationRunner, related_name="attempts", on_delete=models.PROTECT)
    state = models.CharField(max_length=24, default="staging")
    lease_token = models.UUIDField(default=uuid.uuid4)
    started_at = models.DateTimeField(default=timezone.now)
    finished_at = models.DateTimeField(null=True, blank=True)
    error_code = models.CharField(max_length=64, blank=True)
    error_message = models.TextField(blank=True)
    runtime_metrics = models.JSONField(default=dict, blank=True)

    class Meta:
        ordering = ["number"]
        constraints = [
            models.UniqueConstraint(fields=["job", "number"], name="uniq_validation_attempt")
        ]


class ValidationArtifact(BaseTimestampModel):
    ROLE_CHOICES = [
        ("source", "输入录制"),
        ("result_mcap", "结果 MCAP"),
        ("report", "检查报告"),
        ("log", "运行日志"),
        ("metrics", "指标"),
        ("evidence", "证据"),
    ]

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    job = models.ForeignKey(ValidationJob, related_name="artifacts", on_delete=models.CASCADE)
    role = models.CharField(max_length=24, choices=ROLE_CHOICES)
    name = models.CharField(max_length=256)
    object_key = models.CharField(max_length=512)
    sha256 = models.CharField(max_length=64, blank=True)
    size_bytes = models.PositiveBigIntegerField(default=0)
    content_type = models.CharField(max_length=128, default="application/octet-stream")
    metadata = models.JSONField(default=dict, blank=True)

    class Meta:
        ordering = ["role", "name"]
        constraints = [
            models.UniqueConstraint(fields=["job", "role", "name"], name="uniq_validation_artifact")
        ]


class ValidationCheckResult(BaseTimestampModel):
    STATUS_CHOICES = [
        ("PASS", "通过"),
        ("WARN", "告警"),
        ("FAIL", "失败"),
        ("NOT_EVALUATED", "未判定"),
    ]

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    job = models.ForeignKey(ValidationJob, related_name="check_results", on_delete=models.CASCADE)
    rule_id = models.CharField(max_length=128)
    title = models.CharField(max_length=256)
    category = models.CharField(max_length=64)
    status = models.CharField(max_length=16, choices=STATUS_CHOICES)
    severity = models.CharField(max_length=16, default="error")
    hard_failure = models.BooleanField(default=False)
    metric_name = models.CharField(max_length=128, blank=True)
    actual_value = models.JSONField(null=True, blank=True)
    expected_value = models.JSONField(null=True, blank=True)
    start_time_ns = models.BigIntegerField(null=True, blank=True)
    end_time_ns = models.BigIntegerField(null=True, blank=True)
    topics = models.JSONField(default=list, blank=True)
    evidence = models.JSONField(default=dict, blank=True)
    message = models.TextField(blank=True)

    class Meta:
        ordering = ["category", "rule_id"]
        constraints = [
            models.UniqueConstraint(fields=["job", "rule_id"], name="uniq_validation_check")
        ]


class GoldenBaseline(BaseTimestampModel):
    profile = models.ForeignKey(ValidationProfile, related_name="golden_baselines", on_delete=models.PROTECT)
    job = models.OneToOneField(ValidationJob, related_name="golden_baseline", on_delete=models.PROTECT)
    map_hash = models.CharField(max_length=64, blank=True)
    route_hash = models.CharField(max_length=64, blank=True)
    stack_image_digest = models.CharField(max_length=160)
    active = models.BooleanField(default=True)
    approved_by = models.ForeignKey(
        settings.AUTH_USER_MODEL,
        related_name="approved_validation_baselines",
        on_delete=models.PROTECT,
    )

    class Meta:
        ordering = ["-created_at"]
        indexes = [models.Index(fields=["profile", "active"], name="golden_profile_active_idx")]
