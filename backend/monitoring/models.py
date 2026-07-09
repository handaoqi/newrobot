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
            ("manual_takeover", "人工接管"),
            ("emergency_stop", "急停"),
        ],
        default="unknown",
    )
    current_map_id = models.CharField(max_length=128, blank=True)
    current_map_version = models.CharField(max_length=64, blank=True)
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
        constraints = [
            models.UniqueConstraint(
                fields=["robot", "session_id", "sequence_id"],
                name="uniq_robot_session_telemetry_sequence",
            )
        ]

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
        ("move_forward", "前进"),
        ("move_backward", "后退"),
        ("move_left", "左移"),
        ("move_right", "右移"),
        ("turn_left", "左转"),
        ("turn_right", "右转"),
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


class MapData(BaseTimestampModel):
    """地图数据模型"""
    name = models.CharField(max_length=128, verbose_name="地图名称")
    robot = models.ForeignKey(Robot, on_delete=models.SET_NULL, null=True, blank=True, verbose_name="关联机器人", related_name="maps")
    pgm_file = models.FileField(upload_to='maps/', verbose_name="PGM地图文件", null=True, blank=True)
    yaml_file = models.FileField(upload_to='maps/', verbose_name="YAML配置文件", null=True, blank=True)
    thumbnail = models.ImageField(upload_to='maps/thumbnails/', null=True, blank=True, verbose_name="缩略图")
    resolution = models.FloatField(default=0.05, verbose_name="分辨率(m/像素)")
    width = models.IntegerField(default=0, verbose_name="宽度(像素)")
    height = models.IntegerField(default=0, verbose_name="高度(像素)")
    origin = models.JSONField(default=list, verbose_name="原点坐标 [x, y, theta]")
    active = models.BooleanField(default=False, verbose_name="是否为活动地图")
    description = models.TextField(blank=True, verbose_name="地图描述")

    class Meta:
        verbose_name = "地图数据"
        verbose_name_plural = "地图数据"
        ordering = ["-created_at"]

    def __str__(self) -> str:
        return self.name


class PatrolRoute(BaseTimestampModel):
    """巡逻路线模型"""
    name = models.CharField(max_length=128, verbose_name="路线名称")
    map_data = models.ForeignKey(MapData, on_delete=models.CASCADE, verbose_name="关联地图", related_name="routes")
    robot = models.ForeignKey(Robot, on_delete=models.CASCADE, verbose_name="关联机器人", related_name="routes")
    waypoints = models.JSONField(default=list, verbose_name="途经点坐标数组")  # [[x1,y1],[x2,y2],...]
    waypoint_names = models.JSONField(default=list, verbose_name="途经点名称数组")
    description = models.TextField(blank=True, verbose_name="路线描述")

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

    class Meta:
        verbose_name = "禁区"
        verbose_name_plural = "禁区"
        ordering = ["-created_at"]

    def __str__(self) -> str:
        return self.name


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
        ("task.cancel", "终止任务"),
        ("mapping.start", "开始建图"),
        ("mapping.save", "停止并保存地图"),
        ("mapping.cancel", "取消建图"),
        ("mapping.status", "查询建图状态"),
        ("nav.status", "查询导航状态"),
        ("nav.start", "启动导航栈"),
        ("nav.restart", "重启导航栈"),
        ("nav.recover", "恢复导航栈"),
        ("nav.stop", "停止导航栈"),
        ("nav.initial_pose", "设置初始定位"),
        ("map.activate", "切换活动地图"),
        ("teleop.takeover_enter", "进入远程接管"),
        ("teleop.takeover_exit", "退出远程接管"),
        ("teleop.stand_up", "站立"),
        ("teleop.lie_down", "趴下"),
        ("teleop.move_forward", "前进"),
        ("teleop.move_backward", "后退"),
        ("teleop.move_left", "左移"),
        ("teleop.move_right", "右移"),
        ("teleop.turn_left", "左转"),
        ("teleop.turn_right", "右转"),
        ("teleop.move_stop", "停止移动"),
        ("teleop.passive", "软急停"),
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
