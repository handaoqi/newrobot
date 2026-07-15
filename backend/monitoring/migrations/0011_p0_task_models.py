import django.db.models.deletion
import django.utils.timezone
import uuid
from django.conf import settings
from django.db import migrations, models


TASK_STATES = [
    ("created", "已创建"), ("dispatching", "下发中"), ("accepted", "已接受"),
    ("running", "执行中"), ("pausing", "暂停中"), ("paused", "已暂停"),
    ("resuming", "继续中"), ("cancelling", "终止中"), ("completed", "已完成"),
    ("failed", "失败"), ("cancelled", "已终止"), ("timed_out", "超时"),
    ("interrupted", "中断待对账"), ("rejected", "已拒绝"),
]


def backfill_patrol_task_robots(apps, schema_editor):
    Robot = apps.get_model("monitoring", "Robot")
    PatrolTask = apps.get_model("monitoring", "PatrolTask")
    db_alias = schema_editor.connection.alias

    robot = Robot.objects.using(db_alias).order_by("id").first()
    if robot is None:
        robot = Robot.objects.using(db_alias).create(
            code="ZSL-1A-07",
            name="南入口巡检机器人",
            location="太阳宫园区",
            area="园区主通道",
        )

    PatrolTask.objects.using(db_alias).filter(robot__isnull=True).update(robot=robot)


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0010_p0_device_sessions_and_status"),
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
    ]

    operations = [
        migrations.RunPython(backfill_patrol_task_robots, migrations.RunPython.noop),
        migrations.AddField("patroltask", "route", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.PROTECT, related_name="task_templates", to="monitoring.patrolroute")),
        migrations.AddField("patroltask", "enabled", models.BooleanField(default=True)),
        migrations.AddField("patroltask", "description", models.TextField(blank=True)),
        migrations.AddField("patroltask", "created_by", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="created_patrol_tasks", to=settings.AUTH_USER_MODEL)),
        migrations.CreateModel(
            name="TaskExecution",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("id", models.UUIDField(default=uuid.uuid4, editable=False, primary_key=True, serialize=False)),
                ("route_snapshot", models.JSONField(default=dict)),
                ("state", models.CharField(choices=TASK_STATES, default="created", max_length=24)),
                ("state_version", models.BigIntegerField(default=0)),
                ("current_waypoint_index", models.PositiveIntegerField(blank=True, null=True)),
                ("current_waypoint_id", models.CharField(blank=True, max_length=128)),
                ("completed_waypoints", models.PositiveIntegerField(default=0)),
                ("total_waypoints", models.PositiveIntegerField(default=0)),
                ("distance_remaining_m", models.DecimalField(blank=True, decimal_places=3, max_digits=12, null=True)),
                ("estimated_time_remaining_s", models.PositiveIntegerField(blank=True, null=True)),
                ("started_at", models.DateTimeField(blank=True, null=True)),
                ("finished_at", models.DateTimeField(blank=True, null=True)),
                ("accepted_at", models.DateTimeField(blank=True, null=True)),
                ("paused_at", models.DateTimeField(blank=True, null=True)),
                ("failure_code", models.CharField(blank=True, max_length=64)),
                ("failure_message", models.TextField(blank=True)),
                ("last_edge_event_at", models.DateTimeField(blank=True, null=True)),
                ("created_by", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="created_task_executions", to=settings.AUTH_USER_MODEL)),
                ("map_data", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="task_executions", to="monitoring.mapdata")),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name="task_executions", to="monitoring.robot")),
                ("route", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="executions", to="monitoring.patrolroute")),
                ("task", models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name="executions", to="monitoring.patroltask")),
            ],
            options={"ordering": ["-created_at"]},
        ),
        migrations.CreateModel(
            name="TaskExecutionEvent",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("state", models.CharField(choices=TASK_STATES, max_length=24)),
                ("state_version", models.BigIntegerField()),
                ("event_type", models.CharField(max_length=64)),
                ("occurred_at", models.DateTimeField()),
                ("received_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("message_id", models.UUIDField(blank=True, null=True, unique=True)),
                ("reason_code", models.CharField(blank=True, max_length=64)),
                ("reason_message", models.TextField(blank=True)),
                ("payload", models.JSONField(blank=True, default=dict)),
                ("task_execution", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="events", to="monitoring.taskexecution")),
            ],
            options={"ordering": ["state_version"]},
        ),
        migrations.CreateModel(
            name="RemoteCommand",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("id", models.UUIDField(default=uuid.uuid4, editable=False, primary_key=True, serialize=False)),
                ("command_type", models.CharField(choices=[("task.start", "启动任务"), ("task.pause", "暂停任务"), ("task.resume", "继续任务"), ("task.cancel", "终止任务")], max_length=32)),
                ("payload", models.JSONField(default=dict)),
                ("status", models.CharField(choices=[("created", "已创建"), ("published", "已发布"), ("accepted", "已接受"), ("rejected", "已拒绝"), ("executing", "执行中"), ("succeeded", "成功"), ("failed", "失败"), ("cancelled", "已取消"), ("timed_out", "超时"), ("expired", "已过期")], default="created", max_length=16)),
                ("trace_id", models.UUIDField(db_index=True, default=uuid.uuid4)),
                ("issued_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("expires_at", models.DateTimeField()),
                ("published_at", models.DateTimeField(blank=True, null=True)),
                ("acknowledged_at", models.DateTimeField(blank=True, null=True)),
                ("started_at", models.DateTimeField(blank=True, null=True)),
                ("finished_at", models.DateTimeField(blank=True, null=True)),
                ("ack_reason_code", models.CharField(blank=True, max_length=64)),
                ("ack_reason_message", models.TextField(blank=True)),
                ("result_payload", models.JSONField(blank=True, default=dict)),
                ("error_code", models.CharField(blank=True, max_length=64)),
                ("error_message", models.TextField(blank=True)),
                ("retry_count", models.PositiveIntegerField(default=0)),
                ("operator", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="issued_remote_commands", to=settings.AUTH_USER_MODEL)),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name="remote_commands", to="monitoring.robot")),
                ("task_execution", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.CASCADE, related_name="commands", to="monitoring.taskexecution")),
            ],
            options={"ordering": ["-issued_at"]},
        ),
        migrations.CreateModel(
            name="CommandEvent",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("event_type", models.CharField(max_length=32)),
                ("event_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("source", models.CharField(choices=[("center", "中心"), ("edge", "边缘"), ("broker", "Broker")], max_length=16)),
                ("message_id", models.UUIDField(blank=True, null=True, unique=True)),
                ("payload", models.JSONField(blank=True, default=dict)),
                ("command", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="events", to="monitoring.remotecommand")),
            ],
            options={"ordering": ["event_at"]},
        ),
        migrations.CreateModel(
            name="RobotStatusLatest",
            fields=[
                ("robot", models.OneToOneField(on_delete=django.db.models.deletion.CASCADE, primary_key=True, related_name="latest_status", serialize=False, to="monitoring.robot")),
                ("state_version", models.BigIntegerField(default=0)),
                ("sampled_at", models.DateTimeField()),
                ("received_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("frame_id", models.CharField(default="map", max_length=32)),
                ("map_id", models.CharField(blank=True, max_length=128)),
                ("map_version", models.CharField(blank=True, max_length=64)),
                ("x", models.DecimalField(blank=True, decimal_places=4, max_digits=12, null=True)),
                ("y", models.DecimalField(blank=True, decimal_places=4, max_digits=12, null=True)),
                ("z", models.DecimalField(blank=True, decimal_places=4, max_digits=12, null=True)),
                ("yaw", models.DecimalField(blank=True, decimal_places=5, max_digits=10, null=True)),
                ("speed_mps", models.DecimalField(blank=True, decimal_places=5, max_digits=10, null=True)),
                ("localization_status", models.CharField(default="unknown", max_length=24)),
                ("localization_source_status", models.PositiveSmallIntegerField(blank=True, null=True)),
                ("power_available", models.BooleanField(default=False)),
                ("battery_percent", models.PositiveSmallIntegerField(blank=True, null=True)),
                ("charging", models.BooleanField(blank=True, null=True)),
                ("network_type", models.CharField(blank=True, max_length=32)),
                ("signal_percent", models.PositiveSmallIntegerField(blank=True, null=True)),
                ("ros_ready", models.BooleanField(default=False)),
                ("nav_ready", models.BooleanField(default=False)),
                ("emergency_stop", models.BooleanField(default=False)),
                ("control_mode", models.CharField(default="unknown", max_length=24)),
                ("raw_payload", models.JSONField(blank=True, default=dict)),
                ("task_execution", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="latest_robot_statuses", to="monitoring.taskexecution")),
            ],
        ),
    ]
