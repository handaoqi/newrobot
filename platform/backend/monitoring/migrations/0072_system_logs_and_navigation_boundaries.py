import uuid

from django.conf import settings
from django.db import migrations, models
import django.db.models.deletion
import django.utils.timezone


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0071_inbound_message_retention_index"),
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
    ]

    operations = [
        migrations.AddField(
            model_name="zone",
            name="speed_limit_mps",
            field=models.FloatField(blank=True, null=True, verbose_name="限速值(m/s)"),
        ),
        migrations.AddField(
            model_name="zone",
            name="warning_distance_m",
            field=models.FloatField(default=0.5, verbose_name="提前警告距离(m)"),
        ),
        migrations.CreateModel(
            name="MapNavigationBoundary",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("outer_polygon", models.JSONField(blank=True, default=list)),
                ("safety_margin_m", models.FloatField(default=0.2)),
                ("revision", models.PositiveIntegerField(default=0)),
                ("active_revision", models.PositiveIntegerField(default=0)),
                ("active_payload", models.JSONField(blank=True, default=dict)),
                ("apply_status", models.CharField(choices=[("unconfigured", "未配置"), ("draft", "草稿"), ("applying", "应用中"), ("active", "已生效"), ("failed", "应用失败")], default="unconfigured", max_length=16)),
                ("apply_error", models.TextField(blank=True)),
                ("apply_command", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="boundary_applications", to="monitoring.remotecommand")),
                ("map_data", models.OneToOneField(on_delete=django.db.models.deletion.CASCADE, related_name="navigation_boundary", to="monitoring.mapdata")),
            ],
            options={"ordering": ["map_data_id"]},
        ),
        migrations.CreateModel(
            name="DebugLogSession",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("id", models.UUIDField(default=uuid.uuid4, editable=False, primary_key=True, serialize=False)),
                ("modules", models.JSONField(default=list)),
                ("sample_hz", models.FloatField(default=1.0)),
                ("status", models.CharField(choices=[("starting", "启动中"), ("active", "已启用"), ("stopped", "已停止"), ("expired", "已到期"), ("failed", "失败")], default="starting", max_length=16)),
                ("started_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("expires_at", models.DateTimeField()),
                ("stopped_at", models.DateTimeField(blank=True, null=True)),
                ("created_by", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="debug_log_sessions", to=settings.AUTH_USER_MODEL)),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="debug_log_sessions", to="monitoring.robot")),
            ],
            options={"ordering": ["-started_at"]},
        ),
        migrations.AddIndex(
            model_name="debuglogsession",
            index=models.Index(fields=["robot", "-expires_at"], name="debug_log_robot_exp_idx"),
        ),
        migrations.CreateModel(
            name="SystemLog",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("id", models.UUIDField(default=uuid.uuid4, editable=False, primary_key=True, serialize=False)),
                ("occurred_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("received_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("level", models.CharField(choices=[("DEBUG", "DEBUG"), ("INFO", "INFO"), ("WARNING", "WARNING"), ("ERROR", "ERROR")], max_length=8)),
                ("module", models.CharField(choices=[("localization", "定位"), ("navigation", "导航"), ("avoidance", "避障"), ("relocalization", "主动重定位"), ("waypoint", "航点调整"), ("planner", "路径规划"), ("boundary", "导航边界"), ("system", "系统")], max_length=24)),
                ("event_code", models.CharField(max_length=96)),
                ("message", models.CharField(max_length=500)),
                ("source", models.CharField(default="center", max_length=64)),
                ("data", models.JSONField(blank=True, default=dict)),
                ("trace_id", models.UUIDField(blank=True, db_index=True, null=True)),
                ("waypoint_index", models.IntegerField(blank=True, null=True)),
                ("x", models.FloatField(blank=True, null=True)),
                ("y", models.FloatField(blank=True, null=True)),
                ("yaw", models.FloatField(blank=True, null=True)),
                ("repeat_count", models.PositiveIntegerField(default=1)),
                ("command", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="system_logs", to="monitoring.remotecommand")),
                ("map_data", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="system_logs", to="monitoring.mapdata")),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="system_logs", to="monitoring.robot")),
                ("task_execution", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="system_logs", to="monitoring.taskexecution")),
            ],
            options={"ordering": ["-occurred_at", "-id"]},
        ),
        migrations.AddIndex(model_name="systemlog", index=models.Index(fields=["robot", "-occurred_at"], name="syslog_robot_time_idx")),
        migrations.AddIndex(model_name="systemlog", index=models.Index(fields=["robot", "level", "-occurred_at"], name="syslog_level_time_idx")),
        migrations.AddIndex(model_name="systemlog", index=models.Index(fields=["task_execution", "-occurred_at"], name="syslog_task_time_idx")),
        migrations.AlterField(
            model_name="remotecommand",
            name="command_type",
            field=models.CharField(choices=[
                ("task.start", "启动任务"), ("task.pause", "暂停任务"), ("task.resume", "继续任务"), ("task.resume_forward", "恢复前向"), ("task.cancel", "终止任务"), ("task.force_exit", "强制退出并清理任务"),
                ("mapping.start", "开始建图"), ("mapping.save", "停止并保存地图"), ("mapping.cancel", "取消建图"), ("mapping.status", "查询建图状态"), ("mapping.origin_start", "锁定 ENU 原点"), ("mapping.origin_cancel", "取消原点锁定"), ("mapping.origin_extract_global", "提取全局 ENU"), ("mapping.slam_start", "启动 SLAM 预热"), ("mapping.begin", "确认并开始正式建图"),
                ("nav.status", "查询导航状态"), ("nav.start", "启动导航栈"), ("nav.restart", "重启导航栈"), ("nav.recover", "恢复导航栈"), ("nav.stop", "停止导航栈"), ("nav.initial_pose", "设置初始定位"), ("nav.single_goal", "单点导航"), ("nav.relocalize", "主动重定位"), ("diagnostics.log_config", "配置诊断日志"), ("map.activate", "切换活动地图"), ("map.boundary_apply", "应用导航边界"), ("map.optimize", "离线回环优化"), ("sensor.restart", "重启传感器"),
                ("charge.start", "开始充电"), ("charge.stop", "断开充电"), ("motion.start", "启动运控"), ("motion.stop", "停止运控"), ("audio.volume", "调节扬声器音量"),
                ("teleop.takeover_enter", "进入远程接管"), ("teleop.takeover_exit", "退出远程接管"), ("teleop.stand_up", "站立"), ("teleop.lie_down", "趴下"), ("teleop.shake_hand", "打招呼"), ("teleop.two_leg_stand", "双腿站立"), ("teleop.crawl_forward", "匍匐前进"), ("teleop.speed_micro", "微速档"), ("teleop.speed_slow", "低速档"), ("teleop.speed_normal", "中速档"), ("teleop.speed_fast", "高速档"), ("teleop.move_forward", "前进"), ("teleop.move_backward", "后退"), ("teleop.move_left", "左移"), ("teleop.move_right", "右移"), ("teleop.turn_left", "左转"), ("teleop.turn_right", "右转"), ("teleop.move_velocity", "跟随速度"), ("teleop.move_stop", "停止移动"), ("teleop.passive", "软急停"), ("teleop.skill", "执行遥控技能"), ("teleop.skill_list", "列出遥控技能"), ("teleop.skill_status", "查询遥控技能"), ("teleop.skill_cancel", "取消遥控技能"), ("teleop.person_follow_start", "启动人员跟随"), ("teleop.person_follow_stop", "停止人员跟随"), ("teleop.person_follow_status", "查询人员跟随"),
            ], max_length=32),
        ),
    ]
