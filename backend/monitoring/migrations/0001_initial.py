from django.db import migrations, models
import django.db.models.deletion
import django.utils.timezone


class Migration(migrations.Migration):
    initial = True

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name="Robot",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("code", models.CharField(max_length=32, unique=True)),
                ("name", models.CharField(max_length=64)),
                ("location", models.CharField(max_length=128)),
                ("area", models.CharField(max_length=128)),
                ("status", models.CharField(choices=[("online", "在线"), ("offline", "离线"), ("warning", "告警"), ("charging", "充电中")], default="online", max_length=16)),
                ("mode", models.CharField(choices=[("auto", "AI自主巡检"), ("manual", "人工接管"), ("standby", "待命"), ("returning", "返航")], default="auto", max_length=16)),
                ("battery_level", models.PositiveSmallIntegerField(default=100)),
                ("network_strength", models.PositiveSmallIntegerField(default=100)),
                ("speaker_volume", models.PositiveSmallIntegerField(default=84)),
                ("patrol_duration_minutes", models.PositiveIntegerField(default=0)),
                ("today_alerts", models.PositiveIntegerField(default=0)),
                ("current_task_name", models.CharField(blank=True, max_length=128)),
                ("firmware_version", models.CharField(default="v1.0.0", max_length=32)),
                ("last_heartbeat_at", models.DateTimeField(default=django.utils.timezone.now)),
            ],
            options={"ordering": ["code"]},
        ),
        migrations.CreateModel(
            name="InspectionEvent",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("title", models.CharField(max_length=128)),
                ("event_type", models.CharField(max_length=64)),
                ("location", models.CharField(max_length=128)),
                ("detected_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("confidence", models.DecimalField(decimal_places=2, default=0, max_digits=5)),
                ("risk_level", models.CharField(choices=[("high", "高"), ("medium", "中"), ("low", "低")], default="medium", max_length=16)),
                ("status", models.CharField(choices=[("pending", "待处理"), ("processing", "处理中"), ("resolved", "已完成"), ("false_alarm", "误报")], default="pending", max_length=16)),
                ("snapshot_url", models.URLField(blank=True)),
                ("description", models.TextField(blank=True)),
                ("handling_notes", models.TextField(blank=True)),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="events", to="monitoring.robot")),
            ],
            options={"ordering": ["-detected_at"]},
        ),
        migrations.CreateModel(
            name="PatrolTask",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("name", models.CharField(max_length=128)),
                ("route_name", models.CharField(max_length=128)),
                ("scheduled_start", models.DateTimeField()),
                ("scheduled_end", models.DateTimeField()),
                ("status", models.CharField(choices=[("pending", "待执行"), ("running", "执行中"), ("completed", "已完成"), ("paused", "已暂停")], default="pending", max_length=16)),
                ("completion_rate", models.PositiveSmallIntegerField(default=0)),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="tasks", to="monitoring.robot")),
            ],
            options={"ordering": ["-scheduled_start"]},
        ),
        migrations.CreateModel(
            name="RobotTelemetry",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("sequence_id", models.CharField(max_length=64)),
                ("position_name", models.CharField(max_length=128)),
                ("latitude", models.DecimalField(blank=True, decimal_places=6, max_digits=10, null=True)),
                ("longitude", models.DecimalField(blank=True, decimal_places=6, max_digits=10, null=True)),
                ("heading", models.DecimalField(blank=True, decimal_places=2, max_digits=6, null=True)),
                ("speed", models.DecimalField(blank=True, decimal_places=2, max_digits=6, null=True)),
                ("battery_level", models.PositiveSmallIntegerField(default=0)),
                ("network_strength", models.PositiveSmallIntegerField(default=0)),
                ("raw_payload", models.JSONField(blank=True, default=dict)),
                ("reported_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="telemetry_records", to="monitoring.robot")),
            ],
            options={"ordering": ["-reported_at"]},
        ),
    ]
