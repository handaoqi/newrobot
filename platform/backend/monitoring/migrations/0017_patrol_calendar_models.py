import uuid

import django.db.models.deletion
from django.conf import settings
from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
        ("monitoring", "0016_p0_constraints_and_indexes"),
    ]

    operations = [
        migrations.CreateModel(
            name="CalendarDay",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("date", models.DateField(unique=True)),
                ("name", models.CharField(max_length=128)),
                (
                    "day_type",
                    models.CharField(
                        choices=[("holiday", "节假日"), ("workday", "调休日"), ("event_day", "特殊活动日")],
                        default="holiday",
                        max_length=16,
                    ),
                ),
                ("enabled", models.BooleanField(default=True)),
                ("note", models.TextField(blank=True)),
            ],
            options={"ordering": ["date"]},
        ),
        migrations.CreateModel(
            name="PatrolSchedule",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("name", models.CharField(max_length=128)),
                (
                    "schedule_type",
                    models.CharField(
                        choices=[("daily", "日常计划"), ("holiday", "节假日计划"), ("once", "一次性计划")],
                        default="daily",
                        max_length=16,
                    ),
                ),
                ("time_of_day", models.TimeField()),
                ("weekdays", models.JSONField(blank=True, default=list)),
                ("run_date", models.DateField(blank=True, null=True)),
                ("priority", models.IntegerField(default=10)),
                ("enabled", models.BooleanField(default=True)),
                ("last_triggered_at", models.DateTimeField(blank=True, null=True)),
                ("note", models.TextField(blank=True)),
                (
                    "created_by",
                    models.ForeignKey(
                        blank=True,
                        null=True,
                        on_delete=django.db.models.deletion.SET_NULL,
                        related_name="created_patrol_schedules",
                        to=settings.AUTH_USER_MODEL,
                    ),
                ),
                (
                    "map_data",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.PROTECT,
                        related_name="calendar_schedules",
                        to="monitoring.mapdata",
                    ),
                ),
                (
                    "robot",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.CASCADE,
                        related_name="patrol_schedules",
                        to="monitoring.robot",
                    ),
                ),
                (
                    "route",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.PROTECT,
                        related_name="calendar_schedules",
                        to="monitoring.patrolroute",
                    ),
                ),
                (
                    "task_template",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.PROTECT,
                        related_name="calendar_schedules",
                        to="monitoring.patroltask",
                    ),
                ),
            ],
            options={"ordering": ["-enabled", "time_of_day", "-priority", "name"]},
        ),
        migrations.CreateModel(
            name="ScheduleRun",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("id", models.UUIDField(default=uuid.uuid4, editable=False, primary_key=True, serialize=False)),
                ("planned_start_at", models.DateTimeField()),
                ("triggered_at", models.DateTimeField(blank=True, null=True)),
                (
                    "status",
                    models.CharField(
                        choices=[("created", "已创建"), ("dispatched", "已下发"), ("skipped", "已跳过"), ("failed", "失败")],
                        default="created",
                        max_length=16,
                    ),
                ),
                ("skip_reason", models.CharField(blank=True, max_length=64)),
                ("error_message", models.TextField(blank=True)),
                (
                    "remote_command",
                    models.ForeignKey(
                        blank=True,
                        null=True,
                        on_delete=django.db.models.deletion.SET_NULL,
                        related_name="schedule_runs",
                        to="monitoring.remotecommand",
                    ),
                ),
                (
                    "robot",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.CASCADE,
                        related_name="schedule_runs",
                        to="monitoring.robot",
                    ),
                ),
                (
                    "schedule",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.CASCADE,
                        related_name="runs",
                        to="monitoring.patrolschedule",
                    ),
                ),
                (
                    "task_execution",
                    models.ForeignKey(
                        blank=True,
                        null=True,
                        on_delete=django.db.models.deletion.SET_NULL,
                        related_name="schedule_runs",
                        to="monitoring.taskexecution",
                    ),
                ),
            ],
            options={"ordering": ["-planned_start_at"]},
        ),
        migrations.AddIndex(
            model_name="patrolschedule",
            index=models.Index(fields=["enabled", "schedule_type", "time_of_day"], name="patrol_sched_due_idx"),
        ),
        migrations.AddIndex(
            model_name="patrolschedule",
            index=models.Index(fields=["robot", "enabled"], name="patrol_sched_robot_idx"),
        ),
        migrations.AddIndex(
            model_name="schedulerun",
            index=models.Index(fields=["robot", "-planned_start_at"], name="sched_run_robot_time_idx"),
        ),
        migrations.AddIndex(
            model_name="schedulerun",
            index=models.Index(fields=["status", "-planned_start_at"], name="sched_run_status_time_idx"),
        ),
        migrations.AddConstraint(
            model_name="schedulerun",
            constraint=models.UniqueConstraint(fields=("schedule", "planned_start_at"), name="uniq_schedule_planned_start"),
        ),
    ]
