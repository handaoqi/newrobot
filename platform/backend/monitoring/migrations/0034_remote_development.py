import django.db.models.deletion
import uuid

from django.conf import settings
from django.db import migrations, models
import django.utils.timezone


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0033_follow_velocity_commands"),
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
    ]

    operations = [
        migrations.CreateModel(
            name="DevelopmentAgentState",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("status", models.CharField(default="offline", max_length=16)),
                ("agent_version", models.CharField(blank=True, max_length=64)),
                ("codex_binary", models.CharField(blank=True, max_length=512)),
                ("workspaces", models.JSONField(blank=True, default=list)),
                ("last_seen_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("robot", models.OneToOneField(on_delete=django.db.models.deletion.CASCADE, primary_key=True, related_name="development_agent_state", serialize=False, to="monitoring.robot")),
            ],
            options={"abstract": False},
        ),
        migrations.CreateModel(
            name="DevelopmentTask",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("id", models.UUIDField(default=uuid.uuid4, editable=False, primary_key=True, serialize=False)),
                ("workspace", models.CharField(max_length=64)),
                ("prompt", models.TextField()),
                ("status", models.CharField(choices=[("created", "待下发"), ("published", "已下发"), ("running", "执行中"), ("cancelling", "取消中"), ("succeeded", "已完成"), ("failed", "失败"), ("cancelled", "已取消"), ("timed_out", "超时"), ("rejected", "已拒绝")], default="created", max_length=16)),
                ("published_at", models.DateTimeField(blank=True, null=True)),
                ("started_at", models.DateTimeField(blank=True, null=True)),
                ("finished_at", models.DateTimeField(blank=True, null=True)),
                ("cancel_requested_at", models.DateTimeField(blank=True, null=True)),
                ("cancel_published_at", models.DateTimeField(blank=True, null=True)),
                ("exit_code", models.IntegerField(blank=True, null=True)),
                ("error_message", models.TextField(blank=True)),
                ("last_message", models.TextField(blank=True)),
                ("operator", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="development_tasks", to=settings.AUTH_USER_MODEL)),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.PROTECT, related_name="development_tasks", to="monitoring.robot")),
            ],
            options={
                "ordering": ["-created_at"],
                "indexes": [models.Index(fields=["robot", "status"], name="dev_task_robot_status_idx"), models.Index(fields=["-created_at"], name="dev_task_recent_idx")],
            },
        ),
        migrations.CreateModel(
            name="DevelopmentTaskEvent",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("sequence", models.PositiveBigIntegerField()),
                ("event_type", models.CharField(max_length=32)),
                ("stream", models.CharField(blank=True, max_length=16)),
                ("text", models.TextField(blank=True)),
                ("payload", models.JSONField(blank=True, default=dict)),
                ("occurred_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("received_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("task", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="events", to="monitoring.developmenttask")),
            ],
            options={
                "ordering": ["sequence"],
                "constraints": [models.UniqueConstraint(fields=("task", "sequence"), name="uniq_dev_task_event_sequence")],
            },
        ),
    ]
