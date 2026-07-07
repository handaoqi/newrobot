import django.db.models.deletion
import django.utils.timezone
import uuid
from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0009_mapdata_patrolroute_track_zone")]

    operations = [
        migrations.AddField("robot", "connection_status", models.CharField(choices=[("unknown", "未知"), ("online", "在线"), ("offline", "离线")], default="unknown", max_length=16)),
        migrations.AddField("robot", "agent_version", models.CharField(blank=True, max_length=64)),
        migrations.AddField("robot", "capabilities", models.JSONField(blank=True, default=list)),
        migrations.AddField("robot", "localization_status", models.CharField(choices=[("unknown", "未知"), ("initializing", "初始化"), ("relocalizing", "重定位"), ("relocalized", "重定位成功"), ("normal", "正常"), ("lost", "丢失")], default="unknown", max_length=24)),
        migrations.AddField("robot", "ros_ready", models.BooleanField(default=False)),
        migrations.AddField("robot", "nav_ready", models.BooleanField(default=False)),
        migrations.AddField("robot", "control_mode", models.CharField(choices=[("unknown", "未知"), ("autonomous", "自主"), ("manual_takeover", "人工接管"), ("emergency_stop", "急停")], default="unknown", max_length=24)),
        migrations.AddField("robot", "current_map_id", models.CharField(blank=True, max_length=128)),
        migrations.AddField("robot", "current_map_version", models.CharField(blank=True, max_length=64)),
        migrations.AddField("robot", "last_state_version", models.BigIntegerField(default=0)),
        migrations.AddField("robot", "last_seen_at", models.DateTimeField(blank=True, null=True)),
        migrations.CreateModel(
            name="RobotCredential",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("id", models.UUIDField(default=uuid.uuid4, editable=False, primary_key=True, serialize=False)),
                ("credential_id", models.CharField(max_length=128, unique=True)),
                ("secret_hash", models.CharField(blank=True, max_length=256)),
                ("certificate_fingerprint", models.CharField(blank=True, max_length=128)),
                ("status", models.CharField(choices=[("active", "有效"), ("revoked", "已吊销"), ("expired", "已过期")], default="active", max_length=16)),
                ("issued_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("expires_at", models.DateTimeField(blank=True, null=True)),
                ("revoked_at", models.DateTimeField(blank=True, null=True)),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="credentials", to="monitoring.robot")),
            ],
        ),
        migrations.CreateModel(
            name="RobotSession",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("id", models.UUIDField(default=uuid.uuid4, editable=False, primary_key=True, serialize=False)),
                ("session_id", models.UUIDField(unique=True)),
                ("transport", models.CharField(choices=[("mqtt", "MQTT"), ("wss", "WSS")], default="mqtt", max_length=8)),
                ("connected_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("last_heartbeat_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("disconnected_at", models.DateTimeField(blank=True, null=True)),
                ("disconnect_reason", models.CharField(blank=True, max_length=128)),
                ("agent_version", models.CharField(blank=True, max_length=64)),
                ("boot_id", models.CharField(blank=True, max_length=128)),
                ("remote_ip", models.GenericIPAddressField(blank=True, null=True)),
                ("capabilities", models.JSONField(blank=True, default=list)),
                ("metadata", models.JSONField(blank=True, default=dict)),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="sessions", to="monitoring.robot")),
            ],
            options={"ordering": ["-connected_at"]},
        ),
        migrations.CreateModel(
            name="InboundMessage",
            fields=[
                ("message_id", models.UUIDField(primary_key=True, serialize=False)),
                ("session_id", models.UUIDField()),
                ("message_type", models.CharField(max_length=64)),
                ("sequence", models.BigIntegerField(blank=True, null=True)),
                ("topic", models.CharField(max_length=256)),
                ("received_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("processed_at", models.DateTimeField(blank=True, null=True)),
                ("process_status", models.CharField(choices=[("pending", "待处理"), ("processed", "已处理"), ("failed", "失败"), ("ignored", "忽略")], default="pending", max_length=16)),
                ("error_message", models.TextField(blank=True)),
                ("raw_payload", models.JSONField(default=dict)),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="inbound_messages", to="monitoring.robot")),
            ],
        ),
        migrations.AddIndex("robotsession", models.Index(fields=["robot", "-connected_at"], name="robot_session_recent_idx")),
    ]
