from django.db import migrations, models
import django.db.models.deletion


def set_demo_control_endpoint(apps, _schema_editor):
    Robot = apps.get_model("monitoring", "Robot")
    Robot.objects.filter(code="ZSL-1A-07", control_endpoint="").update(
        control_endpoint="http://192.168.234.1:9100/commands"
    )


class Migration(migrations.Migration):

    dependencies = [
        ("monitoring", "0004_event_review_result"),
    ]

    operations = [
        migrations.AddField(
            model_name="robot",
            name="control_endpoint",
            field=models.URLField(blank=True),
        ),
        migrations.CreateModel(
            name="RobotCommand",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                (
                    "action",
                    models.CharField(
                        choices=[
                            ("shake_hand", "握手"),
                            ("stand_up", "站立"),
                            ("lie_down", "趴下"),
                            ("move_stop", "停止移动"),
                            ("passive", "软急停"),
                        ],
                        max_length=32,
                    ),
                ),
                ("payload", models.JSONField(blank=True, default=dict)),
                (
                    "status",
                    models.CharField(
                        choices=[("queued", "待发送"), ("sent", "已发送"), ("failed", "发送失败")],
                        default="queued",
                        max_length=16,
                    ),
                ),
                ("response_payload", models.JSONField(blank=True, default=dict)),
                ("error_message", models.TextField(blank=True)),
                ("sent_at", models.DateTimeField(blank=True, null=True)),
                (
                    "robot",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.CASCADE,
                        related_name="commands",
                        to="monitoring.robot",
                    ),
                ),
            ],
            options={
                "ordering": ["-created_at"],
            },
        ),
        migrations.RunPython(set_demo_control_endpoint, migrations.RunPython.noop),
    ]
