from django.db import migrations, models
import django.db.models.deletion
import django.utils.timezone


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0031_recordedaudio_asr")]

    operations = [
        migrations.CreateModel(
            name="RobotPersonDetectionState",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("camera_id", models.CharField(default="front", max_length=32)),
                ("frame_width", models.PositiveIntegerField(default=0)),
                ("frame_height", models.PositiveIntegerField(default=0)),
                ("captured_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("detections", models.JSONField(blank=True, default=list)),
                (
                    "robot",
                    models.OneToOneField(
                        on_delete=django.db.models.deletion.CASCADE,
                        related_name="person_detection_state",
                        to="monitoring.robot",
                    ),
                ),
            ],
            options={"ordering": ["robot__code"]},
        )
    ]
