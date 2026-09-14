from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0084_bicycle_detection_test_queue"),
        ("monitoring", "0085_task_event_multiple_events_per_state"),
    ]

    operations = [
        migrations.AddField(
            model_name="bicycledetectiontestimage",
            name="alert_event",
            field=models.OneToOneField(
                blank=True,
                null=True,
                on_delete=django.db.models.deletion.SET_NULL,
                related_name="photo_detection_test_image",
                to="monitoring.inspectionevent",
            ),
        ),
    ]
