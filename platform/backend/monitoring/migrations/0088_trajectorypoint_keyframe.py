from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0087_allow_arrival_degraded_task_events")]

    operations = [
        migrations.AddField(
            model_name="trajectorypoint",
            name="keyframe",
            field=models.JSONField(blank=True, default=dict),
        ),
    ]
