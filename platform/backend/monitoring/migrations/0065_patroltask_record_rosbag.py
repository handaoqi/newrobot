from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0064_low_battery_alert_only"),
    ]

    operations = [
        migrations.AddField(
            model_name="patroltask",
            name="record_rosbag",
            field=models.BooleanField(default=False),
        ),
    ]
