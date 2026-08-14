from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0044_robot_charging_map_robot_charging_route"),
    ]

    operations = [
        migrations.AddField(
            model_name="developmenttask",
            name="model",
            field=models.CharField(default="gpt-5.6-terra", max_length=64),
        ),
    ]
