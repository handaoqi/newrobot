from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0019_add_map_activate_command"),
    ]

    operations = [
        migrations.AddField(
            model_name="robotstatuslatest",
            name="localization_quality",
            field=models.JSONField(blank=True, default=dict),
        ),
    ]
