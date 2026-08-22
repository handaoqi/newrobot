from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0051_teleop_skill_list"),
    ]

    operations = [
        migrations.AddField(
            model_name="taskexecution",
            name="loop_session_id",
            field=models.UUIDField(blank=True, db_index=True, null=True),
        ),
        migrations.AddField(
            model_name="taskexecution",
            name="round_number",
            field=models.PositiveIntegerField(default=1),
        ),
    ]
