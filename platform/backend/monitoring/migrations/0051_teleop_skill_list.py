from django.db import migrations, models
from importlib import import_module


REMOTE_COMMAND_CHOICES = import_module(
    "monitoring.migrations.0050_person_follow_remote_commands"
).REMOTE_COMMAND_CHOICES
REMOTE_COMMAND_CHOICES_WITH_LIST = []
for choice in REMOTE_COMMAND_CHOICES:
    if choice[0] == "teleop.skill_status":
        REMOTE_COMMAND_CHOICES_WITH_LIST.append(("teleop.skill_list", "列出遥控技能"))
    REMOTE_COMMAND_CHOICES_WITH_LIST.append(choice)


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0050_person_follow_remote_commands")]

    operations = [
        migrations.AlterField(
            model_name="remotecommand",
            name="command_type",
            field=models.CharField(
                choices=REMOTE_COMMAND_CHOICES_WITH_LIST,
                max_length=32,
            ),
        ),
    ]
