from django.db import migrations


class Migration(migrations.Migration):

    dependencies = [
        ("monitoring", "0005_robot_control_command"),
    ]

    operations = [
        migrations.RemoveField(
            model_name="robot",
            name="control_endpoint",
        ),
    ]
