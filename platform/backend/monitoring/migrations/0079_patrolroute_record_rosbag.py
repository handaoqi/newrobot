from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0078_alter_robot_control_mode")]

    operations = [
        migrations.AddField(
            model_name="patrolroute",
            name="record_rosbag",
            field=models.BooleanField(default=False, verbose_name="录制导航调试包"),
        ),
    ]
