from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0046_mcp_teleop_skills")]

    operations = [
        migrations.AddField(
            model_name="mapdata",
            name="trajectory_file",
            field=models.FileField(blank=True, null=True, upload_to="maps/traces/", verbose_name="建图轨迹"),
        ),
        migrations.AddField(
            model_name="mapdata",
            name="mapping_trace",
            field=models.FileField(blank=True, null=True, upload_to="maps/traces/", verbose_name="建图定位轨迹"),
        ),
    ]
