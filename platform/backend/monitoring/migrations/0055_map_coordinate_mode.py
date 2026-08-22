from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0054_seed_eleven_speech_templates")]

    operations = [
        migrations.AddField(
            model_name="mapdata",
            name="coordinate_mode",
            field=models.CharField(blank=True, max_length=32, verbose_name="坐标模式"),
        ),
        migrations.AddField(
            model_name="mapdata",
            name="scene_scope",
            field=models.CharField(blank=True, max_length=32, verbose_name="场景范围"),
        ),
        migrations.AddField(
            model_name="mapdata",
            name="localization_mode",
            field=models.CharField(blank=True, max_length=32, verbose_name="定位模式"),
        ),
        migrations.AddField(
            model_name="mapdata",
            name="origin_status",
            field=models.CharField(blank=True, max_length=32, verbose_name="原点状态"),
        ),
        migrations.AddField(
            model_name="mapdata",
            name="map_completeness",
            field=models.CharField(blank=True, max_length=32, verbose_name="地图完整度"),
        ),
        migrations.AddField(
            model_name="patrolroute",
            name="scene_scope",
            field=models.CharField(blank=True, default="indoor", max_length=32, verbose_name="场景范围"),
        ),
    ]
