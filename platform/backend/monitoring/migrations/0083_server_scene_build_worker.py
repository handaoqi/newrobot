from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0081_map_scene_builds")]

    operations = [
        migrations.AlterField(
            model_name="mapscenebuild",
            name="robot",
            field=models.ForeignKey(
                blank=True,
                null=True,
                on_delete=django.db.models.deletion.PROTECT,
                related_name="scene_builds",
                to="monitoring.robot",
            ),
        ),
    ]
