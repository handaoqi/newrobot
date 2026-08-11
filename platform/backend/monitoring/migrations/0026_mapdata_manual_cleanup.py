from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0025_merge_map_sets_audio_choices")]

    operations = [
        migrations.AddField(
            model_name="mapdata",
            name="edit_metadata",
            field=models.JSONField(blank=True, default=dict, verbose_name="地图编辑记录"),
        ),
        migrations.AddField(
            model_name="mapdata",
            name="parent_map",
            field=models.ForeignKey(
                blank=True,
                null=True,
                on_delete=django.db.models.deletion.SET_NULL,
                related_name="derived_maps",
                to="monitoring.mapdata",
                verbose_name="来源地图",
            ),
        ),
    ]
