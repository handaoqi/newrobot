import django.db.models.deletion
import django.utils.timezone
from django.conf import settings
from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0013_p0_trajectory_models"),
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
    ]
    operations = [
        migrations.AddField("mediaasset", "media_id", models.UUIDField(blank=True, db_index=True, null=True)),
        migrations.AddField("mediaasset", "event_id", models.UUIDField(blank=True, db_index=True, null=True)),
        migrations.AddField("mediaasset", "task_execution", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="media_assets", to="monitoring.taskexecution")),
        migrations.AddField("mediaasset", "content_type", models.CharField(blank=True, max_length=128)),
        migrations.AddField("mediaasset", "uploaded_at", models.DateTimeField(default=django.utils.timezone.now)),
        migrations.AddField("inspectionevent", "event_id", models.UUIDField(blank=True, db_index=True, null=True)),
        migrations.AddField("inspectionevent", "task_execution", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="inspection_events", to="monitoring.taskexecution")),
        migrations.AddField("inspectionevent", "map_data", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="inspection_events", to="monitoring.mapdata")),
        migrations.AddField("inspectionevent", "map_id", models.CharField(blank=True, max_length=128)),
        migrations.AddField("inspectionevent", "map_version", models.CharField(blank=True, max_length=64)),
        migrations.AddField("inspectionevent", "frame_id", models.CharField(default="map", max_length=32)),
        migrations.AddField("inspectionevent", "position_x", models.DecimalField(blank=True, decimal_places=4, max_digits=12, null=True)),
        migrations.AddField("inspectionevent", "position_y", models.DecimalField(blank=True, decimal_places=4, max_digits=12, null=True)),
        migrations.AddField("inspectionevent", "position_yaw", models.DecimalField(blank=True, decimal_places=5, max_digits=10, null=True)),
        migrations.AddField("inspectionevent", "source_component", models.CharField(blank=True, max_length=64)),
        migrations.AddField("inspectionevent", "source_code", models.CharField(blank=True, max_length=64)),
        migrations.AddField("inspectionevent", "model_version", models.CharField(blank=True, max_length=64)),
        migrations.AddField("inspectionevent", "snapshot_asset", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="snapshot_events", to="monitoring.mediaasset")),
        migrations.AddField("inspectionevent", "clip_asset", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="clip_events", to="monitoring.mediaasset")),
        migrations.AddField("inspectionevent", "handled_by", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="handled_inspection_events", to=settings.AUTH_USER_MODEL)),
        migrations.AddField("inspectionevent", "handled_at", models.DateTimeField(blank=True, null=True)),
    ]
