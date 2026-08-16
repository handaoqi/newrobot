from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0022_merge_20260714_1828")]

    operations = [
        migrations.CreateModel(
            name="MapSet",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("name", models.CharField(max_length=128)),
                ("version", models.CharField(blank=True, max_length=64)),
                ("manifest", models.JSONField(blank=True, default=dict)),
                ("active", models.BooleanField(default=False)),
                (
                    "robot",
                    models.ForeignKey(
                        blank=True,
                        null=True,
                        on_delete=django.db.models.deletion.SET_NULL,
                        related_name="map_sets",
                        to="monitoring.robot",
                    ),
                ),
            ],
            options={"ordering": ["-created_at"]},
        ),
        migrations.CreateModel(
            name="MapSetMember",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("sequence", models.PositiveIntegerField()),
                ("submap_id", models.CharField(max_length=64)),
                ("metadata", models.JSONField(blank=True, default=dict)),
                (
                    "map_data",
                    models.OneToOneField(
                        on_delete=django.db.models.deletion.CASCADE,
                        related_name="map_set_member",
                        to="monitoring.mapdata",
                    ),
                ),
                (
                    "map_set",
                    models.ForeignKey(
                        on_delete=django.db.models.deletion.CASCADE,
                        related_name="members",
                        to="monitoring.mapset",
                    ),
                ),
            ],
            options={"ordering": ["sequence"]},
        ),
        migrations.AddField(
            model_name="patrolroute",
            name="map_set",
            field=models.ForeignKey(
                blank=True,
                null=True,
                on_delete=django.db.models.deletion.SET_NULL,
                related_name="routes",
                to="monitoring.mapset",
            ),
        ),
        migrations.AddConstraint(
            model_name="mapsetmember",
            constraint=models.UniqueConstraint(fields=("map_set", "sequence"), name="unique_map_set_sequence"),
        ),
    ]
