import django.db.models.deletion
import django.utils.timezone
from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0012_p0_task_data_backfill")]
    operations = [
        migrations.AddField("robottelemetry", "session_id", models.CharField(default="legacy", max_length=64)),
        migrations.AddField("robottelemetry", "message_id", models.UUIDField(blank=True, db_index=True, null=True)),
        migrations.AddField("robottelemetry", "task_execution", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="telemetry_records", to="monitoring.taskexecution")),
        migrations.AddField("robottelemetry", "frame_id", models.CharField(blank=True, max_length=32)),
        migrations.AddField("robottelemetry", "map_id", models.CharField(blank=True, max_length=128)),
        migrations.AddField("robottelemetry", "map_version", models.CharField(blank=True, max_length=64)),
        migrations.AddField("robottelemetry", "x", models.DecimalField(blank=True, decimal_places=4, max_digits=12, null=True)),
        migrations.AddField("robottelemetry", "y", models.DecimalField(blank=True, decimal_places=4, max_digits=12, null=True)),
        migrations.AddField("robottelemetry", "z", models.DecimalField(blank=True, decimal_places=4, max_digits=12, null=True)),
        migrations.AddField("robottelemetry", "yaw", models.DecimalField(blank=True, decimal_places=5, max_digits=10, null=True)),
        migrations.AddField("robottelemetry", "localization_status", models.CharField(blank=True, max_length=24)),
        migrations.AlterField("robottelemetry", "sequence_id", models.CharField(max_length=64)),
        migrations.CreateModel(
            name="TrajectoryPoint",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("seq", models.BigIntegerField()),
                ("sampled_at", models.DateTimeField()),
                ("received_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("frame_id", models.CharField(default="map", max_length=32)),
                ("map_id", models.CharField(blank=True, max_length=128)),
                ("map_version", models.CharField(blank=True, max_length=64)),
                ("x", models.DecimalField(decimal_places=4, max_digits=12)),
                ("y", models.DecimalField(decimal_places=4, max_digits=12)),
                ("yaw", models.DecimalField(decimal_places=5, max_digits=10)),
                ("speed_mps", models.DecimalField(blank=True, decimal_places=5, max_digits=10, null=True)),
                ("localization_status", models.CharField(max_length=24)),
                ("batch_id", models.UUIDField()),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="trajectory_points", to="monitoring.robot")),
                ("task_execution", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="trajectory_points", to="monitoring.taskexecution")),
            ],
            options={"ordering": ["seq"]},
        ),
        migrations.CreateModel(
            name="TrajectoryBatchReceipt",
            fields=[
                ("batch_id", models.UUIDField(primary_key=True, serialize=False)),
                ("first_seq", models.BigIntegerField()),
                ("last_seq", models.BigIntegerField()),
                ("point_count", models.PositiveIntegerField()),
                ("received_at", models.DateTimeField(default=django.utils.timezone.now)),
                ("duplicate", models.BooleanField(default=False)),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="trajectory_receipts", to="monitoring.robot")),
                ("task_execution", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="trajectory_receipts", to="monitoring.taskexecution")),
            ],
        ),
    ]
