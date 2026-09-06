from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0072_system_logs_and_navigation_boundaries")]

    operations = [
        migrations.AddField(
            model_name="systemlog", name="route",
            field=models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="system_logs", to="monitoring.patrolroute"),
        ),
        migrations.AddField(
            model_name="systemlog", name="waypoint_id",
            field=models.CharField(blank=True, max_length=128),
        ),
        migrations.AddField(
            model_name="systemlog", name="round_number",
            field=models.PositiveIntegerField(blank=True, null=True),
        ),
        migrations.AddField(
            model_name="systemlog", name="nav_goal_generation",
            field=models.PositiveIntegerField(blank=True, null=True),
        ),
        migrations.AddField(
            model_name="systemlog", name="localization_generation",
            field=models.PositiveIntegerField(blank=True, null=True),
        ),
        migrations.AddField(
            model_name="systemlog", name="dedupe_key",
            field=models.CharField(blank=True, max_length=160),
        ),
        migrations.AddIndex(
            model_name="systemlog",
            index=models.Index(fields=["robot", "trace_id", "-occurred_at"], name="syslog_trace_time_idx"),
        ),
        migrations.AddIndex(
            model_name="systemlog",
            index=models.Index(fields=["robot", "module", "dedupe_key", "-occurred_at"], name="syslog_dedupe_idx"),
        ),
    ]
