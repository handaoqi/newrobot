import uuid
from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0015_p0_alert_data_backfill")]
    operations = [
        migrations.AlterField("mediaasset", "media_id", models.UUIDField(db_index=True, default=uuid.uuid4, unique=True)),
        migrations.AlterField("inspectionevent", "event_id", models.UUIDField(db_index=True, default=uuid.uuid4, unique=True)),
        migrations.AddConstraint("robottelemetry", models.UniqueConstraint(fields=("robot", "session_id", "sequence_id"), name="uniq_robot_session_telemetry_sequence")),
        migrations.AddIndex("commandevent", models.Index(fields=["command", "event_at"], name="command_event_time_idx")),
        migrations.AddIndex("taskexecution", models.Index(fields=["robot", "state"], name="task_exec_robot_state_idx")),
        migrations.AddIndex("taskexecution", models.Index(fields=["-created_at"], name="task_exec_recent_idx")),
        migrations.AddConstraint("taskexecution", models.CheckConstraint(condition=models.Q(("state_version__gte", 0)), name="task_exec_state_version_gte_0")),
        migrations.AddConstraint("taskexecution", models.UniqueConstraint(condition=models.Q(("state__in", ["created", "dispatching", "accepted", "running", "pausing", "paused", "resuming", "cancelling", "interrupted"])), fields=("robot",), name="uniq_active_task_execution_per_robot")),
        migrations.AddIndex("remotecommand", models.Index(fields=["robot", "-issued_at"], name="remote_cmd_robot_recent_idx")),
        migrations.AddIndex("remotecommand", models.Index(fields=["task_execution", "-issued_at"], name="remote_cmd_exec_recent_idx")),
        migrations.AddConstraint("remotecommand", models.CheckConstraint(condition=models.Q(("expires_at__gt", models.F("issued_at"))), name="remote_cmd_expiry_after_issue")),
        migrations.AddConstraint("taskexecutionevent", models.UniqueConstraint(fields=("task_execution", "state_version"), name="uniq_task_execution_state_version")),
        migrations.AddConstraint("trajectorybatchreceipt", models.CheckConstraint(condition=models.Q(("last_seq__gte", models.F("first_seq"))), name="trajectory_batch_valid_range")),
        migrations.AddIndex("trajectorypoint", models.Index(fields=["task_execution", "sampled_at"], name="trajectory_exec_time_idx")),
        migrations.AddIndex("trajectorypoint", models.Index(fields=["robot", "sampled_at"], name="trajectory_robot_time_idx")),
        migrations.AddConstraint("trajectorypoint", models.UniqueConstraint(fields=("robot", "task_execution", "seq"), name="uniq_robot_execution_trajectory_seq")),
        migrations.AddConstraint("trajectorypoint", models.CheckConstraint(condition=models.Q(("seq__gte", 0)), name="trajectory_seq_gte_0")),
    ]
