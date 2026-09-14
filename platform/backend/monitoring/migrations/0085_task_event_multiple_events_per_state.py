from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0083_server_scene_build_worker"),
    ]

    operations = [
        migrations.RemoveConstraint(
            model_name="taskexecutionevent",
            name="uniq_task_execution_state_version",
        ),
        migrations.AddIndex(
            model_name="taskexecutionevent",
            index=models.Index(
                fields=["task_execution", "state_version"],
                name="task_event_exec_ver_idx",
            ),
        ),
        migrations.AddConstraint(
            model_name="taskexecutionevent",
            constraint=models.UniqueConstraint(
                condition=~models.Q(event_type="task.obstacle_stage"),
                fields=("task_execution", "state_version"),
                name="uniq_task_state_ver_non_obstacle",
            ),
        ),
        migrations.AlterModelOptions(
            name="taskexecutionevent",
            options={"ordering": ["state_version", "received_at", "id"]},
        ),
    ]
