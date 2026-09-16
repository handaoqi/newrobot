from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0088_trajectorypoint_keyframe")]

    operations = [
        migrations.RemoveConstraint(
            model_name="taskexecutionevent",
            name="uniq_task_state_ver_non_obstacle",
        ),
        migrations.AddConstraint(
            model_name="taskexecutionevent",
            constraint=models.UniqueConstraint(
                condition=~models.Q(
                    event_type__in=(
                        "task.obstacle_stage",
                        "task.arrival_degraded_accepted",
                        "task.navigation_stage",
                    )
                ),
                fields=("task_execution", "state_version"),
                name="uniq_task_state_ver_non_obstacle",
            ),
        ),
    ]
