from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0086_bicycle_detection_test_alert_event"),
    ]

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
                    )
                ),
                fields=("task_execution", "state_version"),
                name="uniq_task_state_ver_non_obstacle",
            ),
        ),
    ]
