from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0068_alter_remotecommand_command_type"),
    ]

    operations = [
        migrations.AddField(
            model_name="patrolroute",
            name="global_controller",
            field=models.CharField(
                blank=True,
                default="theta_star",
                max_length=32,
                verbose_name="全局控制器",
            ),
        ),
    ]
