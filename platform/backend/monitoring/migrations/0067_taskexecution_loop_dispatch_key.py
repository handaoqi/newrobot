from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0066_add_nav_single_goal_command")]

    operations = [
        migrations.AddField(
            model_name="taskexecution",
            name="loop_dispatch_key",
            field=models.CharField(blank=True, editable=False, max_length=80, null=True, unique=True),
        ),
    ]
