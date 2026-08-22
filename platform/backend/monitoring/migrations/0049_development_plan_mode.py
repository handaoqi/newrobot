from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0048_voice_recognition_event"),
    ]

    operations = [
        migrations.AddField(
            model_name="developmenttask",
            name="execution_mode",
            field=models.CharField(
                choices=[("plan", "规划对话"), ("execute", "执行任务")],
                default="execute",
                max_length=16,
            ),
        ),
        migrations.CreateModel(
            name="DevelopmentConversationState",
            fields=[
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("mode", models.CharField(choices=[("execute", "执行模式"), ("plan", "规划对话模式")], default="execute", max_length=16)),
                ("workspace", models.CharField(default="robot-main", max_length=64)),
                ("model", models.CharField(default="gpt-5.6-terra", max_length=64)),
                ("robot", models.OneToOneField(on_delete=django.db.models.deletion.CASCADE, primary_key=True, related_name="development_conversation_state", serialize=False, to="monitoring.robot")),
            ],
        ),
    ]
