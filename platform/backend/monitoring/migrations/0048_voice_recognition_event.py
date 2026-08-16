from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0047_mapdata_mapping_trace")]

    operations = [
        migrations.CreateModel(
            name="VoiceRecognitionEvent",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("transcript", models.TextField(blank=True)),
                ("command", models.TextField(blank=True)),
                ("asr_engine", models.CharField(blank=True, max_length=64)),
                ("outcome", models.CharField(choices=[("accepted", "已创建开发任务"), ("armed", "已唤醒，等待指令"), ("busy", "已有开发任务执行中"), ("ignored", "未命中唤醒词"), ("no_speech", "未识别到有效语音")], max_length=16)),
                ("robot", models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name="voice_recognition_events", to="monitoring.robot")),
                ("task", models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name="voice_recognition_events", to="monitoring.developmenttask")),
            ],
            options={"ordering": ["-created_at"]},
        ),
        migrations.AddIndex(
            model_name="voicerecognitionevent",
            index=models.Index(fields=["robot", "-created_at"], name="voice_rec_robot_time_idx"),
        ),
    ]
