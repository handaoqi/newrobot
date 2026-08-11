from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0030_recordedaudio")]

    operations = [
        migrations.AddField(
            model_name="recordedaudio",
            name="transcript",
            field=models.TextField(blank=True),
        ),
        migrations.AddField(
            model_name="recordedaudio",
            name="asr_status",
            field=models.CharField(
                choices=[("pending", "识别中"), ("completed", "识别完成"), ("failed", "识别失败")],
                default="pending",
                max_length=16,
            ),
        ),
        migrations.AddField(
            model_name="recordedaudio",
            name="asr_error",
            field=models.CharField(blank=True, max_length=255),
        ),
    ]
