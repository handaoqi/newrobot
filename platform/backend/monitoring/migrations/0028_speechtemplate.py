from django.conf import settings
from django.db import migrations, models
import django.db.models.deletion


def seed_speech_templates(apps, schema_editor):
    SpeechTemplate = apps.get_model("monitoring", "SpeechTemplate")
    defaults = [
        ("重点路段", "您好，当前区域为巡检重点路段，请勿长时间占道停留。"),
        ("驶离提醒", "您好，这里禁止自行车长时间停放，请尽快驶离指定区域，感谢配合。"),
        ("注意避让", "您好，系统检测到现场存在安全风险，请注意避让并配合引导。"),
    ]
    for name, text in defaults:
        SpeechTemplate.objects.get_or_create(name=name, defaults={"text": text})


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0027_alter_robotcommand_status"),
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
    ]

    operations = [
        migrations.CreateModel(
            name="SpeechTemplate",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("name", models.CharField(max_length=64, unique=True)),
                ("text", models.CharField(max_length=500)),
                (
                    "created_by",
                    models.ForeignKey(
                        blank=True,
                        null=True,
                        on_delete=django.db.models.deletion.SET_NULL,
                        related_name="speech_templates",
                        to=settings.AUTH_USER_MODEL,
                    ),
                ),
            ],
            options={"ordering": ["id"]},
        ),
        migrations.RunPython(seed_speech_templates, migrations.RunPython.noop),
    ]
