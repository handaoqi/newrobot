from django.db import migrations, models
import django.db.models.deletion


DEFAULT_SKILLS = (
    ("bicycle_alert", "驶离提醒", "您好，这里禁止自行车长时间停放，请尽快驶离指定区域，感谢配合。", "秩序劝导"),
    ("obstacle_detected", "发现障碍物", "前方发现障碍物，请注意避让。", "巡检智能播报"),
    ("avoidance", "后退尝试避障", "前方通行受阻，正在尝试避障。", "巡检智能播报"),
    ("dissuasion", "劝阻离开线路", "请勿阻挡机器狗巡检路线，请尽快离开。", "巡检智能播报"),
)


def seed_alert_skills(apps, schema_editor):
    AlertSkillBinding = apps.get_model("monitoring", "AlertSkillBinding")
    SpeechCategory = apps.get_model("monitoring", "SpeechCategory")
    SpeechTemplate = apps.get_model("monitoring", "SpeechTemplate")
    for skill_key, template_name, text, category_name in DEFAULT_SKILLS:
        category, _ = SpeechCategory.objects.get_or_create(name=category_name)
        template, _ = SpeechTemplate.objects.get_or_create(
            name=template_name,
            defaults={"text": text, "category": category},
        )
        AlertSkillBinding.objects.get_or_create(
            skill_key=skill_key,
            defaults={"template": template, "enabled": True},
        )


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0038_alter_remotecommand_command_type_and_more")]

    operations = [
        migrations.CreateModel(
            name="AlertSkillBinding",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                (
                    "skill_key",
                    models.CharField(
                        choices=[
                            ("bicycle_alert", "自行车告警"),
                            ("obstacle_detected", "发现障碍"),
                            ("avoidance", "避障"),
                            ("dissuasion", "劝阻"),
                        ],
                        max_length=32,
                        unique=True,
                    ),
                ),
                ("enabled", models.BooleanField(default=True)),
                (
                    "template",
                    models.ForeignKey(
                        blank=True,
                        null=True,
                        on_delete=django.db.models.deletion.SET_NULL,
                        related_name="alert_skill_bindings",
                        to="monitoring.speechtemplate",
                    ),
                ),
            ],
            options={"ordering": ["id"]},
        ),
        migrations.RunPython(seed_alert_skills, migrations.RunPython.noop),
    ]
