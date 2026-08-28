from django.db import migrations, models


def seed_low_battery_alert_skill(apps, schema_editor):
    SpeechCategory = apps.get_model("monitoring", "SpeechCategory")
    SpeechTemplate = apps.get_model("monitoring", "SpeechTemplate")
    AlertSkillBinding = apps.get_model("monitoring", "AlertSkillBinding")

    category, _ = SpeechCategory.objects.get_or_create(name="设备管控")
    template, _ = SpeechTemplate.objects.get_or_create(
        name="低电量自动回充",
        defaults={
            "text": "当前电量低于百分之二十，导航任务已停止，正在执行自动回充，请注意避让。",
            "category": category,
        },
    )
    if template.category_id is None:
        template.category = category
        template.save(update_fields=["category", "updated_at"])
    AlertSkillBinding.objects.get_or_create(
        skill_key="low_battery_return_charge",
        defaults={"template": template, "enabled": True},
    )


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0061_validation_replay_pipeline")]

    operations = [
        migrations.AlterField(
            model_name="alertskillbinding",
            name="skill_key",
            field=models.CharField(
                choices=[
                    ("bicycle_alert", "自行车告警"),
                    ("obstacle_detected", "发现障碍"),
                    ("avoidance", "避障"),
                    ("dissuasion", "劝阻"),
                    ("low_battery_return_charge", "低电量自动回充"),
                ],
                max_length=32,
                unique=True,
            ),
        ),
        migrations.RunPython(seed_low_battery_alert_skill, migrations.RunPython.noop),
    ]
