from django.db import migrations, models


LOW_BATTERY_ALERT_TEXT = (
    "当前电量低于百分之二十，导航任务已停止，请及时人工处理或手动回充。"
)


def migrate_low_battery_alert_skill(apps, schema_editor):
    SpeechCategory = apps.get_model("monitoring", "SpeechCategory")
    SpeechTemplate = apps.get_model("monitoring", "SpeechTemplate")
    AlertSkillBinding = apps.get_model("monitoring", "AlertSkillBinding")

    category, _ = SpeechCategory.objects.get_or_create(name="设备管控")
    template, _ = SpeechTemplate.objects.get_or_create(
        name="低电量停车告警",
        defaults={"text": LOW_BATTERY_ALERT_TEXT, "category": category},
    )
    if template.text != LOW_BATTERY_ALERT_TEXT or template.category_id is None:
        template.text = LOW_BATTERY_ALERT_TEXT
        template.category = template.category or category
        template.save(update_fields=["text", "category", "updated_at"])
    AlertSkillBinding.objects.filter(skill_key="low_battery_return_charge").update(
        template=template
    )


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0063_update_low_battery_return_speech")]

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
                    ("low_battery_return_charge", "低电量停车告警"),
                ],
                max_length=32,
                unique=True,
            ),
        ),
        migrations.RunPython(migrate_low_battery_alert_skill, migrations.RunPython.noop),
    ]
