from django.db import migrations


def remove_legacy_low_battery_template(apps, schema_editor):
    AlertSkillBinding = apps.get_model("monitoring", "AlertSkillBinding")
    SpeechTemplate = apps.get_model("monitoring", "SpeechTemplate")
    for template in SpeechTemplate.objects.filter(name="低电量自动回充"):
        # 0064 normally moved the system binding to the replacement template.
        # Keep a legacy template if an operator deliberately still references it.
        if AlertSkillBinding.objects.filter(template_id=template.pk).exists():
            continue
        template.delete()


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0069_patrolroute_global_controller")]

    operations = [
        migrations.RunPython(
            remove_legacy_low_battery_template,
            migrations.RunPython.noop,
        )
    ]
