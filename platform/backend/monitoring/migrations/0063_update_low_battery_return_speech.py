from django.db import migrations


def update_low_battery_speech(apps, schema_editor):
    SpeechTemplate = apps.get_model("monitoring", "SpeechTemplate")
    SpeechTemplate.objects.filter(name="低电量自动回充").update(
        text="当前电量低于百分之二十，导航任务已停止，正在执行自动回充，请注意避让。"
    )


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0062_low_battery_return_charge_alert_skill")]
    operations = [migrations.RunPython(update_low_battery_speech, migrations.RunPython.noop)]
