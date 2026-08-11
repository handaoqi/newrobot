from django.db import migrations, models


def normalize_event_review_state(apps, _schema_editor):
    InspectionEvent = apps.get_model("monitoring", "InspectionEvent")
    InspectionEvent.objects.filter(status="false_alarm").update(
        status="resolved",
        review_result="false_alarm",
    )


class Migration(migrations.Migration):

    dependencies = [
        ("monitoring", "0003_remove_processing_event_status"),
    ]

    operations = [
        migrations.AddField(
            model_name="inspectionevent",
            name="review_result",
            field=models.CharField(
                choices=[
                    ("confirmed", "确认违规"),
                    ("suspected", "怀疑"),
                    ("false_alarm", "误报"),
                ],
                default="confirmed",
                max_length=16,
            ),
        ),
        migrations.RunPython(normalize_event_review_state, migrations.RunPython.noop),
        migrations.AlterField(
            model_name="inspectionevent",
            name="status",
            field=models.CharField(
                choices=[
                    ("pending", "待处理"),
                    ("resolved", "已处理"),
                ],
                default="pending",
                max_length=16,
            ),
        ),
    ]
