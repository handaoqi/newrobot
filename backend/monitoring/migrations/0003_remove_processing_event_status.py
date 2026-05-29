from django.db import migrations, models


def move_processing_events_to_pending(apps, _schema_editor):
    InspectionEvent = apps.get_model("monitoring", "InspectionEvent")
    InspectionEvent.objects.filter(status="processing").update(status="pending")


class Migration(migrations.Migration):

    dependencies = [
        ("monitoring", "0002_inspectionevent_bbox_height_and_more"),
    ]

    operations = [
        migrations.RunPython(move_processing_events_to_pending, migrations.RunPython.noop),
        migrations.AlterField(
            model_name="inspectionevent",
            name="status",
            field=models.CharField(
                choices=[
                    ("pending", "待处理"),
                    ("resolved", "已完成"),
                    ("false_alarm", "误报"),
                ],
                default="pending",
                max_length=16,
            ),
        ),
    ]
