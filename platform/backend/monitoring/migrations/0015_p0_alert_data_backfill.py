import uuid
from django.db import migrations


def forwards(apps, schema_editor):
    InspectionEvent = apps.get_model("monitoring", "InspectionEvent")
    MediaAsset = apps.get_model("monitoring", "MediaAsset")
    for asset in MediaAsset.objects.filter(media_id__isnull=True).iterator():
        asset.media_id = uuid.uuid4()
        asset.save(update_fields=["media_id"])
    assets_by_url = {
        asset.url: asset.id
        for asset in MediaAsset.objects.exclude(url="").only("id", "url")
    }
    for event in InspectionEvent.objects.filter(event_id__isnull=True).iterator():
        event.event_id = uuid.uuid4()
        if event.snapshot_url in assets_by_url:
            event.snapshot_asset_id = assets_by_url[event.snapshot_url]
        event.save(update_fields=["event_id", "snapshot_asset"])


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0014_p0_alert_media_context")]
    operations = [migrations.RunPython(forwards, migrations.RunPython.noop)]
