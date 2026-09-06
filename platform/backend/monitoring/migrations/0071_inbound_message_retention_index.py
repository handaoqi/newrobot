from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0070_remove_legacy_low_battery_template")]

    operations = [
        migrations.AddIndex(
            model_name="inboundmessage",
            index=models.Index(
                fields=["process_status", "received_at"],
                name="inbound_status_time_idx",
            ),
        ),
    ]
