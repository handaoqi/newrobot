from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0034_remote_development"),
    ]

    operations = [
        migrations.AddField(
            model_name="developmenttask",
            name="codex_thread_id",
            field=models.CharField(blank=True, max_length=128),
        ),
    ]
