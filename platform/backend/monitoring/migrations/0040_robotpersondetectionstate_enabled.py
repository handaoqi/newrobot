from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0039_alertskillbinding")]

    operations = [
        migrations.AddField(
            model_name="robotpersondetectionstate",
            name="enabled",
            field=models.BooleanField(default=False),
        )
    ]
