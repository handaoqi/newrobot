from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0056_mapping_origin_workflow_commands")]

    operations = [
        migrations.AddField(
            model_name="mapdata",
            name="package_file",
            field=models.FileField(
                blank=True,
                null=True,
                upload_to="maps/packages/",
                verbose_name="完整地图包",
            ),
        ),
    ]
