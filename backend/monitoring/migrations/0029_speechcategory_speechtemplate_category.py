from django.db import migrations, models
import django.db.models.deletion


def seed_speech_categories(apps, schema_editor):
    SpeechCategory = apps.get_model("monitoring", "SpeechCategory")
    SpeechTemplate = apps.get_model("monitoring", "SpeechTemplate")
    assignments = {
        "重点路段": "巡检提醒",
        "驶离提醒": "秩序劝导",
        "注意避让": "安全提醒",
    }
    for template_name, category_name in assignments.items():
        category, _ = SpeechCategory.objects.get_or_create(name=category_name)
        SpeechTemplate.objects.filter(name=template_name).update(category=category)


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0028_speechtemplate")]

    operations = [
        migrations.CreateModel(
            name="SpeechCategory",
            fields=[
                ("id", models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name="ID")),
                ("created_at", models.DateTimeField(auto_now_add=True)),
                ("updated_at", models.DateTimeField(auto_now=True)),
                ("name", models.CharField(max_length=64, unique=True)),
            ],
            options={"ordering": ["id"]},
        ),
        migrations.AddField(
            model_name="speechtemplate",
            name="category",
            field=models.ForeignKey(
                blank=True,
                null=True,
                on_delete=django.db.models.deletion.SET_NULL,
                related_name="templates",
                to="monitoring.speechcategory",
            ),
        ),
        migrations.RunPython(seed_speech_categories, migrations.RunPython.noop),
    ]
