from django.db import migrations, models


def normalize_duplicate_route_names(apps, schema_editor):
    PatrolRoute = apps.get_model("monitoring", "PatrolRoute")
    seen = set()
    for route in PatrolRoute.objects.order_by("created_at", "id"):
        name = (route.name or "").strip()
        candidate = name or f"路线-{route.id}"
        if candidate in seen:
            base = candidate
            suffix = f"-副本-{route.id}"
            candidate = f"{base[:128 - len(suffix)]}{suffix}"
            counter = 2
            while candidate in seen:
                extra = f"-{counter}"
                candidate = f"{base[:128 - len(suffix) - len(extra)]}{suffix}{extra}"
                counter += 1
        if route.name != candidate:
            route.name = candidate
            route.save(update_fields=["name", "updated_at"])
        seen.add(candidate)


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0073_system_log_trace_context")]

    operations = [
        migrations.RunPython(normalize_duplicate_route_names, migrations.RunPython.noop),
        migrations.AlterField(
            model_name="patrolroute",
            name="name",
            field=models.CharField(max_length=128, unique=True, verbose_name="路线名称"),
        ),
    ]
