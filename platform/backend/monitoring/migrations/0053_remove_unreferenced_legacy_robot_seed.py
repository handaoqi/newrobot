from django.db import migrations


def remove_unreferenced_legacy_robot(apps, schema_editor):
    Robot = apps.get_model("monitoring", "Robot")
    candidates = Robot.objects.filter(
        code="ZSL-1A-07",
        name="南入口巡检机器人",
        location="太阳宫园区",
        area="园区主通道",
    )
    for robot in candidates.iterator():
        referenced = False
        for relation in Robot._meta.related_objects:
            related_model = relation.related_model
            lookup = {relation.field.name: robot}
            if related_model.objects.filter(**lookup).exists():
                referenced = True
                break
        if not referenced:
            robot.delete()


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0052_taskexecution_loop_context")]
    operations = [
        migrations.RunPython(
            remove_unreferenced_legacy_robot,
            migrations.RunPython.noop,
        )
    ]
