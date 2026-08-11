import uuid
from django.db import migrations
from django.utils import timezone


def forwards(apps, schema_editor):
    PatrolTask = apps.get_model("monitoring", "PatrolTask")
    PatrolRoute = apps.get_model("monitoring", "PatrolRoute")
    TaskExecution = apps.get_model("monitoring", "TaskExecution")
    TaskExecutionEvent = apps.get_model("monitoring", "TaskExecutionEvent")

    migrated_active_robots = set()
    tasks = PatrolTask.objects.all().order_by("robot_id", "-scheduled_start", "-id")
    for task in tasks.iterator():
        routes = PatrolRoute.objects.filter(robot_id=task.robot_id, name=task.route_name)
        route = routes.first() if routes.count() == 1 else None
        if route:
            task.route_id = route.id
            task.save(update_fields=["route"])
        if task.status not in {"running", "paused", "completed"}:
            continue
        state = "completed" if task.status == "completed" else "interrupted"
        if state == "interrupted":
            if task.robot_id in migrated_active_robots:
                state = "failed"
            else:
                migrated_active_robots.add(task.robot_id)
        waypoints = []
        if route:
            names = route.waypoint_names or []
            for index, point in enumerate(route.waypoints or []):
                if isinstance(point, dict):
                    x, y, yaw = point.get("x"), point.get("y"), point.get("yaw", 0)
                elif isinstance(point, (list, tuple)) and len(point) >= 2:
                    x, y, yaw = point[0], point[1], point[2] if len(point) > 2 else 0
                else:
                    continue
                waypoints.append(
                    {
                        "waypoint_id": f"wp-{index + 1}",
                        "sequence": index,
                        "name": names[index] if index < len(names) else f"航点 {index + 1}",
                        "x": x,
                        "y": y,
                        "yaw": yaw,
                        "dwell_seconds": 0,
                        "actions": [],
                    }
                )
        snapshot = {
            "route_id": str(route.id) if route else None,
            "route_name": task.route_name,
            "frame_id": "map",
            "map": {
                "map_id": str(route.map_data_id) if route else "",
                "map_version": f"legacy-mapdata-{route.map_data_id}" if route else "",
                "sha256": None,
            },
            "waypoints": waypoints,
        }
        execution = TaskExecution.objects.create(
            id=uuid.uuid4(),
            task_id=task.id,
            robot_id=task.robot_id,
            route_id=route.id if route else None,
            map_data_id=route.map_data_id if route else None,
            route_snapshot=snapshot,
            state=state,
            state_version=1,
            completed_waypoints=len(waypoints) if state == "completed" else 0,
            total_waypoints=len(waypoints),
            started_at=task.scheduled_start,
            finished_at=task.scheduled_end if state == "completed" else None,
            failure_code=(
                "LEGACY_ACTIVE_TASK_SUPERSEDED"
                if state == "failed"
                else ("" if route else "LEGACY_ROUTE_UNRESOLVED")
            ),
        )
        TaskExecutionEvent.objects.create(
            task_execution_id=execution.id,
            state=state,
            state_version=1,
            event_type="legacy.migrated",
            occurred_at=timezone.now(),
            payload={"legacy_task_status": task.status},
        )


class Migration(migrations.Migration):
    dependencies = [("monitoring", "0011_p0_task_models")]
    operations = [migrations.RunPython(forwards, migrations.RunPython.noop)]
