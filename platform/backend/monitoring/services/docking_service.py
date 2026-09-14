from __future__ import annotations

import uuid
from dataclasses import dataclass
from datetime import timedelta

from django.db import transaction
from django.utils import timezone

from ..models import MapData, PatrolRoute, PatrolTask, RemoteCommand, Robot, TaskExecution
from .command_service import CommandService
from .task_service import TaskExecutionService, TaskStateError


@dataclass
class DockingDispatchResult:
    execution: TaskExecution
    command: RemoteCommand
    created: bool


class DockingDispatchError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@transaction.atomic
def dispatch_docking_task(
    *,
    robot: Robot,
    operator=None,
    map_data: MapData | None = None,
    route: PatrolRoute | None = None,
    low_battery_episode_id: str = "",
    persist_configuration: bool = False,
) -> DockingDispatchResult:
    robot = Robot.objects.select_for_update().get(pk=robot.pk)
    map_data = map_data or robot.charging_map
    route = route or robot.charging_route
    if not map_data or not route:
        raise DockingDispatchError("DOCK_ROUTE_MISSING", "请先选择充电地图和两点回充路线")
    if route.robot_id != robot.id or route.map_data_id != map_data.id:
        raise DockingDispatchError("DOCK_ROUTE_MISMATCH", "回充路线与机器狗或充电地图不匹配")
    if len(route.waypoints or []) != 2:
        raise DockingDispatchError("DOCK_ROUTE_INVALID", "回充路线必须固定为两个点")

    if low_battery_episode_id:
        existing = (
            TaskExecution.objects.filter(robot=robot, loop_session_id=uuid.UUID(low_battery_episode_id))
            .order_by("created_at")
            .first()
        )
        if existing:
            command = existing.commands.filter(command_type="task.start").order_by("issued_at").first()
            if command:
                return DockingDispatchResult(existing, command, False)

    active = TaskExecution.objects.filter(robot=robot, state__in=TaskExecution.ACTIVE_STATES).first()
    if active:
        docking_command = active.commands.filter(
            command_type="task.start", payload__docking__enabled=True
        ).order_by("issued_at").first()
        if docking_command:
            return DockingDispatchResult(active, docking_command, False)
        raise DockingDispatchError("ROBOT_BUSY", "机器人仍有活动导航任务，未重复创建回充任务")
    if robot.effective_connection_status() != "online":
        raise DockingDispatchError("EDGE_OFFLINE", "机器狗 Edge Agent 当前离线")
    if robot.localization_status != "normal" or not robot.nav_ready:
        raise DockingDispatchError("DOCK_NAV_NOT_READY", "定位或导航栈未就绪，不能开始回充")

    if persist_configuration:
        robot.charging_map = map_data
        robot.charging_route = route
        robot.save(update_fields=["charging_map", "charging_route", "updated_at"])

    now = timezone.now()
    task_name = f"一键回充 - {route.name}"
    task, _ = PatrolTask.objects.get_or_create(
        robot=robot,
        route=route,
        name=task_name,
        defaults={
            "route_name": route.name,
            "scheduled_start": now,
            "scheduled_end": now + timedelta(hours=8),
            "enabled": True,
            "description": "机器人自动回充专用两点路线",
            "created_by": operator,
        },
    )
    task.route_name = route.name
    task.scheduled_start = now
    task.scheduled_end = now + timedelta(hours=8)
    task.enabled = True
    task.save(
        update_fields=["route_name", "scheduled_start", "scheduled_end", "enabled", "updated_at"]
    )
    try:
        execution = TaskExecutionService.create_execution(
            task,
            operator,
            loop_session_id=(uuid.UUID(low_battery_episode_id) if low_battery_episode_id else None),
            execution_source="auto_docking",
        )
        command = CommandService.create(
            execution,
            "task.start",
            operator,
            command_options={
                "docking": {
                    "enabled": True,
                    "final_waypoint_index": 1,
                    "charge_retries": 3,
                    "undock_seconds": 3,
                    "low_battery_episode_id": low_battery_episode_id or None,
                }
            },
        )
    except TaskStateError as exc:
        raise DockingDispatchError(str(exc), str(exc)) from exc
    return DockingDispatchResult(execution, command, True)
