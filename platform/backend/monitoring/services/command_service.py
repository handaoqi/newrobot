from __future__ import annotations

import uuid
from datetime import timedelta

from django.conf import settings
from django.db import transaction
from django.utils import timezone
from rest_framework.exceptions import PermissionDenied

from ..models import CommandEvent, RemoteCommand, Robot, TaskExecution
from .task_service import TaskExecutionService, TaskStateError


class CommandService:
    LOW_BATTERY_PERCENT = 20
    COMMAND_TARGET_STATES = {
        "task.start": "dispatching",
        "task.pause": "pausing",
        "task.resume": "resuming",
        "task.resume_forward": "resuming",
        "task.cancel": "cancelling",
        "task.force_exit": "cancelling",
    }

    @classmethod
    @transaction.atomic
    def create(
        cls,
        execution: TaskExecution,
        command_type: str,
        operator=None,
        command_options: dict | None = None,
        trace_id=None,
    ) -> RemoteCommand:
        if command_type not in cls.COMMAND_TARGET_STATES:
            raise ValueError(f"unsupported command type: {command_type}")
        target = cls.COMMAND_TARGET_STATES[command_type]
        command_options = command_options or {}
        cls._ensure_battery_allows(
            execution.robot,
            command_type,
            allow_docking=bool((command_options.get("docking") or {}).get("enabled")),
        )
        if command_type == "task.start":
            record_rosbag = command_options.get("record_rosbag")
            if record_rosbag is None:
                record_rosbag = execution.task.record_rosbag
            command_payload = {
                "task_id": str(execution.task_id),
                "task_name": execution.task.name,
                "map": execution.route_snapshot["map"],
                "route_snapshot": execution.route_snapshot,
                "policy": {
                    "on_waypoint_failure": "abort",
                    "max_duration_seconds": getattr(settings, "TASK_MAX_DURATION_SECONDS", 1800),
                    "continue_on_disconnect": True,
                },
                "record_rosbag": bool(record_rosbag),
                "loop_execution": bool(command_options.get("loop_execution", False)),
                "loop_session_id": str(execution.loop_session_id) if execution.loop_session_id else None,
                "round_number": execution.round_number,
                "loop_total": max(1, int(command_options.get("loop_total", 1))),
                "smart_initialize": bool(command_options.get("smart_initialize", True)),
                "docking": dict(command_options.get("docking") or {}),
            }
            expiry_seconds = getattr(settings, "TASK_MAX_DURATION_SECONDS", 1800)
        elif command_type == "task.resume":
            command_payload = {"resume_from_waypoint_index": execution.current_waypoint_index or 0}
            expiry_seconds = getattr(settings, "COMMAND_CONTROL_EXPIRY_SECONDS", 15)
        elif command_type == "task.resume_forward":
            command_payload = {"reason": "operator_resume_forward", "resume_forward": True}
            expiry_seconds = getattr(settings, "COMMAND_CONTROL_EXPIRY_SECONDS", 15)
        else:
            command_payload = {"reason": "operator_request"}
            expiry_seconds = getattr(settings, "COMMAND_CONTROL_EXPIRY_SECONDS", 15)

        command = RemoteCommand.objects.create(
            robot=execution.robot,
            task_execution=execution,
            command_type=command_type,
            payload=command_payload,
            operator=operator,
            trace_id=uuid.UUID(str(trace_id)) if trace_id else uuid.uuid4(),
            expires_at=timezone.now() + timedelta(seconds=expiry_seconds),
        )
        CommandEvent.objects.create(
            command=command,
            event_type="created",
            source="center",
            payload={"command_type": command_type},
        )
        if command_type == "task.force_exit":
            # This is deliberately a local clean-up too.  A lost edge session
            # must not leave a unique active-task row preventing the next task.
            active_executions = list(
                TaskExecution.objects.select_for_update().filter(
                    robot=execution.robot,
                    state__in=TaskExecution.ACTIVE_STATES,
                )
            )
            for active_execution in active_executions:
                TaskExecutionService.transition(
                    active_execution,
                    "cancelled",
                    event_type="task.force_exit.requested",
                    reason_code="FORCE_EXIT",
                    reason_message="操作员强制退出并清理任务状态",
                    payload={"command_id": str(command.id), "requested_execution_id": str(execution.id)},
                )
        else:
            TaskExecutionService.transition(
                execution,
                target,
                event_type=f"{command_type}.requested",
                payload={"command_id": str(command.id)},
            )
        return command

    @staticmethod
    def mark_published(command: RemoteCommand) -> None:
        command.status = "published"
        command.published_at = timezone.now()
        command.save(update_fields=["status", "published_at", "updated_at"])
        CommandEvent.objects.create(command=command, event_type="published", source="broker")

    @staticmethod
    def mark_publish_failed(command: RemoteCommand, message: str) -> None:
        command.status = "failed"
        command.error_code = "MQTT_PUBLISH_FAILED"
        command.error_message = message
        command.finished_at = timezone.now()
        command.save(
            update_fields=["status", "error_code", "error_message", "finished_at", "updated_at"]
        )
        CommandEvent.objects.create(
            command=command,
            event_type="publish_failed",
            source="center",
            payload={"error": message},
        )

    @staticmethod
    def mark_timeout(command: RemoteCommand) -> None:
        if command.status in {"succeeded", "failed", "cancelled", "rejected", "timed_out", "expired"}:
            return
        command.status = "timed_out"
        command.error_code = "COMMAND_TIMED_OUT"
        command.finished_at = timezone.now()
        command.save(update_fields=["status", "error_code", "finished_at", "updated_at"])
        CommandEvent.objects.create(command=command, event_type="timeout", source="center")
        if command.task_execution_id and command.task_execution.state in TaskExecution.ACTIVE_STATES:
            try:
                if command.command_type == "task.pause":
                    if command.task_execution.state == "pausing":
                        TaskExecutionService.transition(
                            command.task_execution,
                            "running",
                            event_type="task.pause.timeout",
                            reason_code="COMMAND_TIMED_OUT",
                            reason_message="暂停指令超时，保留任务执行状态",
                            payload={"command_id": str(command.id)},
                        )
                    return
                if command.command_type == "task.resume":
                    if command.task_execution.state == "resuming":
                        TaskExecutionService.transition(
                            command.task_execution,
                            "paused",
                            event_type="task.resume.timeout",
                            reason_code="COMMAND_TIMED_OUT",
                            reason_message="继续指令超时，保留任务暂停状态",
                            payload={"command_id": str(command.id)},
                        )
                    return
                if command.command_type == "task.force_exit":
                    return
                TaskExecutionService.transition(
                    command.task_execution,
                    "timed_out",
                    event_type=f"{command.command_type}.timeout",
                    reason_code="COMMAND_TIMED_OUT",
                    reason_message="Edge Agent 未在超时时间内确认或完成指令",
                    payload={"command_id": str(command.id)},
                )
            except TaskStateError:
                pass

    @classmethod
    @transaction.atomic
    def create_robot_command(
        cls,
        *,
        robot: Robot,
        command_type: str,
        payload: dict,
        operator=None,
        expiry_seconds: int | None = None,
        trace_id=None,
    ) -> RemoteCommand:
        supported = {choice[0] for choice in RemoteCommand.TYPE_CHOICES}
        if command_type not in supported:
            raise ValueError(f"unsupported command type: {command_type}")
        expiry_seconds = expiry_seconds or getattr(settings, "COMMAND_CONTROL_EXPIRY_SECONDS", 15)
        command = RemoteCommand.objects.create(
            robot=robot,
            task_execution=None,
            command_type=command_type,
            payload=payload,
            operator=operator,
            trace_id=uuid.UUID(str(trace_id)) if trace_id else uuid.uuid4(),
            expires_at=timezone.now() + timedelta(seconds=expiry_seconds),
        )
        CommandEvent.objects.create(
            command=command,
            event_type="created",
            source="center",
            payload={"command_type": command_type},
        )
        return command

    @classmethod
    def ensure_task_start_allowed(cls, robot: Robot, *, allow_docking: bool = False) -> None:
        """Run admission checks that must happen before creating an execution."""
        cls._ensure_battery_allows(robot, "task.start", allow_docking=allow_docking)

    @classmethod
    def _ensure_battery_allows(cls, robot: Robot, command_type: str, *, allow_docking: bool = False) -> None:
        """Reject only a new non-docking patrol when the battery is critically low."""
        latest = getattr(robot, "latest_status", None)
        percent = None
        if latest and latest.power_available and latest.battery_percent is not None:
            percent = int(latest.battery_percent)
        elif robot.battery_level is not None:
            percent = int(robot.battery_level)
        if percent is None or percent >= cls.LOW_BATTERY_PERCENT:
            return
        if command_type == "task.start" and not allow_docking:
            raise PermissionDenied(
                f"电量不足（当前 {percent}%），低于 {cls.LOW_BATTERY_PERCENT}% 时仅可执行一键回充任务。"
            )
