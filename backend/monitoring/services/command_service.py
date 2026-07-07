from __future__ import annotations

from datetime import timedelta

from django.conf import settings
from django.db import transaction
from django.utils import timezone

from ..models import CommandEvent, RemoteCommand, Robot, TaskExecution
from .task_service import TaskExecutionService, TaskStateError


class CommandService:
    COMMAND_TARGET_STATES = {
        "task.start": "dispatching",
        "task.pause": "pausing",
        "task.resume": "resuming",
        "task.cancel": "cancelling",
    }

    @classmethod
    @transaction.atomic
    def create(cls, execution: TaskExecution, command_type: str, operator=None) -> RemoteCommand:
        if command_type not in cls.COMMAND_TARGET_STATES:
            raise ValueError(f"unsupported command type: {command_type}")
        target = cls.COMMAND_TARGET_STATES[command_type]
        if command_type == "task.start":
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
            }
            expiry_seconds = getattr(settings, "TASK_MAX_DURATION_SECONDS", 1800)
        elif command_type == "task.resume":
            command_payload = {"resume_from_waypoint_index": execution.current_waypoint_index or 0}
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
            expires_at=timezone.now() + timedelta(seconds=expiry_seconds),
        )
        CommandEvent.objects.create(
            command=command,
            event_type="created",
            source="center",
            payload={"command_type": command_type},
        )
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
            expires_at=timezone.now() + timedelta(seconds=expiry_seconds),
        )
        CommandEvent.objects.create(
            command=command,
            event_type="created",
            source="center",
            payload={"command_type": command_type},
        )
        return command
