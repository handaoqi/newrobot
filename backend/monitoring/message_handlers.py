from __future__ import annotations

import uuid
from collections.abc import Callable

from django.db import IntegrityError, transaction
from django.utils import timezone

from .models import (
    CommandEvent,
    InboundMessage,
    RemoteCommand,
    Robot,
    RobotSession,
    TaskExecution,
    TrajectoryBatchReceipt,
    TrajectoryPoint,
)
from .protocol import MessageEnvelope, ProtocolError, parse_message
from .realtime_gateway import realtime_publisher
from .serializers import EventSerializer, TaskExecutionSerializer
from .services.alert_service import AlertService
from .services.task_service import TaskExecutionService, TaskStateError
from .services.telemetry_service import TelemetryService


ResponsePublisher = Callable[[str, dict, int, bool], None]


def _event_time(payload: dict, *keys: str):
    from .protocol import normalize_timestamp

    for key in keys:
        if payload.get(key):
            return normalize_timestamp(payload[key])
    return timezone.now()


def _record_inbound(topic: str, envelope: MessageEnvelope, robot: Robot) -> tuple[InboundMessage, bool]:
    try:
        message, created = InboundMessage.objects.get_or_create(
            message_id=envelope.message_id,
            defaults={
                "robot": robot,
                "session_id": uuid.UUID(envelope.session_id),
                "message_type": envelope.message_type,
                "sequence": envelope.sequence,
                "topic": topic,
                "raw_payload": envelope.raw,
            },
        )
    except ValueError as exc:
        raise ProtocolError("INVALID_MESSAGE", "session_id must be UUID for Edge uplink") from exc
    return message, created


def handle_mqtt_message(
    topic: str,
    raw_payload: bytes | str | dict,
    publish_response: ResponsePublisher | None = None,
) -> dict:
    envelope = parse_message(raw_payload)
    robot = Robot.objects.filter(code=envelope.robot_id).first()
    if robot is None:
        raise ProtocolError("UNKNOWN_ROBOT", f"unknown robot: {envelope.robot_id}")

    with transaction.atomic():
        inbound, created = _record_inbound(topic, envelope, robot)
        if not created:
            return {"duplicate": True, "message_id": str(envelope.message_id)}
        try:
            result = _dispatch(envelope, robot, publish_response)
            inbound.process_status = "processed"
            inbound.processed_at = timezone.now()
            inbound.save(update_fields=["process_status", "processed_at"])
            return result
        except Exception as exc:
            inbound.process_status = "failed"
            inbound.error_message = str(exc)
            inbound.processed_at = timezone.now()
            inbound.save(update_fields=["process_status", "error_message", "processed_at"])
            raise


def _dispatch(
    envelope: MessageEnvelope,
    robot: Robot,
    publish_response: ResponsePublisher | None,
) -> dict:
    message_type = envelope.message_type
    payload = envelope.payload
    if message_type.startswith("presence."):
        return _handle_presence(envelope, robot)
    if message_type == "telemetry.status":
        status, changed = TelemetryService.apply_status(robot, payload)
        realtime_publisher.publish_robot_status(robot.id, payload)
        return {"changed": changed, "state_version": status.state_version}
    if message_type == "telemetry.pose":
        robot.last_seen_at = timezone.now()
        robot.save(update_fields=["last_seen_at", "updated_at"])
        realtime_publisher.publish_robot_status(robot.id, payload)
        return {"accepted": True}
    if message_type == "command.ack":
        return _handle_command_ack(envelope, robot)
    if message_type == "command.result":
        return _handle_command_result(envelope, robot)
    if message_type.startswith("task."):
        return _handle_task_event(envelope, robot)
    if message_type == "trajectory.batch":
        return _handle_trajectory(envelope, robot, publish_response)
    if message_type == "alert.event":
        event, created = AlertService.ingest_edge_alert(robot, payload)
        if created:
            realtime_publisher.publish_alert(
                {
                    "event": EventSerializer(event).data,
                    "robot": {"id": robot.id, "code": robot.code, "name": robot.name},
                }
            )
        return {"created": created, "event_id": str(event.event_id)}
    if message_type == "sync.request":
        return _handle_sync(envelope, robot, publish_response)
    return {"ignored": True}


def _handle_presence(envelope: MessageEnvelope, robot: Robot) -> dict:
    payload = envelope.payload
    now = timezone.now()
    session_uuid = uuid.UUID(envelope.session_id)
    if envelope.message_type == "presence.online":
        session, _ = RobotSession.objects.update_or_create(
            session_id=session_uuid,
            defaults={
                "robot": robot,
                "transport": "mqtt",
                "connected_at": envelope.sent_at,
                "last_heartbeat_at": now,
                "disconnected_at": None,
                "disconnect_reason": "",
                "agent_version": payload.get("agent_version", ""),
                "boot_id": payload.get("boot_id", ""),
                "capabilities": payload.get("capabilities") or [],
                "metadata": payload,
            },
        )
        robot.connection_status = "online"
        robot.status = "online"
        robot.agent_version = session.agent_version
        robot.capabilities = session.capabilities
        current_map = payload.get("current_map") or {}
        robot.current_map_id = current_map.get("map_id", "")
        robot.current_map_version = current_map.get("map_version", "")
    elif envelope.message_type == "presence.heartbeat":
        RobotSession.objects.filter(session_id=session_uuid, robot=robot).update(last_heartbeat_at=now)
        robot.connection_status = "online"
        robot.status = "online"
    else:
        RobotSession.objects.filter(session_id=session_uuid, robot=robot).update(
            disconnected_at=now,
            disconnect_reason=payload.get("reason", "unexpected_disconnect"),
        )
        robot.connection_status = "offline"
        robot.status = "offline"
    robot.last_heartbeat_at = now
    robot.last_seen_at = now
    robot.save()
    realtime_publisher.publish_robot_status(robot.id, {"connection_status": robot.connection_status})
    return {"connection_status": robot.connection_status}


def _get_command(payload: dict, robot: Robot) -> RemoteCommand:
    try:
        return RemoteCommand.objects.select_for_update().get(pk=payload["command_id"], robot=robot)
    except RemoteCommand.DoesNotExist as exc:
        raise ProtocolError("UNKNOWN_COMMAND", "command not found") from exc


def _handle_command_ack(envelope: MessageEnvelope, robot: Robot) -> dict:
    payload = envelope.payload
    command = _get_command(payload, robot)
    if command.status in {"accepted", "rejected", "executing", "succeeded", "failed", "cancelled"}:
        return {"duplicate": True, "status": command.status}
    accepted = payload["ack"] == "accepted"
    command.status = "accepted" if accepted else "rejected"
    command.acknowledged_at = _event_time(payload, "acknowledged_at", "accepted_at")
    command.ack_reason_code = payload.get("reason_code") or ""
    command.ack_reason_message = payload.get("reason_message") or ""
    command.save()
    CommandEvent.objects.create(
        command=command,
        event_type="ack",
        source="edge",
        message_id=envelope.message_id,
        payload=payload,
    )
    execution = command.task_execution
    if execution and command.command_type == "task.start":
        target = "accepted" if accepted else "rejected"
        TaskExecutionService.transition(
            execution,
            target,
            event_type="command.ack",
            state_version=int(payload.get("edge_state_version") or execution.state_version + 1),
            occurred_at=command.acknowledged_at,
            reason_code=command.ack_reason_code,
            reason_message=command.ack_reason_message,
            payload=payload,
        )
    realtime_publisher.publish_task_event(str(execution.id), payload) if execution else None
    return {"status": command.status}


def _handle_command_result(envelope: MessageEnvelope, robot: Robot) -> dict:
    payload = envelope.payload
    command = _get_command(payload, robot)
    terminal_status = payload["status"]
    command.status = terminal_status
    command.started_at = _event_time(payload, "started_at")
    command.finished_at = _event_time(payload, "finished_at")
    command.error_code = payload.get("error_code") or ""
    command.error_message = payload.get("error_message") or ""
    command.result_payload = payload.get("result") or {}
    command.save()
    CommandEvent.objects.get_or_create(
        message_id=envelope.message_id,
        defaults={
            "command": command,
            "event_type": "result",
            "source": "edge",
            "payload": payload,
        },
    )
    execution = command.task_execution
    if execution:
        final_state = command.result_payload.get("final_task_state")
        if not final_state and terminal_status in {"failed", "timed_out", "expired"}:
            final_state = "timed_out" if command.error_code in {"COMMAND_EXPIRED", "COMMAND_TIMED_OUT"} else "failed"
        if final_state and final_state != execution.state:
            execution = TaskExecutionService.transition(
                execution,
                final_state,
                event_type="command.result",
                state_version=max(
                    int(command.result_payload.get("state_version") or 0),
                    execution.state_version + 1,
                ),
                occurred_at=command.finished_at,
                reason_code=command.error_code,
                reason_message=command.error_message,
                payload=payload,
            )
        result_payload = command.result_payload or {}
        progress_fields = []
        if "completed_waypoints" in result_payload:
            execution.completed_waypoints = int(result_payload["completed_waypoints"])
            progress_fields.append("completed_waypoints")
        if "total_waypoints" in result_payload:
            execution.total_waypoints = int(result_payload["total_waypoints"])
            progress_fields.append("total_waypoints")
        if "current_waypoint_index" in result_payload:
            execution.current_waypoint_index = result_payload["current_waypoint_index"]
            progress_fields.append("current_waypoint_index")
        if progress_fields:
            execution.save(update_fields=progress_fields + ["updated_at"])
        realtime_publisher.publish_task_event(str(execution.id), payload)
    return {"status": command.status}


def _handle_task_event(envelope: MessageEnvelope, robot: Robot) -> dict:
    payload = envelope.payload
    try:
        execution = TaskExecution.objects.select_for_update().get(
            pk=payload["task_execution_id"],
            robot=robot,
        )
    except TaskExecution.DoesNotExist as exc:
        raise ProtocolError("UNKNOWN_TASK_EXECUTION", "task execution not found") from exc
    if envelope.message_type == "task.progress":
        execution = TaskExecutionService.apply_progress(execution, payload)
    else:
        target = payload["state"]
        incoming_version = int(payload["state_version"])
        if incoming_version <= execution.state_version:
            return {"ignored": True, "reason": "stale_state_version"}
        try:
            execution = TaskExecutionService.transition(
                execution,
                target,
                event_type=envelope.message_type,
                state_version=incoming_version,
                occurred_at=_event_time(payload, "occurred_at", "reported_at"),
                message_id=envelope.message_id,
                reason_code=payload.get("reason_code") or "",
                reason_message=payload.get("reason_message") or "",
                payload=payload,
            )
        except TaskStateError:
            if target != execution.state:
                raise
    realtime_publisher.publish_task_event(str(execution.id), TaskExecutionSerializer(execution).data)
    return {"state": execution.state, "state_version": execution.state_version}


def _handle_trajectory(
    envelope: MessageEnvelope,
    robot: Robot,
    publish_response: ResponsePublisher | None,
) -> dict:
    payload = envelope.payload
    batch_id = uuid.UUID(str(payload["batch_id"]))
    existing = TrajectoryBatchReceipt.objects.filter(batch_id=batch_id).first()
    if existing:
        result = {
            "task_execution_id": str(existing.task_execution_id),
            "batch_id": str(existing.batch_id),
            "accepted_first_seq": existing.first_seq,
            "accepted_last_seq": existing.last_seq,
            "duplicate": True,
        }
        _publish_trajectory_ack(robot, envelope, result, publish_response)
        return result
    try:
        execution = TaskExecution.objects.get(pk=payload["task_execution_id"], robot=robot)
    except TaskExecution.DoesNotExist as exc:
        raise ProtocolError("UNKNOWN_TASK_EXECUTION", "task execution not found") from exc
    points = [
        TrajectoryPoint(
            robot=robot,
            task_execution=execution,
            seq=point["seq"],
            sampled_at=point["sampled_at"],
            frame_id=payload.get("frame_id", "map"),
            map_id=payload.get("map_id", ""),
            map_version=payload.get("map_version", ""),
            x=point["x"],
            y=point["y"],
            yaw=point["yaw"],
            speed_mps=point.get("speed_mps"),
            localization_status=point["localization_status"],
            batch_id=batch_id,
        )
        for point in payload["points"]
    ]
    TrajectoryPoint.objects.bulk_create(points, ignore_conflicts=True)
    receipt = TrajectoryBatchReceipt.objects.create(
        batch_id=batch_id,
        robot=robot,
        task_execution=execution,
        first_seq=payload["first_seq"],
        last_seq=payload["last_seq"],
        point_count=len(points),
    )
    result = {
        "task_execution_id": str(execution.id),
        "batch_id": str(receipt.batch_id),
        "accepted_first_seq": receipt.first_seq,
        "accepted_last_seq": receipt.last_seq,
        "duplicate": False,
    }
    _publish_trajectory_ack(robot, envelope, result, publish_response)
    realtime_publisher.publish_task_event(str(execution.id), {"type": "trajectory.updated", **result})
    return result


def _publish_trajectory_ack(
    robot: Robot,
    envelope: MessageEnvelope,
    payload: dict,
    publish_response: ResponsePublisher | None,
) -> None:
    if publish_response:
        publish_response(
            f"robots/{robot.code}/sync/state",
            {
                "protocol_version": "1.0",
                "message_id": str(uuid.uuid4()),
                "message_type": "trajectory.ack",
                "robot_id": robot.code,
                "session_id": "center",
                "sent_at": timezone.now().isoformat(timespec="milliseconds"),
                "trace_id": str(envelope.trace_id),
                "payload": payload,
            },
            1,
            False,
        )


def _handle_sync(
    envelope: MessageEnvelope,
    robot: Robot,
    publish_response: ResponsePublisher | None,
) -> dict:
    payload = envelope.payload
    execution = None
    if payload.get("current_task_execution_id"):
        execution = TaskExecution.objects.filter(pk=payload["current_task_execution_id"], robot=robot).first()
    if execution is None:
        response_payload = {
            "task_execution_id": None,
            "expected_task_state": None,
            "expected_state_version": 0,
            "action": "report_only",
            "last_accepted_trajectory_seq": -1,
        }
    else:
        last_point = execution.trajectory_points.order_by("-seq").first()
        action = "continue"
        if execution.state != payload.get("local_task_state"):
            action = "hold"
            if execution.state in {"cancelling", "cancelled"}:
                action = "cancel"
        response_payload = {
            "task_execution_id": str(execution.id),
            "expected_task_state": execution.state,
            "expected_state_version": execution.state_version,
            "action": action,
            "last_accepted_trajectory_seq": last_point.seq if last_point else -1,
        }
    if publish_response:
        publish_response(
            f"robots/{robot.code}/sync/state",
            {
                "protocol_version": "1.0",
                "message_id": str(uuid.uuid4()),
                "message_type": "sync.response",
                "robot_id": robot.code,
                "session_id": "center",
                "sent_at": timezone.now().isoformat(timespec="milliseconds"),
                "trace_id": str(envelope.trace_id),
                "payload": response_payload,
            },
            1,
            False,
        )
    return response_payload
