from __future__ import annotations

import uuid
import logging
from collections.abc import Callable

from django.conf import settings
from django.db import IntegrityError, transaction
from django.utils import timezone

from .models import (
    CommandEvent,
    InboundMessage,
    RemoteCommand,
    Robot,
    RobotCommand,
    RobotSession,
    TaskExecution,
    TaskExecutionEvent,
    SpeechTemplate,
    TrajectoryBatchReceipt,
    TrajectoryPoint,
)
from .protocol import MessageEnvelope, ProtocolError, parse_message
from .realtime_gateway import realtime_publisher
from .serializers import EventSerializer, TaskExecutionSerializer
from .services.alert_service import AlertService
from .services.task_service import TaskExecutionService, TaskStateError
from .services.telemetry_service import TelemetryService
from .services import tts_service
from .services.alert_skill_service import resolve_alert_template


ResponsePublisher = Callable[[str, dict, int, bool], None]
LOGGER = logging.getLogger(__name__)


def _public_media_url(saved_path: str) -> str:
    base_url = str(getattr(settings, "PUBLIC_BASE_URL", "") or "").rstrip("/")
    media_url = str(settings.MEDIA_URL).strip("/")
    return f"{base_url}/{media_url}/{saved_path.lstrip('/')}"


def _queue_waypoint_speech(
    execution: TaskExecution,
    robot: Robot,
    waypoint_index: int | None = None,
    waypoint_id: str = "",
):
    waypoints = (execution.route_snapshot or {}).get("waypoints") or []
    if waypoint_id:
        waypoint_index = next(
            (
                index
                for index, waypoint in enumerate(waypoints)
                if str(waypoint.get("waypoint_id") or "") == str(waypoint_id)
            ),
            None,
        )
    if waypoint_index is None:
        return None
    if waypoint_index < 0 or waypoint_index >= len(waypoints):
        return None
    waypoint = waypoints[waypoint_index]
    template_id = waypoint.get("speech_template_id")
    if not template_id:
        return None
    duplicate_filter = {
        "robot": robot,
        "action": "play_audio",
        "payload__source": "patrol_waypoint_speech",
        "payload__task_execution_id": str(execution.id),
        "payload__waypoint_index": waypoint_index,
    }
    if RobotCommand.objects.filter(**duplicate_filter).exists():
        return None
    template = SpeechTemplate.objects.filter(
        id=template_id,
        category__name=settings.INSPECTION_SPEECH_CATEGORY_NAME,
    ).first()
    if not template:
        LOGGER.warning("waypoint speech template unavailable execution=%s waypoint=%s", execution.id, waypoint_index)
        return None
    try:
        saved_path, cache_hit = tts_service.synthesize_speech(template.text)
    except Exception:
        LOGGER.exception("waypoint speech synthesis failed execution=%s waypoint=%s", execution.id, waypoint_index)
        return None
    return RobotCommand.objects.create(
        robot=robot,
        action="play_audio",
        payload={
            "audio_url": _public_media_url(saved_path),
            "audio_name": template.name,
            "text": template.text,
            "source": "patrol_waypoint_speech",
            "content_type": "audio/mpeg",
            "tts_cache_hit": cache_hit,
            "task_execution_id": str(execution.id),
            "waypoint_index": waypoint_index,
            "waypoint_id": waypoint.get("waypoint_id", ""),
            "blocking_fifo": True,
        },
    )


MAPPING_DIVERGED_SPEECH = "建图定位已发散，请立即停止移动。请到地图页停止并生成救援地图。"


def _queue_mapping_divergence_speech(robot: Robot, payload: dict):
    session_id = str((payload.get("attributes") or {}).get("mapping_session_id") or "")
    duplicate_filter = {
        "robot": robot,
        "action": "play_audio",
        "payload__source": "mapping_divergence_speech",
    }
    if session_id:
        duplicate_filter["payload__mapping_session_id"] = session_id
    if RobotCommand.objects.filter(**duplicate_filter).exists():
        return None
    try:
        saved_path, cache_hit = tts_service.synthesize_speech(MAPPING_DIVERGED_SPEECH)
    except Exception:
        LOGGER.exception("mapping divergence speech synthesis failed robot=%s", robot.code)
        return None
    return RobotCommand.objects.create(
        robot=robot,
        action="play_audio",
        payload={
            "audio_url": _public_media_url(saved_path),
            "audio_name": "建图定位已发散",
            "text": MAPPING_DIVERGED_SPEECH,
            "source": "mapping_divergence_speech",
            "dual_output": True,
            "content_type": "audio/mpeg",
            "tts_cache_hit": cache_hit,
            "mapping_session_id": session_id,
            "alert_event_id": payload.get("event_id", ""),
        },
    )


def _queue_low_battery_alert_speech(robot: Robot, payload: dict):
    episode_id = str((payload.get("attributes") or {}).get("low_battery_episode_id") or payload.get("event_id") or "")
    duplicate_filter = {
        "robot": robot,
        "action": "play_audio",
        "payload__source": "low_battery_alert_speech",
        "payload__low_battery_episode_id": episode_id,
    }
    if RobotCommand.objects.filter(**duplicate_filter).exists():
        return None
    template = resolve_alert_template("low_battery_return_charge", "低电量停车告警")
    if not template:
        LOGGER.warning("low-battery alert speech skill is disabled or has no template robot=%s", robot.code)
        return None
    try:
        saved_path, cache_hit = tts_service.synthesize_speech(template.text)
    except Exception:
        LOGGER.exception("low-battery return speech synthesis failed robot=%s", robot.code)
        return None
    RobotCommand.objects.filter(
        robot=robot,
        action="play_audio",
        status="queued",
        payload__source="patrol_waypoint_speech",
    ).update(
        status="superseded",
        error_message="低电量停车告警已中止巡检点位播报",
        updated_at=timezone.now(),
    )
    return RobotCommand.objects.create(
        robot=robot,
        action="play_audio",
        payload={
            "audio_url": _public_media_url(saved_path),
            "audio_name": template.name,
            "text": template.text,
            "source": "low_battery_alert_speech",
            "alert_skill": "low_battery_return_charge",
            "dual_output": True,
            "priority": "critical",
            "content_type": "audio/mpeg",
            "tts_cache_hit": cache_hit,
            "low_battery_episode_id": episode_id,
            "alert_event_id": payload.get("event_id", ""),
        },
    )


def _queue_obstacle_speech(execution: TaskExecution, robot: Robot, payload: dict):
    """Queue the fixed recovery announcement selected by the robot-side stage."""
    stage_skills = {
        "obstacle_detected": ("obstacle_detected", "发现障碍物"),
        "recovery_attempt": ("avoidance", "后退尝试避障"),
        "leave_route": ("dissuasion", "劝阻离开线路"),
    }
    stage = str(payload.get("speech_stage") or "")
    skill = stage_skills.get(stage)
    if not skill:
        LOGGER.warning("unsupported obstacle speech stage execution=%s stage=%s", execution.id, stage)
        return None
    skill_key, title = skill
    attempt = int(payload.get("recovery_attempt") or 0)
    episode_id = str(payload.get("obstacle_episode_id") or "")
    duplicate_filter = {
        "robot": robot,
        "action": "play_audio",
        "payload__source": "patrol_obstacle_speech",
        "payload__task_execution_id": str(execution.id),
        "payload__speech_stage": stage,
        "payload__recovery_attempt": attempt,
    }
    if episode_id:
        duplicate_filter["payload__obstacle_episode_id"] = episode_id
    if RobotCommand.objects.filter(**duplicate_filter).exists():
        return None
    template = resolve_alert_template(skill_key, title)
    if not template:
        LOGGER.warning("obstacle speech template missing execution=%s title=%s", execution.id, title)
        return None
    try:
        saved_path, cache_hit = tts_service.synthesize_speech(template.text)
    except Exception:
        LOGGER.exception("obstacle speech synthesis failed execution=%s title=%s", execution.id, title)
        return None
    return RobotCommand.objects.create(
        robot=robot,
        action="play_audio",
        payload={
            "audio_url": _public_media_url(saved_path),
            "audio_name": template.name,
            "text": template.text,
            "source": "patrol_obstacle_speech",
            "alert_skill": skill_key,
            "dual_output": True,
            "content_type": "audio/mpeg",
            "tts_cache_hit": cache_hit,
            "task_execution_id": str(execution.id),
            "obstacle_episode_id": episode_id,
            "speech_stage": stage,
            "recovery_attempt": attempt,
            "front_obstacle_distance_m": payload.get("front_obstacle_distance_m"),
        },
    )


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
        if created and event is not None:
            realtime_publisher.publish_alert(
                {
                    "event": EventSerializer(event).data,
                    "robot": {"id": robot.id, "code": robot.code, "name": robot.name},
                }
            )
        source_code = str((payload.get("source") or {}).get("code") or "")
        if payload.get("event_type") == "slam_diverged" or source_code == "SLAM_DIVERGED":
            _queue_mapping_divergence_speech(robot, payload)
        if payload.get("event_type") in {
            "low_battery_alert",
            "low_battery_return_charge",
        } or source_code in {"LOW_BATTERY_ALERT", "LOW_BATTERY_RETURN_CHARGE"}:
            # System safety handling stays active even though non-bicycle
            # alerts no longer create business event-center records.
            _queue_low_battery_alert_speech(robot, payload)
            return {
                "created": created,
                "registered": event is not None,
                "event_id": str(event.event_id) if event else str(payload["event_id"]),
                "automatic_docking": False,
            }
        return {
            "created": created,
            "registered": event is not None,
            "event_id": str(event.event_id) if event else str(payload["event_id"]),
        }
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
    if command.command_type == "map.activate" and terminal_status == "succeeded":
        current_map = command.result_payload.get("current_map") or command.result_payload
        robot.current_map_id = str(current_map.get("map_id") or "")
        robot.current_map_version = str(current_map.get("map_version") or "")
        robot.last_seen_at = timezone.now()
        robot.last_heartbeat_at = timezone.now()
        robot.save(update_fields=[
            "current_map_id",
            "current_map_version",
            "last_seen_at",
            "last_heartbeat_at",
            "updated_at",
        ])
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
            if command.command_type == "task.pause":
                # A late pause failure must never overwrite a newer resume.
                # If pause is still the latest intent, restore running state.
                if execution.state == "pausing":
                    execution = TaskExecutionService.transition(
                        execution,
                        "running",
                        event_type="task.pause.failed",
                        reason_code=command.error_code,
                        reason_message=command.error_message,
                        payload=payload,
                    )
                final_state = None
            elif command.command_type == "task.resume":
                # Symmetric handling: keep a newer pause, otherwise restore
                # the last confirmed paused state.
                if execution.state == "resuming":
                    execution = TaskExecutionService.transition(
                        execution,
                        "paused",
                        event_type="task.resume.failed",
                        reason_code=command.error_code,
                        reason_message=command.error_message,
                        payload=payload,
                    )
                final_state = None
            else:
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
    if envelope.message_type == "task.obstacle_speech":
        command = _queue_obstacle_speech(execution, robot, payload)
        realtime_publisher.publish_task_event(
            str(execution.id),
            {"type": "obstacle_speech", "payload": payload, "audio_command_id": command.id if command else None},
        )
        return {"state": execution.state, "state_version": execution.state_version, "audio_command_id": command.id if command else None}
    if envelope.message_type == "task.round_started":
        execution.round_number = max(execution.round_number, int(payload.get("round_number") or execution.round_number))
        execution.state_version = max(execution.state_version, int(payload.get("state_version") or execution.state_version))
        execution.save(update_fields=["round_number", "state_version", "updated_at"])
        realtime_publisher.publish_task_event(str(execution.id), {"type": "round_started", "payload": payload})
        return {"state": execution.state, "state_version": execution.state_version, "round_number": execution.round_number}
    if envelope.message_type == "task.progress":
        execution = TaskExecutionService.apply_progress(execution, payload)
        milestone = str(payload.get("milestone") or "")
        if milestone in {"target_dispatched", "waypoint_reached"} and execution.state_version == int(payload["state_version"]):
            TaskExecutionEvent.objects.get_or_create(
                task_execution=execution,
                state_version=execution.state_version,
                defaults={
                    "state": execution.state,
                    "event_type": f"task.{milestone}",
                    "occurred_at": _event_time(payload, "reported_at", "occurred_at"),
                    "message_id": envelope.message_id,
                    "payload": payload,
                },
            )
        if milestone == "waypoint_reached":
            waypoint = payload.get("waypoint") or {}
            _queue_waypoint_speech(
                execution,
                robot,
                waypoint_index=payload.get("execution_waypoint_index"),
                waypoint_id=str(waypoint.get("waypoint_id") or payload.get("current_waypoint_id") or ""),
            )
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
        local_state = str(payload.get("local_task_state") or "")
        reconciled = False
        if (
            execution.state in TaskExecution.ACTIVE_STATES
            and local_state in {"completed", "failed", "cancelled", "timed_out", "rejected"}
        ):
            try:
                local_version = int(payload.get("local_task_state_version") or 0)
            except (TypeError, ValueError):
                local_version = 0
            execution = TaskExecutionService.transition(
                execution,
                local_state,
                event_type="task.sync_terminal_reconciled",
                state_version=max(execution.state_version + 1, local_version),
                reason_code="" if local_state == "completed" else "EDGE_SYNC_TERMINAL",
                reason_message="" if local_state == "completed" else f"Edge 重连时上报终态 {local_state}",
                payload={
                    "source": "edge_sync",
                    "previous_cloud_state": execution.state,
                    "local_task_state": local_state,
                    "local_task_state_version": local_version,
                    "outbox_pending": payload.get("outbox_pending"),
                },
            )
            realtime_publisher.publish_task_event(
                str(execution.id),
                TaskExecutionSerializer(execution).data,
            )
            reconciled = True
        last_point = execution.trajectory_points.order_by("-seq").first()
        action = "report_only" if reconciled else "continue"
        if not reconciled and execution.state != local_state:
            action = "hold"
            if execution.state in {"cancelling", "cancelled"}:
                action = "cancel"
        response_payload = {
            "task_execution_id": str(execution.id),
            "expected_task_state": execution.state,
            "expected_state_version": execution.state_version,
            "action": action,
            "last_accepted_trajectory_seq": last_point.seq if last_point else -1,
            "terminal_reconciled": reconciled,
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
