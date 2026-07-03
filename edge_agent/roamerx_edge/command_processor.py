from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Callable

from .local_store import LocalStore
from .protocol import MessageEnvelope, ProtocolError, build_ack, build_result, decode_message, now_iso
from .safety_policy import SafetyPolicy
from .task_executor import TaskExecutor

LOGGER = logging.getLogger(__name__)


class CommandProcessor:
    def __init__(
        self,
        *,
        robot_id: str,
        store: LocalStore,
        safety: SafetyPolicy,
        task_executor: TaskExecutor,
        publish_ack: Callable[[str, dict], None],
        publish_result: Callable[[str, dict], None],
        mapping_adapter=None,
    ) -> None:
        self.robot_id = robot_id
        self.store = store
        self.safety = safety
        self.task_executor = task_executor
        self.mapping_adapter = mapping_adapter
        self.publish_ack = publish_ack
        self.publish_result = publish_result

    def handle_command(self, raw) -> tuple[dict, dict | None]:
        envelope = decode_message(raw)
        LOGGER.info("handle_command: type=%s id=%s", envelope.message_type, envelope.payload.get("command_id", "?"))
        if envelope.robot_id != self.robot_id:
            raise ProtocolError("ROBOT_ID_MISMATCH", "command robot_id does not match this device")
        command_id = envelope.payload["command_id"]
        duplicate = self.store.get_processed_command(command_id)
        if duplicate:
            self.publish_ack(command_id, duplicate["ack"])
            if duplicate["result"]:
                self.publish_result(command_id, duplicate["result"])
            return duplicate["ack"], duplicate["result"]

        started_at = now_iso()
        try:
            if datetime.now(timezone.utc) >= datetime.fromisoformat(
                envelope.payload["expires_at"].replace("Z", "+00:00")
            ):
                raise ProtocolError("COMMAND_EXPIRED", "command has expired")
            self._validate_expected_state(envelope)
            ack = build_ack(
                envelope,
                accepted=True,
                edge_state_version=self._state_version(),
            )
            self.store.save_command_ack(command_id, ack)
            self.publish_ack(command_id, ack)
            result = self._execute(envelope, started_at)
            if result:
                self.store.save_command_result(command_id, result)
                self.publish_result(command_id, result)
            return ack, result
        except ProtocolError as exc:
            # If ack was already sent as accepted, send a failure result instead of
            # a reject ack (which cloud would ignore as duplicate).
            result = build_result(
                envelope,
                status="failed",
                result={},
                started_at=started_at,
                error_code=exc.code,
                error_message=exc.message,
            )
            self.store.save_command_result(command_id, result)
            self.publish_result(command_id, result)
            return ack if "ack" in dir() else {}, result
        except Exception as exc:
            LOGGER.exception("unexpected error executing command %s", command_id)
            result = build_result(
                envelope,
                status="failed",
                result={},
                started_at=started_at,
                error_code="INTERNAL_ERROR",
                error_message=str(exc),
            )
            self.store.save_command_result(command_id, result)
            self.publish_result(command_id, result)
            return ack if "ack" in dir() else {}, result

    def _validate_expected_state(self, envelope: MessageEnvelope) -> None:
        expected = envelope.payload.get("expected_robot_state_version")
        if expected is None:
            return
        current = self._state_version()
        if int(expected) < current:
            raise ProtocolError("STALE_ROBOT_STATE", f"expected={expected}, current={current}")

    def _execute(self, envelope: MessageEnvelope, started_at: str) -> dict | None:
        if envelope.message_type == "task.start":
            self.safety.validate_task_start(envelope, self.task_executor.has_active_task())
            self.task_executor.start_task(envelope)
            return None
        if envelope.message_type.startswith("mapping."):
            return self._execute_mapping(envelope, started_at)
        execution_id = envelope.payload["task_execution_id"]
        if not self.task_executor.context:
            raise ProtocolError("TASK_CONTEXT_MISMATCH", "no active task context")
        if envelope.message_type == "task.pause":
            self.safety.validate_pause(self.task_executor.context.state)
            result_payload = self.task_executor.pause_task(execution_id)
        elif envelope.message_type == "task.resume":
            self.safety.validate_resume(self.task_executor.context.state)
            resume_index = int(envelope.payload["command"].get("resume_from_waypoint_index", -1))
            result_payload = self.task_executor.resume_task(execution_id, resume_index)
        else:
            self.safety.validate_cancel(self.task_executor.context.state)
            result_payload = self.task_executor.cancel_task(execution_id)
        return build_result(
            envelope,
            status="succeeded",
            result=result_payload,
            started_at=started_at,
        )

    def _execute_mapping(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if not self.mapping_adapter:
            raise ProtocolError("MAPPING_UNAVAILABLE", "mapping adapter is not configured")
        command = envelope.payload.get("command") or {}
        if envelope.message_type == "mapping.start":
            result_payload = self.mapping_adapter.start_mapping(command)
        elif envelope.message_type == "mapping.save":
            result_payload = self.mapping_adapter.save_mapping(command)
        elif envelope.message_type == "mapping.cancel":
            result_payload = self.mapping_adapter.cancel_mapping(command)
        elif envelope.message_type == "mapping.status":
            result_payload = self.mapping_adapter.status()
        else:
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        return build_result(
            envelope,
            status="succeeded",
            result=result_payload,
            started_at=started_at,
        )

    def _state_version(self) -> int:
        return self.task_executor.context.state_version if self.task_executor.context else 0
