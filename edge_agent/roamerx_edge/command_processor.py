from __future__ import annotations

import logging
import threading
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
        map_activation_adapter=None,
        navigation_stack_adapter=None,
        localization_adapter=None,
        teleop_control_adapter=None,
    ) -> None:
        self.robot_id = robot_id
        self.store = store
        self.safety = safety
        self.task_executor = task_executor
        self.mapping_adapter = mapping_adapter
        self.map_activation_adapter = map_activation_adapter
        self.navigation_stack_adapter = navigation_stack_adapter
        self.localization_adapter = localization_adapter
        self.teleop_control_adapter = teleop_control_adapter
        self.publish_ack = publish_ack
        self.publish_result = publish_result
        self._navigation_command_lock = threading.Lock()

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
        if envelope.message_type.startswith("map."):
            return self._execute_map(envelope, started_at)
        if envelope.message_type.startswith("nav."):
            return self._execute_navigation(envelope, started_at)
        if envelope.message_type.startswith("teleop."):
            return self._execute_teleop(envelope, started_at)
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

    def _execute_navigation(self, envelope: MessageEnvelope, started_at: str) -> dict:
        command = envelope.payload.get("command") or {}
        if envelope.message_type == "nav.status":
            if not self.navigation_stack_adapter:
                raise ProtocolError("NAVIGATION_STACK_UNAVAILABLE", "navigation stack adapter is not configured")
            result_payload = self.navigation_stack_adapter.status()
        else:
            if not self._navigation_command_lock.acquire(blocking=False):
                raise ProtocolError("NAV_COMMAND_BUSY", "another navigation command is still running")
            try:
                if envelope.message_type == "nav.initial_pose":
                    if not self.localization_adapter:
                        raise ProtocolError("LOCALIZATION_UNAVAILABLE", "localization adapter is not configured")
                    result_payload = self.localization_adapter.set_initial_pose(command)
                else:
                    if not self.navigation_stack_adapter:
                        raise ProtocolError("NAVIGATION_STACK_UNAVAILABLE", "navigation stack adapter is not configured")
                    if envelope.message_type == "nav.start":
                        result_payload = self.navigation_stack_adapter.start(command)
                    elif envelope.message_type == "nav.restart":
                        result_payload = self.navigation_stack_adapter.restart(command)
                    elif envelope.message_type == "nav.recover":
                        result_payload = self.navigation_stack_adapter.recover(command)
                    elif envelope.message_type == "nav.stop":
                        result_payload = self.navigation_stack_adapter.stop(command)
                    else:
                        raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
            finally:
                self._navigation_command_lock.release()
        return build_result(
            envelope,
            status="succeeded",
            result=result_payload,
            started_at=started_at,
        )

    def _execute_teleop(self, envelope: MessageEnvelope, started_at: str) -> dict:
        teleop_adapter = self.localization_adapter
        if not teleop_adapter:
            raise ProtocolError("TELEOP_UNAVAILABLE", "teleop adapter is not configured")
        bridge_status = None
        if self.teleop_control_adapter:
            bridge_status = self.teleop_control_adapter.ensure_ready()
        command = envelope.payload.get("command") or {}
        action = envelope.message_type.removeprefix("teleop.")
        if action == "takeover_enter":
            result_payload = teleop_adapter.teleop_action("stand_up")
            self.safety.state.control_mode = "manual_takeover"
        elif action == "takeover_exit":
            teleop_adapter.teleop_velocity(0.0, 0.0, 0.0)
            result_payload = teleop_adapter.teleop_action("passive")
            self.safety.state.control_mode = "autonomous"
        elif action == "stand_up":
            result_payload = teleop_adapter.teleop_action("stand_up")
            self.safety.state.control_mode = "manual_takeover"
        elif action == "lie_down":
            teleop_adapter.teleop_velocity(0.0, 0.0, 0.0)
            result_payload = teleop_adapter.teleop_action("lie_down")
            self.safety.state.control_mode = "autonomous"
        elif action == "move_stop":
            result_payload = teleop_adapter.teleop_velocity(0.0, 0.0, 0.0)
        elif action == "move_forward":
            result_payload = teleop_adapter.teleop_velocity(vx=float(command.get("vx", 0.35)))
        elif action == "move_backward":
            result_payload = teleop_adapter.teleop_velocity(vx=float(command.get("vx", -0.35)))
        elif action == "move_left":
            result_payload = teleop_adapter.teleop_velocity(vy=float(command.get("vy", 0.25)))
        elif action == "move_right":
            result_payload = teleop_adapter.teleop_velocity(vy=float(command.get("vy", -0.25)))
        elif action == "turn_left":
            result_payload = teleop_adapter.teleop_velocity(yaw_rate=float(command.get("yaw_rate", 0.45)))
        elif action == "turn_right":
            result_payload = teleop_adapter.teleop_velocity(yaw_rate=float(command.get("yaw_rate", -0.45)))
        elif action == "passive":
            result_payload = teleop_adapter.teleop_action("passive")
            self.safety.state.control_mode = "autonomous"
        else:
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        if bridge_status is not None:
            result_payload["teleop_bridge"] = bridge_status
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

    def _execute_map(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if not self.map_activation_adapter:
            raise ProtocolError("MAP_ACTIVATION_UNAVAILABLE", "map activation adapter is not configured")
        command = envelope.payload.get("command") or {}
        if envelope.message_type == "map.activate":
            result_payload = self.map_activation_adapter.activate(command)
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
