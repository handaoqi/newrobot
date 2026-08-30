from __future__ import annotations

import logging
import threading
from datetime import datetime, timezone
from typing import Callable

from .local_store import LocalStore
from .protocol import MessageEnvelope, ProtocolError, build_ack, build_result, decode_message, now_iso
from .safety_policy import SafetyPolicy
from .task_executor import TaskExecutor
from .teleop_skill_executor import TeleopSkillExecutor

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
        person_follow_controller=None,
        sensor_control_adapter=None,
        charge_control_adapter=None,
        audio_control_adapter=None,
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
        self.person_follow_controller = person_follow_controller
        self.sensor_control_adapter = sensor_control_adapter
        self.charge_control_adapter = charge_control_adapter
        self.audio_control_adapter = audio_control_adapter
        self.publish_ack = publish_ack
        self.publish_result = publish_result
        self._navigation_command_lock = threading.Lock()
        self.skill_executor = TeleopSkillExecutor(localization_adapter) if localization_adapter else None

    def handle_command(self, raw) -> tuple[dict, dict | None]:
        envelope = decode_message(raw)
        LOGGER.info("handle_command: type=%s id=%s", envelope.message_type, envelope.payload.get("command_id", "?"))
        if envelope.robot_id != self.robot_id:
            raise ProtocolError("ROBOT_ID_MISMATCH", "command robot_id does not match this device")
        command_id = envelope.payload["command_id"]
        duplicate = self.store.get_processed_command(command_id)
        if duplicate:
            if duplicate["ack"]:
                self.publish_ack(command_id, duplicate["ack"])
            if duplicate["result"]:
                self.publish_result(command_id, duplicate["result"])
            return duplicate["ack"] or {}, duplicate["result"]

        started_at = now_iso()
        ack = None
        try:
            if datetime.now(timezone.utc) >= datetime.fromisoformat(
                envelope.payload["expires_at"].replace("Z", "+00:00")
            ):
                raise ProtocolError("COMMAND_EXPIRED", "command has expired")
            self._validate_expected_state(envelope)
            prepared_task_start = envelope.message_type == "task.start"
            if prepared_task_start:
                self._release_manual_control_for_task()
                self._ensure_navigation_stack_for_task()
                docking = ((envelope.payload.get("command") or {}).get("docking") or {})
                if docking.get("enabled"):
                    # A return-to-charge route may intentionally use a
                    # dedicated map. Activate it before the final map and
                    # localization checks; validating the old patrol map first
                    # rejects every legitimate cross-map docking task.
                    self._prepare_docking_map(envelope.payload["command"])
                self.safety.wait_until_localization_stable()
                self.safety.validate_task_start(envelope, self.task_executor.has_active_task())
                # The acknowledgement must carry the same task version sequence
                # as task.started/task.progress.  Previously it reused a
                # completed task's stale version, causing progress events to be
                # discarded by the center as out-of-order.
                self.task_executor.prepare_task_start(envelope)
            ack = build_ack(
                envelope,
                accepted=True,
                edge_state_version=self._state_version(),
            )
            self.store.save_command_ack(command_id, ack)
            self.publish_ack(command_id, ack)
            result = None if prepared_task_start else self._execute(envelope, started_at)
            if prepared_task_start:
                # Initialization can actively relocalize and may take up to 90s.
                # Persist and publish the acceptance first so an Edge restart in
                # that window cannot leave the center stuck in `dispatching`.
                if bool((envelope.payload.get("command") or {}).get("smart_initialize", True)):
                    self.task_executor.initialize_before_navigation()
                self.task_executor.launch_prepared_task()
            if result:
                self.store.save_command_result(command_id, result)
                self.publish_result(command_id, result)
            return ack, result
        except ProtocolError as exc:
            if ack is None:
                ack = build_ack(
                    envelope,
                    accepted=False,
                    edge_state_version=self._state_version(),
                    code=exc.code,
                    message=exc.message,
                )
                self.store.save_command_ack(command_id, ack)
                self.publish_ack(command_id, ack)
                return ack, None
            # Once accepted, failures must be reported as a command result.
            # A task.start is acknowledged before launch so the platform sees a
            # strictly ordered lifecycle.  If launch then fails (for example,
            # Nav2's costmap nodes are still appearing), do not retain the
            # accepted context: it would reject every later task as ROBOT_BUSY.
            if prepared_task_start:
                try:
                    self.task_executor.force_exit(envelope.payload["task_execution_id"])
                except Exception:
                    LOGGER.exception("failed to clear task after launch error")
            result = build_result(
                envelope,
                status="failed",
                result=exc.details if isinstance(exc.details, dict) else {},
                started_at=started_at,
                error_code=exc.code,
                error_message=exc.message,
            )
            self.store.save_command_result(command_id, result)
            self.publish_result(command_id, result)
            return ack, result
        except Exception as exc:
            LOGGER.exception("unexpected error executing command %s", command_id)
            if prepared_task_start and ack is not None:
                try:
                    self.task_executor.force_exit(envelope.payload["task_execution_id"])
                except Exception:
                    LOGGER.exception("failed to clear task after unexpected launch error")
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

    def _ensure_navigation_stack_for_task(self) -> None:
        """Repair an unexpectedly stopped Nav2 stack before task validation.

        Nav2 and localization stay warm between tasks. Starting them for every
        route adds latency and resets useful state, but a task should still be
        able to recover when the resident stack was stopped or crashed.
        """
        if self.safety.state.nav_ready or not self.navigation_stack_adapter:
            return
        self.navigation_stack_adapter.start({"reason": "task_start"})
        self._await_navigation_stack_ready(
            timeout_seconds=45.0,
            message="navigation stack did not become ready for task start",
        )

    def _await_navigation_stack_ready(self, timeout_seconds: float, message: str) -> None:
        wait_until_ready = getattr(self.task_executor.navigation, "wait_until_ready", None)
        if not callable(wait_until_ready) or not wait_until_ready(timeout_seconds=timeout_seconds):
            raise ProtocolError("NAV_STACK_NOT_READY", message)
        self.safety.state.nav_ready = True

    def _release_manual_control_for_task(self) -> None:
        """Clear manual input before a navigation task takes control."""
        if self.person_follow_controller:
            self.person_follow_controller.stop("task_takeover")
        if self.safety.state.control_mode != "manual_takeover":
            return
        if self.localization_adapter:
            self.localization_adapter.teleop_velocity(0.0, 0.0, 0.0)
        self.safety.state.control_mode = "autonomous"
        LOGGER.info("task.start cleared manual remote control; teleop bridge remains resident")

    def _prepare_docking_map(self, command: dict) -> None:
        if not self.map_activation_adapter or not self.navigation_stack_adapter:
            raise ProtocolError("DOCKING_UNAVAILABLE", "map activation or navigation stack is unavailable")
        map_payload = dict(command.get("map") or {})
        map_payload.setdefault("map_name", map_payload.get("map_version", ""))
        if not map_payload.get("map_id") or not map_payload.get("map_version"):
            raise ProtocolError("DOCKING_MAP_INVALID", "docking task has no map identity")
        if map_payload.get("local_map_dir"):
            map_payload["local_map_dir"] = map_payload["local_map_dir"]
        activated = self.map_activation_adapter.activate(map_payload)
        self.navigation_stack_adapter.switch_map()
        navigation = self.task_executor.navigation
        if not navigation.wait_until_ready(timeout_seconds=45):
            raise ProtocolError(
                "NAV_STACK_NOT_READY",
                "Nav2 did not become active after switching the docking map",
            )
        LOGGER.info("docking map activated: %s", activated.get("map_id"))

    def _validate_expected_state(self, envelope: MessageEnvelope) -> None:
        # Task controls express an operator's current intent.  They must remain
        # usable while the centre and the edge are briefly out of sync (for
        # example while a navigation result is still being reconciled).
        if envelope.message_type in {"task.pause", "task.resume", "task.force_exit"}:
            return
        expected = envelope.payload.get("expected_robot_state_version")
        if expected is None:
            return
        current = self._state_version()
        if int(expected) < current:
            raise ProtocolError("STALE_ROBOT_STATE", f"expected={expected}, current={current}")

    def _execute(self, envelope: MessageEnvelope, started_at: str) -> dict | None:
        if envelope.message_type == "task.start":
            raise ProtocolError("INVALID_COMMAND_FLOW", "task.start must be prepared before acknowledgement")
        if envelope.message_type.startswith("mapping."):
            return self._execute_mapping(envelope, started_at)
        if envelope.message_type.startswith("map."):
            return self._execute_map(envelope, started_at)
        if envelope.message_type.startswith("nav."):
            return self._execute_navigation(envelope, started_at)
        if envelope.message_type.startswith("teleop."):
            return self._execute_teleop(envelope, started_at)
        if envelope.message_type.startswith("sensor."):
            return self._execute_sensor(envelope, started_at)
        if envelope.message_type.startswith("charge."):
            return self._execute_charge(envelope, started_at)
        if envelope.message_type.startswith("motion."):
            return self._execute_motion_control(envelope, started_at)
        if envelope.message_type.startswith("audio."):
            return self._execute_audio(envelope, started_at)
        execution_id = envelope.payload["task_execution_id"]
        if envelope.message_type == "task.force_exit":
            result_payload = self.task_executor.force_exit(envelope.payload["task_execution_id"])
            return build_result(envelope, status="succeeded", result=result_payload, started_at=started_at)
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
        if envelope.message_type == "nav.single_goal":
            navigation = getattr(self.task_executor, "navigation", None)
            if navigation is None:
                raise ProtocolError("NAVIGATION_STACK_UNAVAILABLE", "navigation adapter is not configured")
            goal = {"x": float(command["x"]), "y": float(command["y"]), "yaw": float(command["yaw"]), "require_yaw": bool(command.get("require_yaw", True))}
            accepted = navigation.send_waypoints([goal], lambda *_: None, lambda *_: None)
            if not accepted:
                raise ProtocolError("NAV_GOAL_REJECTED", "Nav2 rejected the single-point goal")
            result_payload = {"accepted": True, "goal": goal}
        elif envelope.message_type == "nav.status":
            if not self.navigation_stack_adapter:
                raise ProtocolError("NAVIGATION_STACK_UNAVAILABLE", "navigation stack adapter is not configured")
            result_payload = self.navigation_stack_adapter.status()
        elif envelope.message_type in {"nav.initial_pose", "nav.relocalize"}:
            # Pose commands must remain usable while nav.start/restart is
            # waiting for localization; sharing the stack-management lock
            # creates a deadlock where localization can never be initialized.
            if not self.localization_adapter:
                raise ProtocolError("LOCALIZATION_UNAVAILABLE", "localization adapter is not configured")
            if envelope.message_type == "nav.initial_pose":
                if str(command.get("seed_source") or "") == "rtk":
                    result_payload = self.localization_adapter.set_initial_pose_from_rtk(
                        wait_seconds=float(command.get("wait_seconds", 30.0)),
                    )
                else:
                    pose = self._resolve_localization_seed(command)
                    pose.setdefault("wait_seconds", 30.0)
                    # Initialization is complete only after the verified NDT
                    # match has handed ownership to an absolute pose source.
                    # A transient status=3 frame must not acknowledge the UI.
                    pose.setdefault("require_absolute", True)
                    result_payload = self.localization_adapter.set_initial_pose(pose)
            else:
                if str(command.get("seed_source") or "last_trusted") == "global":
                    result_payload = self.localization_adapter.global_relocalize(
                        wait_seconds=float(command.get("wait_seconds", 90.0)),
                    )
                else:
                    seed = self._resolve_localization_seed(command)
                    result_payload = self.localization_adapter.active_relocalize(seed)
        else:
            wait_seconds = 90.0 if envelope.message_type in {
                "nav.start", "nav.restart", "nav.recover",
            } else 5.0
            if not self._navigation_command_lock.acquire(blocking=True, timeout=wait_seconds):
                raise ProtocolError("NAV_COMMAND_BUSY", "another navigation command is still running")
            try:
                if not self.navigation_stack_adapter:
                    raise ProtocolError("NAVIGATION_STACK_UNAVAILABLE", "navigation stack adapter is not configured")
                if envelope.message_type == "nav.start":
                    result_payload = self.navigation_stack_adapter.start(command)
                    self._await_navigation_stack_ready(
                        timeout_seconds=45.0,
                        message="Nav2 did not become ready after nav.start",
                    )
                elif envelope.message_type == "nav.restart":
                    result_payload = self.navigation_stack_adapter.restart(command)
                    self._await_navigation_stack_ready(
                        timeout_seconds=45.0,
                        message="Nav2 did not become ready after nav.restart",
                    )
                elif envelope.message_type == "nav.recover":
                    result_payload = self.navigation_stack_adapter.recover(command)
                    self._await_navigation_stack_ready(
                        timeout_seconds=45.0,
                        message="Nav2 did not become ready after nav.recover",
                    )
                elif envelope.message_type == "nav.stop":
                    result_payload = self.navigation_stack_adapter.stop(command)
                    self.safety.state.nav_ready = False
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

    def _resolve_localization_seed(self, command: dict) -> dict:
        seed = dict(command)
        if all(seed.get(field) is not None for field in ("x", "y", "yaw")):
            return seed
        map_id = str(command.get("map_id") or self.safety.state.current_map_id or "")
        map_version = str(command.get("map_version") or self.safety.state.current_map_version or "")
        seed_source = str(command.get("seed_source") or "last_trusted")
        if seed_source == "mapping_start":
            seed = self.map_activation_adapter.mapping_start_pose() if self.map_activation_adapter else {}
        else:
            seed = self.store.load_last_trusted_pose(map_id, map_version) or {}
        if not all(seed.get(field) is not None for field in ("x", "y", "yaw")):
            raise ProtocolError(
                "RELOCALIZATION_SEED_UNAVAILABLE",
                "no trusted pose exists for this map; select an approximate pose on the map first",
            )
        return seed

    def _execute_teleop(self, envelope: MessageEnvelope, started_at: str) -> dict:
        teleop_adapter = self.localization_adapter
        if not teleop_adapter:
            raise ProtocolError("TELEOP_UNAVAILABLE", "teleop adapter is not configured")
        command = envelope.payload.get("command") or {}
        action = envelope.message_type.removeprefix("teleop.")
        bridge_status = None
        bridge_free_actions = {
            "person_follow_status", "person_follow_stop",
            "skill_list", "skill_status", "skill_cancel",
        }
        if self.teleop_control_adapter and action not in bridge_free_actions:
            velocity_actions = {
                "move_forward", "move_backward", "move_left", "move_right",
                "turn_left", "turn_right", "move_velocity", "move_stop",
            }
            bridge_status = (
                self.teleop_control_adapter.recent_ready_status()
                if action in velocity_actions
                else self.teleop_control_adapter.ensure_ready()
            )
        if self.person_follow_controller and action in {
            "takeover_enter", "takeover_exit", "move_forward", "move_backward",
            "move_left", "move_right", "turn_left", "turn_right", "move_velocity", "skill",
            "shake_hand", "two_leg_stand",
        }:
            self.person_follow_controller.stop("manual_teleop_override")
        if action == "takeover_enter":
            result_payload = teleop_adapter.confirmed_remote_teleop_action(
                "stand_up", {"standing_up", "standing"}, {"stand_up_retrying"}
            )
            self.safety.state.control_mode = "manual_takeover"
        elif action == "takeover_exit":
            teleop_adapter.teleop_velocity(0.0, 0.0, 0.0)
            result_payload = teleop_adapter.release_to_remote_control()
            self.safety.state.control_mode = "autonomous"
        elif action == "stand_up":
            result_payload = teleop_adapter.confirmed_remote_teleop_action(
                "stand_up", {"standing_up", "standing"}, {"stand_up_retrying"}
            )
            self.safety.state.control_mode = "manual_takeover"
        elif action == "lie_down":
            if self.person_follow_controller:
                self.person_follow_controller.stop("lie_down")
            teleop_adapter.teleop_velocity(0.0, 0.0, 0.0)
            result_payload = teleop_adapter.remote_teleop_action("lie_down")
            self.safety.state.control_mode = "autonomous"
        elif action == "shake_hand":
            result_payload = teleop_adapter.confirmed_remote_teleop_action(
                "shake_hand", {"greeting"}
            )
            self.safety.state.control_mode = "manual_takeover"
        elif action == "two_leg_stand":
            result_payload = teleop_adapter.confirmed_remote_teleop_action(
                "two_leg_stand", {"two_leg_standing"}
            )
            self.safety.state.control_mode = "manual_takeover"
        elif action == "move_stop":
            if self.person_follow_controller:
                self.person_follow_controller.stop("move_stop")
            result_payload = teleop_adapter.teleop_velocity(0.0, 0.0, 0.0)
        elif action == "person_follow_start":
            if not self.person_follow_controller:
                raise ProtocolError("PERSON_FOLLOW_UNAVAILABLE", "person follow controller is not configured")
            if self.task_executor.has_active_task():
                raise ProtocolError("PERSON_FOLLOW_UNAVAILABLE", "cannot start person follow while a navigation task is active")
            stand = teleop_adapter.confirmed_remote_teleop_action(
                "stand_up", {"standing_up", "standing"}, {"stand_up_retrying"}
            )
            result_payload = {
                "follow": self.person_follow_controller.start(str(command.get("track_id") or "")),
                "stand": stand,
            }
            self.safety.state.control_mode = "manual_takeover"
        elif action == "person_follow_stop":
            if not self.person_follow_controller:
                raise ProtocolError("PERSON_FOLLOW_UNAVAILABLE", "person follow controller is not configured")
            result_payload = self.person_follow_controller.stop("operator_stop")
        elif action == "person_follow_status":
            if not self.person_follow_controller:
                raise ProtocolError("PERSON_FOLLOW_UNAVAILABLE", "person follow controller is not configured")
            result_payload = self.person_follow_controller.status()
        elif action == "skill":
            if not self.skill_executor:
                raise ProtocolError("TELEOP_UNAVAILABLE", "skill executor is not configured")
            run = self.skill_executor.start(
                envelope.payload["command_id"],
                command,
                lambda outcome: self._complete_skill(envelope, started_at, outcome),
            )
            return None
        elif action == "skill_list":
            if not self.skill_executor:
                raise ProtocolError("TELEOP_UNAVAILABLE", "skill executor is not configured")
            result_payload = {"presets": self.skill_executor.list_presets()}
        elif action == "skill_status":
            if not self.skill_executor:
                raise ProtocolError("TELEOP_UNAVAILABLE", "skill executor is not configured")
            result_payload = self.skill_executor.status(str(command.get("command_id") or ""))
        elif action == "skill_cancel":
            if not self.skill_executor:
                raise ProtocolError("TELEOP_UNAVAILABLE", "skill executor is not configured")
            result_payload = self.skill_executor.cancel(str(command.get("command_id") or ""))
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
        elif action == "move_velocity":
            vx = max(-0.5, min(0.5, float(command.get("vx", 0.0))))
            vy = max(-0.5, min(0.5, float(command.get("vy", 0.0))))
            yaw_rate = max(-0.5, min(0.5, float(command.get("yaw_rate", 0.0))))
            result_payload = teleop_adapter.teleop_velocity(vx=vx, vy=vy, yaw_rate=yaw_rate)
        elif action in {"speed_micro", "speed_slow", "speed_normal", "speed_fast"}:
            result_payload = teleop_adapter.remote_teleop_action(action)
        elif action == "passive":
            if self.person_follow_controller:
                self.person_follow_controller.stop("passive")
            result_payload = teleop_adapter.confirmed_remote_teleop_action(
                "passive", {"passive"}, {"passive_failed"}
            )
            # The bridge remains resident. Its remote passive handler clears
            # the held stick command before issuing the vendor emergency stop.
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

    def _complete_skill(self, envelope: MessageEnvelope, started_at: str, outcome: dict) -> None:
        """Publish the terminal result after the asynchronous local skill ends."""
        status = outcome.pop("status", "failed")
        result = build_result(
            envelope,
            status=status,
            result=outcome,
            started_at=started_at,
            error_code=outcome.get("error_code", ""),
            error_message=outcome.get("error_message", ""),
        )
        command_id = envelope.payload["command_id"]
        self.store.save_command_result(command_id, result)
        self.publish_result(command_id, result)

    def _execute_sensor(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if not self.sensor_control_adapter:
            raise ProtocolError("SENSOR_CONTROL_UNAVAILABLE", "sensor control adapter is not configured")
        if envelope.message_type != "sensor.restart":
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        result_payload = self.sensor_control_adapter.restart(
            (envelope.payload.get("command") or {}).get("sensor")
        )
        return build_result(
            envelope,
            status="succeeded",
            result=result_payload,
            started_at=started_at,
        )

    def _execute_charge(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if not self.charge_control_adapter:
            raise ProtocolError("CHARGE_CONTROL_UNAVAILABLE", "charge control adapter is not configured")
        if envelope.message_type == "charge.start":
            result_payload = self.charge_control_adapter.start()
        elif envelope.message_type == "charge.stop":
            result_payload = self.charge_control_adapter.stop()
        else:
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        return build_result(envelope, status="succeeded", result=result_payload, started_at=started_at)

    def _execute_motion_control(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if not self.charge_control_adapter:
            raise ProtocolError("MOTION_CONTROL_UNAVAILABLE", "charge control adapter is not configured")
        if self.task_executor.has_active_task() and envelope.message_type == "motion.stop":
            raise ProtocolError("TASK_ACTIVE", "cannot stop motion control while a navigation task is active")
        if envelope.message_type == "motion.start":
            result_payload = self.charge_control_adapter.start_motion_control()
        elif envelope.message_type == "motion.stop":
            result_payload = self.charge_control_adapter.stop_motion_control()
        else:
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        return build_result(envelope, status="succeeded", result=result_payload, started_at=started_at)

    def _execute_audio(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if not self.audio_control_adapter:
            raise ProtocolError("AUDIO_CONTROL_UNAVAILABLE", "audio control adapter is not configured")
        if envelope.message_type != "audio.volume":
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        command = envelope.payload.get("command") or {}
        result_payload = self.audio_control_adapter.set_volume(
            str(command.get("target") or ""),
            int(command.get("volume", 0)),
        )
        return build_result(envelope, status="succeeded", result=result_payload, started_at=started_at)

    def _execute_mapping(self, envelope: MessageEnvelope, started_at: str) -> dict:
        if not self.mapping_adapter:
            raise ProtocolError("MAPPING_UNAVAILABLE", "mapping adapter is not configured")
        command = envelope.payload.get("command") or {}
        if envelope.message_type == "mapping.start":
            result_payload = self.mapping_adapter.start_mapping(command)
        elif envelope.message_type == "mapping.origin_start":
            result_payload = self.mapping_adapter.start_origin_lock(command)
        elif envelope.message_type == "mapping.origin_cancel":
            result_payload = self.mapping_adapter.cancel_origin_lock(command)
        elif envelope.message_type == "mapping.origin_extract_global":
            result_payload = self.mapping_adapter.extract_global_enu(command)
        elif envelope.message_type == "mapping.slam_start":
            result_payload = self.mapping_adapter.start_slam_warmup(command)
        elif envelope.message_type == "mapping.begin":
            result_payload = self.mapping_adapter.begin_mapping(command)
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
            if not self.navigation_stack_adapter:
                raise ProtocolError("MAP_RELOAD_UNAVAILABLE", "navigation stack adapter is not configured")
            active_files = result_payload["current_map"]["active_files"]
            reload_if_running = getattr(self.navigation_stack_adapter, "reload_map_if_running", None)
            if callable(reload_if_running):
                result_payload["map_reload"] = reload_if_running(
                    active_files["map.pcd"],
                    active_files["map.yaml"],
                )
            else:
                result_payload["map_reload"] = self.navigation_stack_adapter.reload_map(
                    active_files["map.pcd"],
                    active_files["map.yaml"],
                )
            # A map-local pose cannot be carried across maps.  The map is
            # loaded now, but a fresh map-specific initial pose is required
            # before task admission can consider localization usable.
            self.safety.state.localization_status = "initializing"
            self.safety.state.localization_normal_since_monotonic = 0.0
            result_payload["localization_reset_required"] = True
        elif envelope.message_type == "map.optimize":
            if not self.mapping_adapter:
                raise ProtocolError("MAPPING_UNAVAILABLE", "mapping adapter is not configured")
            context = self.task_executor.context
            if context and context.state not in self.task_executor.TERMINAL_STATES:
                raise ProtocolError("ROBOT_BUSY", "机器人正在执行任务，不能离线优化地图")
            source_dir = self.map_activation_adapter.resolve_source_dir(command)
            result_payload = self.mapping_adapter.optimize_historical_map(command, source_dir)
        else:
            raise ProtocolError("UNSUPPORTED_COMMAND", envelope.message_type)
        return build_result(
            envelope,
            status="succeeded",
            result=result_payload,
            started_at=started_at,
        )

    def _state_version(self) -> int:
        context = self.task_executor.context
        if not context or context.state in self.task_executor.TERMINAL_STATES:
            return 0
        return context.state_version
