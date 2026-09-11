from __future__ import annotations

import uuid
import logging
from datetime import datetime, timedelta

from django.conf import settings
from django.db import IntegrityError, transaction
from django.utils import timezone

from ..models import (
    PatrolLoopEvent,
    PatrolLoopSession,
    PatrolTask,
    RemoteCommand,
    Robot,
    RobotLowBatteryEpisode,
    RobotStatusLatest,
    TaskExecution,
)
from .command_service import CommandService
from .task_service import TaskExecutionService, TaskStateError, build_route_snapshot


LOGGER = logging.getLogger(__name__)


class PatrolLoopError(ValueError):
    pass


class PatrolLoopService:
    OBSERVATION_SECONDS = 5
    MAX_RECOVERY_ATTEMPTS = 10
    STOP_SPEED_MPS = 0.05
    STATUS_FRESH_SECONDS = 15
    LOW_BATTERY_PERCENT = getattr(settings, "LOW_BATTERY_STOP_PERCENT", 20)
    LOW_BATTERY_REARM_PERCENT = getattr(settings, "LOW_BATTERY_REARM_PERCENT", 25)
    RECOVERY_IN_PROGRESS_TIMEOUT_SECONDS = 180

    @staticmethod
    def _metadata(session: PatrolLoopSession) -> dict:
        return dict(session.metadata or {})

    @classmethod
    def _clear_recovery_episode(cls, session: PatrolLoopSession) -> None:
        """Clear a completed episode so a later same-code fault is new work."""
        session.recovery_attempt = 0
        session.recovery_episode_id = None
        session.recovery_reason_code = ""
        session.recovery_reason_message = ""
        session.observation_started_at = None
        metadata = cls._metadata(session)
        for key in (
            "observation_blocker",
            "recovery_in_progress_started_at",
            "stop_scope",
        ):
            metadata.pop(key, None)
        session.metadata = metadata

    @classmethod
    def _begin_recovery_stop(
        cls,
        session: PatrolLoopSession,
        execution: TaskExecution | None,
        *,
        reason_code: str,
        reason_message: str,
    ) -> PatrolLoopSession:
        """End only the current round after the Edge confirms a safe stop."""
        cls._force_exit_execution(execution, reason="recovery_attempts_exhausted")
        metadata = cls._metadata(session)
        metadata["stop_scope"] = "round"
        session.metadata = metadata
        return cls._set_state(
            session,
            "stopping",
            "loop.recovery_stopping",
            reason_code=reason_code,
            reason_message=reason_message,
            next_action_at=timezone.now() + timedelta(seconds=1),
        )

    @classmethod
    @transaction.atomic
    def create_session(
        cls,
        *,
        task: PatrolTask,
        duration_seconds: int,
        rest_seconds: int = 0,
        operator=None,
        session_id=None,
    ) -> tuple[PatrolLoopSession, bool]:
        duration_seconds = int(duration_seconds)
        rest_seconds = int(rest_seconds)
        if duration_seconds <= 0:
            raise PatrolLoopError("duration_seconds 必须大于 0")
        if not getattr(settings, "PATROL_LOOP_SELF_HEAL_ENABLED", True):
            raise PatrolLoopError("中心循环自愈功能当前未启用")
        if rest_seconds < 0:
            raise PatrolLoopError("rest_seconds 不能小于 0")
        requested_id = uuid.UUID(str(session_id)) if session_id else uuid.uuid4()
        existing = PatrolLoopSession.objects.filter(pk=requested_id).first()
        if existing:
            if existing.task_id != task.id:
                raise PatrolLoopError("session_id 已被其他任务使用")
            return existing, False
        robot = Robot.objects.select_for_update().get(pk=task.robot_id)
        if PatrolLoopSession.objects.filter(
            robot=robot, state__in=PatrolLoopSession.ACTIVE_STATES
        ).exists():
            raise PatrolLoopError("该机器人已有活动循环")
        if TaskExecution.objects.filter(robot=robot, state__in=TaskExecution.ACTIVE_STATES).exists():
            raise PatrolLoopError("该机器人已有活动任务")
        if not task.enabled or task.route_id is None:
            raise PatrolLoopError("巡逻任务未启用或缺少路线")
        if robot.effective_connection_status() != "online":
            raise PatrolLoopError("机器狗 Edge Agent 当前离线")
        if robot.localization_status != "normal" or not robot.nav_ready:
            raise PatrolLoopError("机器狗定位或导航栈未就绪")
        if robot.capabilities and "task.recover.v1" not in robot.capabilities:
            raise PatrolLoopError("当前 Edge 版本不支持中心循环自愈，请先升级 Edge")
        CommandService.ensure_task_start_allowed(robot)
        now = timezone.now()
        session = PatrolLoopSession.objects.create(
            id=requested_id,
            robot=robot,
            task=task,
            route_snapshot=build_route_snapshot(task.route),
            duration_seconds=duration_seconds,
            rest_seconds=rest_seconds,
            started_at=now,
            ends_at=now + timedelta(seconds=duration_seconds),
            next_action_at=now,
            recovery_max_attempts=cls.MAX_RECOVERY_ATTEMPTS,
            created_by=operator,
        )
        cls._event(session, "loop.created", key="created")
        return session, True

    @classmethod
    def _event(
        cls,
        session: PatrolLoopSession,
        event_type: str,
        *,
        key: str,
        reason_code: str = "",
        reason_message: str = "",
        payload: dict | None = None,
    ) -> None:
        PatrolLoopEvent.objects.get_or_create(
            idempotency_key=f"{session.id}:{key}"[:160],
            defaults={
                "loop_session": session,
                "task_execution": session.current_execution,
                "event_type": event_type,
                "state": session.state,
                "reason_code": reason_code,
                "reason_message": reason_message,
                "recovery_attempt": session.recovery_attempt,
                "payload": payload or {},
            },
        )

    @classmethod
    def _set_state(
        cls,
        session: PatrolLoopSession,
        state: str,
        event_type: str,
        *,
        reason_code: str = "",
        reason_message: str = "",
        next_action_at=None,
        terminal: bool = False,
        payload: dict | None = None,
    ) -> PatrolLoopSession:
        session.state = state
        session.state_version += 1
        if next_action_at is not None:
            session.next_action_at = next_action_at
        if terminal:
            session.finished_at = timezone.now()
        session.save()
        cls._event(
            session,
            event_type,
            key=f"v{session.state_version}:{event_type}",
            reason_code=reason_code,
            reason_message=reason_message,
            payload=payload,
        )
        return session

    @classmethod
    def _is_docking_execution(cls, execution: TaskExecution | None) -> bool:
        if execution is None:
            return False
        start = execution.commands.filter(command_type="task.start").order_by("issued_at").first()
        return bool((((start.payload if start else {}) or {}).get("docking") or {}).get("enabled"))

    @classmethod
    def _force_exit_execution(cls, execution: TaskExecution | None, *, reason: str) -> None:
        if execution is None or execution.state not in TaskExecution.ACTIVE_STATES:
            return
        if cls._is_docking_execution(execution):
            return
        pending = execution.commands.filter(
            command_type="task.force_exit",
            status__in=["created", "published", "accepted", "executing"],
        ).exists()
        if not pending:
            CommandService.create(
                execution,
                "task.force_exit",
                command_options={"reason": reason},
            )

    @classmethod
    @transaction.atomic
    def pause(cls, session: PatrolLoopSession, operator=None) -> PatrolLoopSession:
        session = PatrolLoopSession.objects.select_for_update().get(pk=session.pk)
        if session.state not in PatrolLoopSession.ACTIVE_STATES:
            raise PatrolLoopError("循环已结束")
        session.manual_paused = True
        if session.current_execution and session.current_execution.state in {
            "accepted", "running", "resuming"
        }:
            CommandService.create(session.current_execution, "task.pause", operator)
        return cls._set_state(
            session,
            "paused",
            "loop.manual_paused",
            next_action_at=timezone.now() + timedelta(seconds=1),
        )

    @classmethod
    def _ensure_manual_pause(cls, session: PatrolLoopSession) -> PatrolLoopSession:
        execution = session.current_execution
        if execution:
            execution.refresh_from_db()
        if execution and execution.state in {"accepted", "running", "resuming"}:
            pending = execution.commands.filter(
                command_type="task.pause",
                status__in=["created", "published", "accepted", "executing"],
            ).exists()
            if not pending:
                CommandService.create(execution, "task.pause", session.created_by)
        session.next_action_at = timezone.now() + timedelta(seconds=1)
        session.save(update_fields=["next_action_at", "updated_at"])
        return session

    @classmethod
    @transaction.atomic
    def resume(cls, session: PatrolLoopSession) -> PatrolLoopSession:
        session = PatrolLoopSession.objects.select_for_update().get(pk=session.pk)
        if session.state != "paused" or not session.manual_paused:
            raise PatrolLoopError("循环不处于人工暂停状态")
        session.manual_paused = False
        session.observation_started_at = None
        session.recovery_episode_id = uuid.uuid4()
        session.recovery_reason_code = "MANUAL_RESUME"
        session.recovery_reason_message = "操作员要求继续，重新执行安全观察"
        return cls._set_state(
            session,
            "observing",
            "loop.manual_resume_requested",
            next_action_at=timezone.now(),
        )

    @classmethod
    @transaction.atomic
    def stop(cls, session: PatrolLoopSession, operator=None) -> PatrolLoopSession:
        session = PatrolLoopSession.objects.select_for_update().get(pk=session.pk)
        if session.state in PatrolLoopSession.TERMINAL_STATES:
            return session
        cls._force_exit_execution(session.current_execution, reason="loop_operator_stop")
        session.manual_paused = False
        return cls._set_state(
            session,
            "cancelled",
            "loop.stopped",
            reason_code="OPERATOR_STOP",
            reason_message="操作员停止循环",
            terminal=True,
        )

    @classmethod
    @transaction.atomic
    def terminate_low_battery(
        cls,
        *,
        robot: Robot,
        episode_key: str,
        battery_percent: int,
        source: str,
        task_execution_id=None,
    ) -> RobotLowBatteryEpisode:
        known = RobotLowBatteryEpisode.objects.select_for_update().filter(
            robot=robot,
            episode_key=str(episode_key)[:128],
        ).first()
        if known is not None and not known.active:
            # A replayed alert from a cleared episode must not stop a newer task.
            return known
        active = RobotLowBatteryEpisode.objects.select_for_update().filter(
            robot=robot, active=True
        ).first()
        if active is None:
            active = known or RobotLowBatteryEpisode.objects.create(
                robot=robot,
                episode_key=str(episode_key)[:128],
                battery_percent=max(0, min(100, int(battery_percent))),
                threshold_percent=cls.LOW_BATTERY_PERCENT,
                rearm_percent=cls.LOW_BATTERY_REARM_PERCENT,
                source=source,
            )
        sessions = list(
            PatrolLoopSession.objects.select_for_update().filter(
                robot=robot, state__in=PatrolLoopSession.ACTIVE_STATES
            )
        )
        executions = list(
            TaskExecution.objects.select_for_update().filter(
                robot=robot, state__in=TaskExecution.ACTIVE_STATES
            )
        )
        for session in sessions:
            if cls._is_docking_execution(session.current_execution):
                continue
            session.manual_paused = False
            session.last_error = f"电量 {battery_percent}% 低于巡逻阈值"
            cls._set_state(
                session,
                "low_battery_stopped",
                "loop.low_battery_stopped",
                reason_code="LOW_BATTERY",
                reason_message=session.last_error,
                terminal=True,
                payload={"episode_key": active.episode_key, "source": source},
            )
        for execution in executions:
            if cls._is_docking_execution(execution):
                continue
            cls._force_exit_execution(execution, reason="LOW_BATTERY")
        return active

    @classmethod
    @transaction.atomic
    def observe_battery(cls, latest: RobotStatusLatest) -> None:
        if not latest.power_available or latest.battery_percent is None:
            return
        percent = int(latest.battery_percent)
        active = RobotLowBatteryEpisode.objects.select_for_update().filter(
            robot=latest.robot, active=True
        ).first()
        if active and percent >= active.rearm_percent:
            active.active = False
            active.cleared_at = timezone.now()
            active.save(update_fields=["active", "cleared_at", "updated_at"])
            return
        if percent >= cls.LOW_BATTERY_PERCENT or active:
            return
        # The Edge is authoritative for the two-sample latch.  The center uses
        # two distinct status versions as a delivery-loss fallback.
        cache_key = f"low_battery_samples:{latest.robot_id}"
        from django.core.cache import cache

        sample = cache.get(cache_key) or {"count": 0, "version": None}
        if sample.get("version") == latest.state_version:
            return
        count = int(sample.get("count") or 0) + 1
        cache.set(cache_key, {"count": count, "version": latest.state_version}, timeout=60)
        if count >= 2:
            cls.terminate_low_battery(
                robot=latest.robot,
                episode_key=f"center-{latest.robot_id}-{latest.state_version}",
                battery_percent=percent,
                source="center_telemetry",
                task_execution_id=latest.task_execution_id,
            )
            cache.delete(cache_key)

    @classmethod
    def _health(cls, session: PatrolLoopSession) -> tuple[bool, str, str]:
        safe, code, message = cls._observation_safety(session)
        if not safe:
            return safe, code, message
        latest = RobotStatusLatest.objects.filter(robot=session.robot).first()
        if latest.localization_status != "normal":
            return False, "LOCALIZATION_NOT_READY", "定位尚未恢复"
        if not latest.ros_ready or not latest.nav_ready:
            return False, "NAV_NOT_READY", "ROS 或 Nav2 尚未就绪"
        return True, "", ""

    @classmethod
    def _observation_safety(cls, session: PatrolLoopSession) -> tuple[bool, str, str]:
        robot = session.robot
        latest = RobotStatusLatest.objects.filter(robot=robot).first()
        if robot.effective_connection_status() != "online" or latest is None:
            return False, "EDGE_OFFLINE", "Edge 离线或没有状态数据"
        if latest.received_at < timezone.now() - timedelta(seconds=cls.STATUS_FRESH_SECONDS):
            return False, "TELEMETRY_STALE", "设备状态已过期"
        if latest.emergency_stop or latest.control_mode == "emergency_stop":
            return False, "EMERGENCY_STOP", "急停未解除"
        if latest.control_mode == "manual_takeover":
            return False, "MANUAL_TAKEOVER", "机器人仍处于人工接管"
        if latest.power_available and latest.battery_percent is not None:
            if int(latest.battery_percent) < cls.LOW_BATTERY_PERCENT:
                return False, "LOW_BATTERY", "电量低于巡逻阈值"
        if latest.speed_mps is None or abs(float(latest.speed_mps)) > cls.STOP_SPEED_MPS:
            return False, "ROBOT_NOT_STOPPED", "尚未确认机器人停止"
        return True, "", ""

    @classmethod
    def _dispatch_round(cls, session: PatrolLoopSession) -> PatrolLoopSession:
        round_number = session.current_round + 1
        existing = TaskExecutionService.find_loop_execution(session.id, round_number)
        if existing is None:
            execution = TaskExecutionService.create_execution(
                session.task,
                session.created_by,
                loop_session_id=session.id,
                round_number=round_number,
                route_snapshot=session.route_snapshot,
            )
            CommandService.create(
                execution,
                "task.start",
                session.created_by,
                command_options={"loop_execution": True, "loop_total": 1},
            )
        else:
            execution = existing
        session.current_round = round_number
        session.current_execution = execution
        session.recovery_attempt = 0
        session.recovery_episode_id = None
        session.recovery_reason_code = ""
        session.recovery_reason_message = ""
        session.observation_started_at = None
        return cls._set_state(
            session,
            "running",
            "loop.round_dispatched",
            next_action_at=timezone.now() + timedelta(seconds=1),
            payload={"round_number": round_number, "execution_id": str(execution.id)},
        )

    @classmethod
    def _begin_observation(
        cls, session: PatrolLoopSession, *, code: str, message: str
    ) -> PatrolLoopSession:
        session.observation_started_at = None
        if session.recovery_episode_id is None or session.recovery_reason_code != code:
            session.recovery_episode_id = uuid.uuid4()
            session.recovery_attempt = 0
        session.recovery_reason_code = code or "TASK_INTERRUPTED"
        session.recovery_reason_message = message or "任务异常，等待恢复"
        return cls._set_state(
            session,
            "observing",
            "loop.recovery_observation_started",
            reason_code=session.recovery_reason_code,
            reason_message=session.recovery_reason_message,
            next_action_at=timezone.now(),
        )

    @classmethod
    @transaction.atomic
    def process(cls, session_id) -> PatrolLoopSession:
        session = PatrolLoopSession.objects.select_for_update().select_related(
            "robot", "task", "task__route", "current_execution"
        ).get(pk=session_id)
        now = timezone.now()
        if session.state in PatrolLoopSession.TERMINAL_STATES:
            return session
        if session.state == "stopping":
            execution = session.current_execution
            if execution:
                execution.refresh_from_db()
            force_command = (
                execution.commands.filter(command_type="task.force_exit").order_by("-issued_at").first()
                if execution
                else None
            )
            if force_command and force_command.status in {
                "created", "published", "accepted", "executing"
            }:
                session.next_action_at = now + timedelta(seconds=1)
                session.save(update_fields=["next_action_at", "updated_at"])
                return session
            if execution is None or execution.state not in TaskExecution.ACTIVE_STATES:
                stopped = force_command is None or bool(
                    force_command.status == "succeeded"
                    and (force_command.result_payload or {}).get("robot_stopped")
                )
                if (session.metadata or {}).get("stop_scope") == "round":
                    if not stopped:
                        return cls._set_state(
                            session,
                            "failed",
                            "loop.recovery_stop_unconfirmed",
                            reason_code="ROBOT_STOP_UNCONFIRMED",
                            reason_message="自愈耗尽后未收到机器人停车确认",
                            terminal=True,
                        )
                    cls._clear_recovery_episode(session)
                    return cls._set_state(
                        session,
                        "resting",
                        "loop.recovery_stop_confirmed",
                        reason_code="RECOVERY_ATTEMPTS_EXHAUSTED",
                        reason_message="本轮自愈耗尽，机器人已停止；将继续下一轮",
                        next_action_at=now + timedelta(seconds=session.rest_seconds),
                    )
                return cls._set_state(
                    session,
                    "completed" if stopped else "failed",
                    "loop.stop_confirmed" if stopped else "loop.stop_unconfirmed",
                    reason_code="LOOP_DEADLINE" if stopped else "ROBOT_STOP_UNCONFIRMED",
                    reason_message=(
                        "循环时长已达到，机器人已停止"
                        if stopped
                        else "循环已停止调度，但尚未收到机器人停车确认"
                    ),
                    terminal=True,
                )
            session.next_action_at = now + timedelta(seconds=1)
            session.save(update_fields=["next_action_at", "updated_at"])
            return session
        if now >= session.ends_at:
            cls._force_exit_execution(session.current_execution, reason="loop_deadline")
            if session.current_execution and session.current_execution.state in TaskExecution.ACTIVE_STATES:
                return cls._set_state(
                    session,
                    "stopping",
                    "loop.deadline_stopping",
                    reason_code="LOOP_DEADLINE",
                    reason_message="循环时长已达到，正在确认机器人停止",
                    next_action_at=now + timedelta(seconds=1),
                )
            return cls._set_state(
                session,
                "completed",
                "loop.deadline_reached",
                reason_code="LOOP_DEADLINE",
                reason_message="循环时长已达到，任务已安全停止",
                terminal=True,
            )
        if session.manual_paused or session.state == "paused":
            return cls._ensure_manual_pause(session)
        latest = RobotStatusLatest.objects.filter(robot=session.robot).first()
        if latest:
            cls.observe_battery(latest)
            session.refresh_from_db()
            if session.state in PatrolLoopSession.TERMINAL_STATES:
                return session

        execution = session.current_execution
        if execution:
            execution.refresh_from_db()

        if session.state in {"starting", "resting"}:
            if now < session.next_action_at:
                return session
            healthy, code, message = cls._health(session)
            if not healthy:
                return cls._begin_observation(session, code=code, message=message)
            return cls._dispatch_round(session)

        if session.state == "running":
            if execution is None:
                return cls._begin_observation(
                    session, code="EXECUTION_MISSING", message="循环当前执行缺失"
                )
            if execution.state == "completed":
                session.current_execution = execution
                session.recovery_attempt = 0
                return cls._set_state(
                    session,
                    "resting",
                    "loop.round_completed",
                    next_action_at=now + timedelta(seconds=session.rest_seconds),
                    payload={"round_number": session.current_round},
                )
            if execution.state in {"paused", "interrupted"}:
                return cls._begin_observation(
                    session,
                    code=execution.failure_code or "TASK_INTERRUPTED",
                    message=execution.failure_message or "任务暂停，准备恢复",
                )
            if execution.state in {"failed", "cancelled", "timed_out", "rejected"}:
                if execution.failure_code == "LOW_BATTERY":
                    cls.terminate_low_battery(
                        robot=session.robot,
                        episode_key=f"execution-{execution.id}",
                        battery_percent=session.robot.battery_level,
                        source="task_terminal",
                    )
                    return PatrolLoopSession.objects.get(pk=session.pk)
                return cls._begin_observation(
                    session,
                    code=execution.failure_code or "ROUND_FAILED",
                    message=execution.failure_message or "本轮异常结束，准备下一轮",
                )
            session.next_action_at = now + timedelta(seconds=1)
            session.save(update_fields=["next_action_at", "updated_at"])
            return session

        if session.state == "observing":
            healthy, code, message = cls._observation_safety(session)
            if not healthy:
                metadata = cls._metadata(session)
                blocker = metadata.get("observation_blocker") or {}
                reason_changed = blocker.get("code") != code
                if session.observation_started_at is not None or reason_changed:
                    session.observation_started_at = None
                    # Keep the navigation/localization root cause intact. A
                    # transient movement or interlock is only an observation
                    # blocker, not a replacement recovery diagnosis.
                    metadata["observation_blocker"] = {"code": code, "message": message}
                    session.metadata = metadata
                    session.save()
                    cls._event(
                        session,
                        "loop.observation_reset",
                        key=f"v{session.state_version}:reset:{latest.state_version if latest else now.timestamp()}:{code}",
                        reason_code=code,
                        reason_message=message,
                    )
                session.next_action_at = now + timedelta(seconds=1)
                session.save(update_fields=["next_action_at", "updated_at"])
                return session
            if session.observation_started_at is None:
                session.observation_started_at = now
                # Re-sample every second so the five-second window is truly
                # continuous; an interlock flicker resets it instead of being
                # invisible between its endpoints.
                session.next_action_at = now + timedelta(seconds=1)
                session.save(update_fields=["observation_started_at", "next_action_at", "updated_at"])
                cls._event(
                    session,
                    "loop.observation_healthy",
                    key=f"v{session.state_version}:healthy",
                    payload={"observation_seconds": cls.OBSERVATION_SECONDS},
                )
                return session
            if now < session.observation_started_at + timedelta(seconds=cls.OBSERVATION_SECONDS):
                session.next_action_at = now + timedelta(seconds=1)
                session.save(update_fields=["next_action_at", "updated_at"])
                return session
            if execution is None or execution.state in {
                "completed", "failed", "cancelled", "timed_out", "rejected"
            }:
                session.current_execution = execution
                delay = 0 if session.recovery_reason_code == "MANUAL_RESUME" else session.rest_seconds
                return cls._set_state(
                    session,
                    "resting",
                    "loop.round_recovery_replaced",
                    next_action_at=now + timedelta(seconds=delay),
                )
            if session.recovery_attempt >= session.recovery_max_attempts:
                return cls._begin_recovery_stop(
                    session,
                    execution,
                    reason_code="RECOVERY_ATTEMPTS_EXHAUSTED",
                    reason_message="单个异常自愈已达到 10 次，正在确认结束本轮",
                )
            session.recovery_attempt += 1
            command = CommandService.create_task_recovery(
                execution,
                episode_id=session.recovery_episode_id or uuid.uuid4(),
                attempt=session.recovery_attempt,
                reason_code=session.recovery_reason_code,
                reason_message=session.recovery_reason_message,
            )
            return cls._set_state(
                session,
                "recovering",
                "loop.recovery_dispatched",
                next_action_at=now + timedelta(seconds=1),
                payload={"command_id": str(command.id), "attempt": session.recovery_attempt},
            )

        if session.state == "recovering":
            command = RemoteCommand.objects.filter(
                task_execution=execution,
                command_type="task.recover.v1",
                payload__recovery_episode_id=str(session.recovery_episode_id),
                payload__attempt=session.recovery_attempt,
            ).first()
            if command is None or command.status in {"failed", "cancelled", "rejected", "timed_out", "expired"}:
                session.observation_started_at = None
                return cls._set_state(
                    session,
                    "observing",
                    "loop.recovery_failed",
                    reason_code=command.error_code if command else "RECOVERY_COMMAND_MISSING",
                    reason_message=command.error_message if command else "恢复命令缺失",
                    next_action_at=now,
                )
            if command.status == "succeeded":
                execution.refresh_from_db()
                if execution.state in {"running", "accepted", "resuming"}:
                    cls._clear_recovery_episode(session)
                    return cls._set_state(
                        session,
                        "running",
                        "loop.recovery_succeeded",
                        next_action_at=now + timedelta(seconds=1),
                    )
                result = command.result_payload or {}
                recovery_status = str(result.get("recovery_status") or "").lower()
                if recovery_status == "in_progress":
                    metadata = cls._metadata(session)
                    started_at = metadata.get("recovery_in_progress_started_at")
                    if started_at is None:
                        metadata["recovery_in_progress_started_at"] = now.isoformat()
                        session.metadata = metadata
                        session.next_action_at = now + timedelta(seconds=1)
                        session.save(update_fields=["metadata", "next_action_at", "updated_at"])
                        cls._event(
                            session,
                            "loop.recovery_in_progress",
                            key=f"v{session.state_version}:in_progress:{session.recovery_attempt}",
                            reason_code=result.get("reason_code") or session.recovery_reason_code,
                            reason_message=result.get("reason_message") or "Edge 正在异步恢复",
                        )
                        return session
                    try:
                        elapsed = (now - datetime.fromisoformat(started_at)).total_seconds()
                    except (TypeError, ValueError):
                        elapsed = cls.RECOVERY_IN_PROGRESS_TIMEOUT_SECONDS + 1
                    if elapsed < cls.RECOVERY_IN_PROGRESS_TIMEOUT_SECONDS:
                        session.next_action_at = now + timedelta(seconds=1)
                        session.save(update_fields=["next_action_at", "updated_at"])
                        return session
                    metadata.pop("recovery_in_progress_started_at", None)
                    session.metadata = metadata
                    session.observation_started_at = None
                    return cls._set_state(
                        session,
                        "observing",
                        "loop.recovery_in_progress_timeout",
                        reason_code="RECOVERY_IN_PROGRESS_TIMEOUT",
                        reason_message="Edge 异步恢复超时，重新进行安全观察",
                        next_action_at=now,
                    )
                if recovery_status == "non_retryable":
                    return cls._begin_recovery_stop(
                        session,
                        execution,
                        reason_code=result.get("reason_code") or "RECOVERY_NON_RETRYABLE",
                        reason_message=result.get("reason_message") or "Edge 判定当前异常不可继续恢复",
                    )
                session.observation_started_at = None
                return cls._set_state(
                    session,
                    "observing",
                    "loop.recovery_still_held",
                    next_action_at=now,
                )
            session.next_action_at = now + timedelta(seconds=1)
            session.save(update_fields=["next_action_at", "updated_at"])
        return session

    @classmethod
    def process_due(cls, *, limit: int = 50) -> int:
        ids = list(
            PatrolLoopSession.objects.filter(
                state__in=PatrolLoopSession.ACTIVE_STATES,
                next_action_at__lte=timezone.now(),
            )
            .order_by("next_action_at")
            .values_list("id", flat=True)[:limit]
        )
        processed = 0
        for session_id in ids:
            try:
                cls.process(session_id)
                processed += 1
            except (IntegrityError, TaskStateError, PatrolLoopError):
                LOGGER.exception("patrol loop processing failed session=%s", session_id)
                session = PatrolLoopSession.objects.filter(pk=session_id).first()
                if session and session.state in PatrolLoopSession.ACTIVE_STATES:
                    session.last_error = "循环监督处理失败"
                    session.next_action_at = timezone.now() + timedelta(seconds=1)
                    session.save(update_fields=["last_error", "next_action_at", "updated_at"])
        return processed
