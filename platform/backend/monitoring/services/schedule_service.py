from __future__ import annotations

from collections import defaultdict
from datetime import date, datetime, time, timedelta

from django.db import IntegrityError, transaction
from django.utils import timezone

from ..models import CalendarDay, PatrolSchedule, RemoteCommand, ScheduleRun, TaskExecution
from .command_service import CommandService
from .task_service import TaskExecutionService, TaskStateError


class ScheduleService:
    @staticmethod
    def local_minute(value=None):
        local = timezone.localtime(value or timezone.now())
        return local.replace(second=0, microsecond=0)

    @staticmethod
    def day_meta(target_date: date) -> dict:
        special = CalendarDay.objects.filter(date=target_date, enabled=True).first()
        if not special:
            return {"date": target_date.isoformat(), "day_type": "normal", "name": "", "label": "普通日"}
        return {
            "date": target_date.isoformat(),
            "day_type": special.day_type,
            "name": special.name,
            "label": special.get_day_type_display(),
        }

    @classmethod
    def schedule_matches_date(cls, schedule: PatrolSchedule, target_date: date, day_type: str) -> bool:
        if schedule.schedule_type == "once":
            return schedule.run_date == target_date
        if schedule.weekdays and target_date.isoweekday() not in schedule.weekdays:
            return False
        if schedule.schedule_type == "holiday":
            return day_type in {"holiday", "event_day"}
        return schedule.schedule_type == "daily"

    @classmethod
    def planned_start(cls, target_date: date, target_time: time) -> datetime:
        naive = datetime.combine(target_date, target_time)
        if timezone.is_naive(naive):
            return timezone.make_aware(naive, timezone.get_current_timezone())
        return naive

    @classmethod
    def schedules_for_day(cls, target_date: date, robot_id: int | None = None):
        day_type = cls.day_meta(target_date)["day_type"]
        queryset = PatrolSchedule.objects.select_related(
            "robot", "task_template", "route", "map_data"
        ).filter(enabled=True)
        if robot_id:
            queryset = queryset.filter(robot_id=robot_id)
        result = []
        for schedule in queryset:
            if cls.schedule_matches_date(schedule, target_date, day_type):
                result.append(schedule)
        return sorted(result, key=lambda item: (item.time_of_day, -item.priority, item.name))

    @classmethod
    def calendar_payload(cls, start_date: date, end_date: date, robot_id: int | None = None) -> dict:
        days = []
        current = start_date
        while current <= end_date:
            meta = cls.day_meta(current)
            schedules = cls.schedules_for_day(current, robot_id=robot_id)
            planned_times = [cls.planned_start(current, schedule.time_of_day) for schedule in schedules]
            runs = ScheduleRun.objects.select_related("task_execution").filter(
                planned_start_at__in=planned_times
            )
            run_by_schedule_time = {(run.schedule_id, run.planned_start_at): run for run in runs}
            items = []
            for schedule in schedules:
                planned = cls.planned_start(current, schedule.time_of_day)
                run = run_by_schedule_time.get((schedule.id, planned))
                items.append(
                    {
                        "schedule_id": schedule.id,
                        "schedule_name": schedule.name,
                        "task_name": schedule.task_template.name,
                        "robot_id": schedule.robot_id,
                        "robot_name": schedule.robot.name,
                        "route_name": schedule.route.name,
                        "schedule_type": schedule.schedule_type,
                        "time": schedule.time_of_day.strftime("%H:%M"),
                        "planned_start_at": planned,
                        "priority": schedule.priority,
                        "status": run.status if run else "pending",
                        "run_id": str(run.id) if run else None,
                        "task_execution_id": str(run.task_execution_id) if run and run.task_execution_id else None,
                        "skip_reason": run.skip_reason if run else "",
                        "error_message": run.error_message if run else "",
                    }
                )
            days.append({**meta, "items": items})
            current += timedelta(days=1)
        return {"from": start_date.isoformat(), "to": end_date.isoformat(), "days": days}

    @classmethod
    def due_schedules(cls, planned_minute=None):
        planned_minute = cls.local_minute(planned_minute)
        target_date = planned_minute.date()
        target_time = planned_minute.time().replace(second=0, microsecond=0)
        day_type = cls.day_meta(target_date)["day_type"]
        candidates = PatrolSchedule.objects.select_related(
            "robot", "task_template", "route", "map_data"
        ).filter(enabled=True, time_of_day=target_time)
        return [
            schedule
            for schedule in candidates
            if cls.schedule_matches_date(schedule, target_date, day_type)
        ]

    @classmethod
    def dispatch_due(cls, planned_minute=None) -> list[ScheduleRun]:
        planned_minute = cls.local_minute(planned_minute)
        due = cls.due_schedules(planned_minute)
        grouped = defaultdict(list)
        for schedule in due:
            grouped[schedule.robot_id].append(schedule)
        runs = []
        for schedules in grouped.values():
            holiday_schedules = [schedule for schedule in schedules if schedule.schedule_type == "holiday"]
            effective = holiday_schedules or schedules
            effective = sorted(effective, key=lambda item: (-item.priority, item.id))
            selected = effective[0]
            for skipped in [item for item in schedules if item.id != selected.id]:
                reason = "holiday_override" if selected.schedule_type == "holiday" and skipped.schedule_type == "daily" else "lower_priority"
                runs.append(cls.record_skip(skipped, planned_minute, reason))
            runs.append(cls.trigger(selected, planned_minute, source="scheduler"))
        return runs

    @classmethod
    def trigger_now(cls, schedule: PatrolSchedule, operator=None) -> ScheduleRun:
        return cls.trigger(schedule, cls.local_minute(), operator=operator, source="run_now")

    @classmethod
    @transaction.atomic
    def record_skip(cls, schedule: PatrolSchedule, planned_start_at, reason: str, message: str = "") -> ScheduleRun:
        run, created = ScheduleRun.objects.get_or_create(
            schedule=schedule,
            planned_start_at=planned_start_at,
            defaults={
                "robot": schedule.robot,
                "triggered_at": timezone.now(),
                "status": "skipped",
                "skip_reason": reason,
                "error_message": message,
            },
        )
        if not created and run.status == "created":
            run.status = "skipped"
            run.skip_reason = reason
            run.error_message = message
            run.triggered_at = timezone.now()
            run.save(update_fields=["status", "skip_reason", "error_message", "triggered_at", "updated_at"])
        return run

    @classmethod
    @transaction.atomic
    def trigger(cls, schedule: PatrolSchedule, planned_start_at, operator=None, source: str = "scheduler") -> ScheduleRun:
        try:
            run, created = ScheduleRun.objects.get_or_create(
                schedule=schedule,
                planned_start_at=planned_start_at,
                defaults={
                    "robot": schedule.robot,
                    "triggered_at": timezone.now(),
                    "status": "created",
                },
            )
        except IntegrityError:
            return ScheduleRun.objects.get(schedule=schedule, planned_start_at=planned_start_at)
        if not created and run.status != "created":
            return run

        if schedule.robot.status == "offline" or schedule.robot.effective_connection_status() == "offline":
            run.status = "failed"
            run.skip_reason = "ROBOT_OFFLINE"
            run.error_message = "机器人离线，无法下发计划任务"
            run.triggered_at = timezone.now()
            run.save(update_fields=["status", "skip_reason", "error_message", "triggered_at", "updated_at"])
            return run
        if not schedule.task_template.enabled:
            run.status = "skipped"
            run.skip_reason = "TASK_DISABLED"
            run.error_message = "任务模板已停用，计划不自动下发"
            run.triggered_at = timezone.now()
            run.save(update_fields=["status", "skip_reason", "error_message", "triggered_at", "updated_at"])
            return run
        if schedule.task_template.route_id != schedule.route_id:
            run.status = "failed"
            run.skip_reason = "INVALID_CONFIG"
            run.error_message = "MVP 阶段计划路线必须与任务模板路线一致"
            run.triggered_at = timezone.now()
            run.save(update_fields=["status", "skip_reason", "error_message", "triggered_at", "updated_at"])
            return run
        if TaskExecution.objects.filter(robot=schedule.robot, state__in=TaskExecution.ACTIVE_STATES).exists():
            run.status = "skipped"
            run.skip_reason = "ROBOT_BUSY"
            run.error_message = "机器人已有巡检任务运行"
            run.triggered_at = timezone.now()
            run.save(update_fields=["status", "skip_reason", "error_message", "triggered_at", "updated_at"])
            return run

        try:
            execution = TaskExecutionService.create_execution(schedule.task_template, operator)
            command = CommandService.create(execution, "task.start", operator)
        except TaskStateError as exc:
            run.status = "failed"
            run.skip_reason = str(exc)
            run.error_message = str(exc)
            run.triggered_at = timezone.now()
            run.save(update_fields=["status", "skip_reason", "error_message", "triggered_at", "updated_at"])
            return run

        run.status = "dispatched"
        run.task_execution = execution
        run.remote_command = command
        run.triggered_at = timezone.now()
        run.save(update_fields=["status", "task_execution", "remote_command", "triggered_at", "updated_at"])
        schedule.last_triggered_at = run.triggered_at
        schedule.save(update_fields=["last_triggered_at", "updated_at"])
        return run
