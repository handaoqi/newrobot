from django.contrib import admin

from .models import (
    CalendarDay,
    CommandEvent,
    InspectionEvent,
    PatrolSchedule,
    PatrolLoopEvent,
    PatrolLoopSession,
    PatrolTask,
    RemoteCommand,
    Robot,
    RobotLowBatteryEpisode,
    RobotSession,
    RobotStatusLatest,
    RobotTelemetry,
    RobotTelemetryDailySummary,
    TaskExecution,
    TrajectoryPoint,
    ScheduleRun,
)


@admin.register(Robot)
class RobotAdmin(admin.ModelAdmin):
    list_display = ("code", "name", "status", "mode", "location", "battery_level", "today_alerts")
    search_fields = ("code", "name", "location", "area")
    list_filter = ("status", "mode")


@admin.register(InspectionEvent)
class InspectionEventAdmin(admin.ModelAdmin):
    list_display = ("title", "robot", "event_type", "risk_level", "status", "review_result", "detected_at")
    search_fields = ("title", "event_type", "location")
    list_filter = ("status", "review_result", "risk_level", "event_type")


@admin.register(PatrolTask)
class PatrolTaskAdmin(admin.ModelAdmin):
    list_display = ("name", "robot", "route_name", "status", "completion_rate", "scheduled_start")
    list_filter = ("status",)


@admin.register(CalendarDay)
class CalendarDayAdmin(admin.ModelAdmin):
    list_display = ("date", "name", "day_type", "enabled")
    list_filter = ("day_type", "enabled")
    search_fields = ("name", "note")


@admin.register(PatrolSchedule)
class PatrolScheduleAdmin(admin.ModelAdmin):
    list_display = ("name", "robot", "task_template", "schedule_type", "time_of_day", "priority", "enabled")
    list_filter = ("schedule_type", "enabled", "robot")
    search_fields = ("name", "note", "task_template__name")


@admin.register(ScheduleRun)
class ScheduleRunAdmin(admin.ModelAdmin):
    list_display = ("id", "schedule", "robot", "planned_start_at", "status", "task_execution", "skip_reason")
    list_filter = ("status", "robot")


@admin.register(RobotTelemetry)
class RobotTelemetryAdmin(admin.ModelAdmin):
    list_display = ("robot", "sequence_id", "position_name", "battery_level", "reported_at")
    search_fields = ("robot__code", "sequence_id", "position_name")


@admin.register(RobotTelemetryDailySummary)
class RobotTelemetryDailySummaryAdmin(admin.ModelAdmin):
    list_display = ("robot", "day", "sample_count", "active_seconds", "distance_km")
    list_filter = ("day", "robot")


@admin.register(RobotSession)
class RobotSessionAdmin(admin.ModelAdmin):
    list_display = ("robot", "session_id", "transport", "connected_at", "last_heartbeat_at", "disconnected_at")


@admin.register(TaskExecution)
class TaskExecutionAdmin(admin.ModelAdmin):
    list_display = ("id", "task", "robot", "state", "state_version", "current_waypoint_index", "created_at")
    list_filter = ("state",)


@admin.register(PatrolLoopSession)
class PatrolLoopSessionAdmin(admin.ModelAdmin):
    list_display = ("id", "robot", "task", "state", "current_round", "ends_at", "updated_at")
    list_filter = ("state", "robot")


@admin.register(PatrolLoopEvent)
class PatrolLoopEventAdmin(admin.ModelAdmin):
    list_display = ("loop_session", "event_type", "state", "recovery_attempt", "occurred_at")
    list_filter = ("event_type", "state")


@admin.register(RobotLowBatteryEpisode)
class RobotLowBatteryEpisodeAdmin(admin.ModelAdmin):
    list_display = ("robot", "battery_percent", "active", "source", "triggered_at", "cleared_at")
    list_filter = ("active", "source")


@admin.register(RemoteCommand)
class RemoteCommandAdmin(admin.ModelAdmin):
    list_display = ("id", "robot", "command_type", "status", "issued_at", "acknowledged_at", "finished_at")
    list_filter = ("command_type", "status")


@admin.register(CommandEvent)
class CommandEventAdmin(admin.ModelAdmin):
    list_display = ("command", "event_type", "source", "event_at", "message_id")


@admin.register(RobotStatusLatest)
class RobotStatusLatestAdmin(admin.ModelAdmin):
    list_display = ("robot", "state_version", "localization_status", "ros_ready", "nav_ready", "sampled_at")


@admin.register(TrajectoryPoint)
class TrajectoryPointAdmin(admin.ModelAdmin):
    list_display = ("task_execution", "seq", "sampled_at", "x", "y", "localization_status")
