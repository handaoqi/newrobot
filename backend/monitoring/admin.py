from django.contrib import admin

from .models import InspectionEvent, PatrolTask, Robot, RobotTelemetry


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


@admin.register(RobotTelemetry)
class RobotTelemetryAdmin(admin.ModelAdmin):
    list_display = ("robot", "sequence_id", "position_name", "battery_level", "reported_at")
    search_fields = ("robot__code", "sequence_id", "position_name")

# Register your models here.
