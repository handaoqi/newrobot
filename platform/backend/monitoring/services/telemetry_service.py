from __future__ import annotations

from decimal import Decimal

from django.db import transaction
from django.utils import timezone
from django.utils.dateparse import parse_datetime

from ..models import Robot, RobotStatusLatest, TaskExecution


def _decimal(value):
    return Decimal(str(value)) if value is not None else None


class TelemetryService:
    @staticmethod
    @transaction.atomic
    def apply_status(robot: Robot, payload: dict) -> tuple[RobotStatusLatest, bool]:
        incoming_version = int(payload["state_version"])
        latest = RobotStatusLatest.objects.select_for_update().filter(robot=robot).first()
        incoming_sampled_at = parse_datetime(str(payload["sampled_at"]))
        if incoming_sampled_at and timezone.is_naive(incoming_sampled_at):
            incoming_sampled_at = timezone.make_aware(incoming_sampled_at)
        incoming_mapping_progress = (
            ((payload.get("mapping") or {}).get("save_progress") or {}).get("updated_at_unix") or 0
        )
        latest_mapping_progress = (
            ((((latest.raw_payload or {}).get("mapping") or {}).get("save_progress") or {}).get("updated_at_unix") or 0)
            if latest
            else 0
        )
        try:
            mapping_progress_advanced = float(incoming_mapping_progress) > float(latest_mapping_progress)
        except (TypeError, ValueError):
            mapping_progress_advanced = False
        if latest and not mapping_progress_advanced and incoming_version <= latest.state_version and (
            not incoming_sampled_at or incoming_sampled_at <= latest.sampled_at
        ):
            return latest, False

        pose = payload.get("pose") or {}
        localization = payload.get("localization") or {}
        power = payload.get("power") or {}
        network = payload.get("network") or {}
        runtime = payload.get("runtime") or {}
        current_map = payload.get("current_map") or {}
        map_set = payload.get("map_set") or {}
        localization_quality = dict(localization.get("quality") or {})
        if map_set:
            localization_quality["map_set"] = map_set
        execution = None
        execution_id = runtime.get("task_execution_id")
        if execution_id:
            execution = TaskExecution.objects.filter(pk=execution_id, robot=robot).first()

        values = {
            "state_version": incoming_version,
            "sampled_at": incoming_sampled_at or payload["sampled_at"],
            "received_at": timezone.now(),
            "frame_id": pose.get("frame_id", "map"),
            "map_id": current_map.get("map_id", ""),
            "map_version": current_map.get("map_version", ""),
            "x": _decimal(pose.get("x")),
            "y": _decimal(pose.get("y")),
            "z": _decimal(pose.get("z")),
            "yaw": _decimal(pose.get("yaw")),
            "speed_mps": _decimal(pose.get("speed_mps")),
            "localization_status": localization.get("status", "unknown"),
            "localization_source_status": localization.get("source_status"),
            "localization_quality": localization_quality,
            "power_available": bool(power.get("available", False)),
            "battery_percent": power.get("percent") if power.get("available") else None,
            "charging": power.get("charging") if power.get("available") else None,
            "network_type": network.get("type", ""),
            "signal_percent": network.get("signal_percent"),
            "ros_ready": bool(runtime.get("ros_ready", False)),
            "nav_ready": bool(runtime.get("nav_ready", False)),
            "emergency_stop": bool(runtime.get("emergency_stop", False)),
            "control_mode": runtime.get("control_mode", "unknown"),
            "task_execution": execution,
            "raw_payload": payload,
        }
        if latest:
            for field, value in values.items():
                setattr(latest, field, value)
            latest.save()
            created = False
        else:
            latest = RobotStatusLatest.objects.create(robot=robot, **values)
            created = True

        robot.connection_status = "online"
        robot.status = "online"
        robot.localization_status = values["localization_status"]
        robot.ros_ready = values["ros_ready"]
        robot.nav_ready = values["nav_ready"]
        robot.control_mode = values["control_mode"]
        robot.current_map_id = values["map_id"]
        robot.current_map_version = values["map_version"]
        robot.last_state_version = incoming_version
        robot.last_seen_at = timezone.now()
        robot.last_heartbeat_at = timezone.now()
        if values["power_available"] and values["battery_percent"] is not None:
            robot.battery_level = values["battery_percent"]
        if values["signal_percent"] is not None:
            robot.network_strength = values["signal_percent"]
        robot.save()
        return latest, created
