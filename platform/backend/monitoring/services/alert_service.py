from __future__ import annotations

from decimal import Decimal

from django.conf import settings
from django.db import transaction
from django.utils import timezone

from ..models import InspectionEvent, MediaAsset, Robot, TaskExecution, TrajectoryPoint


def is_bicycle_detection(detection: dict) -> bool:
    """Keep the business event center scoped to bicycle alerts."""
    values = {
        str(detection.get(field) or "").strip().lower()
        for field in ("object_class", "class", "type", "event_type", "label")
    }
    if values & {
        "bicycle",
        "bike",
        "自行车",
        "vehicle_illegal_parking",
        "自行车违停",
    }:
        return True
    return any("自行车" in value or "bicycle" in value for value in values)


def is_bicycle_alert(payload: dict) -> bool:
    detection = dict(payload.get("detection") or {})
    detection.setdefault("event_type", payload.get("event_type"))
    return is_bicycle_detection(detection)


def is_low_battery_alert(payload: dict) -> bool:
    event_type = str(payload.get("event_type") or "").strip().lower()
    source_code = str((payload.get("source") or {}).get("code") or "").strip().upper()
    object_class = str((payload.get("detection") or {}).get("class") or "").strip().lower()
    return (
        event_type in {"low_battery_alert", "low_battery_return_charge"}
        or source_code in {"LOW_BATTERY_ALERT", "LOW_BATTERY_RETURN_CHARGE"}
        or object_class == "low_battery"
    )


def should_register_edge_alert(payload: dict) -> bool:
    """Register business bicycle events and the operator-facing battery safety alert."""
    return is_bicycle_alert(payload) or is_low_battery_alert(payload)


def _decimal(value):
    return Decimal(str(value)) if value is not None else None


def _confidence_percent(value) -> float:
    confidence = float(value or 0)
    return round(confidence * 100 if confidence <= 1 else confidence, 2)


def _bicycle_alert_cooldown_seconds() -> int:
    try:
        return max(0, int(settings.BICYCLE_ALERT_COOLDOWN_SECONDS))
    except (AttributeError, TypeError, ValueError):
        return 10


class AlertService:
    @staticmethod
    def _recent_bicycle_event(robot: Robot) -> InspectionEvent | None:
        """Find the still-cooling bicycle incident for this robot.

        The check uses the server-created timestamp rather than device event
        time: a delayed/retried device payload must not bypass the cooldown by
        carrying an old or future timestamp.
        """
        cooldown_seconds = _bicycle_alert_cooldown_seconds()
        if cooldown_seconds <= 0:
            return None
        cutoff = timezone.now() - timezone.timedelta(seconds=cooldown_seconds)
        candidates = InspectionEvent.objects.filter(
            robot=robot,
            created_at__gte=cutoff,
        ).order_by("-created_at")[:20]
        for event in candidates:
            detection = dict(event.raw_detection or {})
            detection.update(
                {
                    "label": detection.get("label") or event.title,
                    "type": detection.get("type") or event.event_type,
                    "object_class": detection.get("object_class") or event.object_class,
                }
            )
            if is_bicycle_detection(detection):
                return event
        return None

    @staticmethod
    def _merge_bicycle_event(event: InspectionEvent, detection: dict) -> InspectionEvent:
        """Keep one operator-facing record while retaining merge diagnostics."""
        raw_detection = dict(event.raw_detection or {})
        aggregation = dict(raw_detection.get("alert_aggregation") or {})
        aggregation["merged_reports"] = max(1, int(aggregation.get("merged_reports") or 1)) + 1
        aggregation["cooldown_seconds"] = _bicycle_alert_cooldown_seconds()
        aggregation["last_merged_at"] = timezone.now().isoformat()
        raw_detection.update(detection)
        raw_detection["alert_aggregation"] = aggregation

        bbox = detection.get("bbox") or {}
        updates = {
            "raw_detection": raw_detection,
            "confidence": max(event.confidence, Decimal(str(_confidence_percent(detection.get("confidence"))))),
            "snapshot_url": detection.get("snapshot_url") or event.snapshot_url,
            "camera_id": detection.get("camera_id") or event.camera_id,
            "stream_id": detection.get("stream_id") or event.stream_id,
            "object_class": detection.get("object_class") or event.object_class,
            "track_id": detection.get("track_id") or event.track_id,
            "bbox_x": bbox.get("x") if bbox.get("x") is not None else event.bbox_x,
            "bbox_y": bbox.get("y") if bbox.get("y") is not None else event.bbox_y,
            "bbox_width": bbox.get("width") if bbox.get("width") is not None else event.bbox_width,
            "bbox_height": bbox.get("height") if bbox.get("height") is not None else event.bbox_height,
        }
        for field, value in updates.items():
            setattr(event, field, value)
        event.save(update_fields=[*updates, "updated_at"])
        return event

    @staticmethod
    @transaction.atomic
    def ingest_telemetry_bicycle_detection(
        robot: Robot,
        detection: dict,
        *,
        location: str,
        reported_at,
        camera_id: str,
        stream_id: str,
        frame_width,
        frame_height,
    ) -> tuple[InspectionEvent, bool]:
        """Create one confirmed bicycle incident or merge a cooling duplicate."""
        locked_robot = Robot.objects.select_for_update().get(pk=robot.pk)
        existing = AlertService._recent_bicycle_event(locked_robot)
        if existing:
            return AlertService._merge_bicycle_event(existing, detection), False

        bbox = detection.get("bbox") or {}
        event = InspectionEvent.objects.create(
            robot=locked_robot,
            title=detection.get("label") or detection.get("type") or "AI识别事件",
            event_type=detection.get("type", "generic_detection"),
            location=location,
            detected_at=detection.get("event_time") or reported_at,
            confidence=_confidence_percent(detection.get("confidence")),
            risk_level=detection.get("risk_level", "medium"),
            status="pending",
            snapshot_url=detection.get("snapshot_url", ""),
            description=f"板端识别上报: {detection.get('label') or detection.get('type')}",
            camera_id=detection.get("camera_id") or camera_id,
            stream_id=detection.get("stream_id") or stream_id,
            object_class=detection.get("object_class", ""),
            track_id=detection.get("track_id", ""),
            bbox_x=bbox.get("x"),
            bbox_y=bbox.get("y"),
            bbox_width=bbox.get("width"),
            bbox_height=bbox.get("height"),
            frame_width=frame_width,
            frame_height=frame_height,
            raw_detection={
                **detection,
                "alert_aggregation": {
                    "confirmed_frames": 3,
                    "merged_reports": 1,
                    "cooldown_seconds": _bicycle_alert_cooldown_seconds(),
                    "created_at": timezone.now().isoformat(),
                },
            },
            source_component="bike_bot",
            source_code="BICYCLE_ALERT",
        )
        locked_robot.today_alerts += 1
        locked_robot.save(update_fields=["today_alerts", "updated_at"])
        return event, True

    @staticmethod
    @transaction.atomic
    def ingest_edge_alert(robot: Robot, payload: dict) -> tuple[InspectionEvent | None, bool]:
        if not should_register_edge_alert(payload):
            return None, False
        existing = InspectionEvent.objects.filter(event_id=payload["event_id"]).first()
        if existing:
            return existing, False

        locked_robot = Robot.objects.select_for_update().get(pk=robot.pk)
        detection = payload.get("detection") or {}
        if is_bicycle_alert(payload):
            cooling_event = AlertService._recent_bicycle_event(locked_robot)
            if cooling_event:
                return AlertService._merge_bicycle_event(cooling_event, detection), False

        execution = None
        if payload.get("task_execution_id"):
            execution = TaskExecution.objects.filter(pk=payload["task_execution_id"], robot=locked_robot).first()
        media = payload.get("media") or {}
        snapshot = None
        clip = None
        if media.get("snapshot_media_id"):
            snapshot = MediaAsset.objects.filter(media_id=media["snapshot_media_id"], robot=locked_robot).first()
        if media.get("clip_media_id"):
            clip = MediaAsset.objects.filter(media_id=media["clip_media_id"], robot=locked_robot).first()
        pose = payload.get("pose") or {}
        source = payload.get("source") or {}
        bbox = detection.get("bbox") or {}
        event = InspectionEvent.objects.create(
            event_id=payload["event_id"],
            robot=locked_robot,
            task_execution=execution,
            map_data=execution.map_data if execution else None,
            title=detection.get("label") or payload["event_type"],
            event_type=payload["event_type"],
            location=locked_robot.location,
            detected_at=payload["occurred_at"],
            confidence=_confidence_percent(detection.get("confidence")),
            risk_level=payload.get("severity", "medium"),
            snapshot_url=snapshot.url if snapshot else "",
            snapshot_asset=snapshot,
            clip_asset=clip,
            map_id=payload.get("map_id", ""),
            map_version=payload.get("map_version", ""),
            frame_id=pose.get("frame_id", "map"),
            position_x=_decimal(pose.get("x")),
            position_y=_decimal(pose.get("y")),
            position_yaw=_decimal(pose.get("yaw")),
            source_component=source.get("component", ""),
            source_code=source.get("code", ""),
            model_version=source.get("model_version", ""),
            object_class=detection.get("class", ""),
            track_id=detection.get("track_id", ""),
            bbox_x=bbox.get("x"),
            bbox_y=bbox.get("y"),
            bbox_width=bbox.get("width"),
            bbox_height=bbox.get("height"),
            frame_width=detection.get("frame_width"),
            frame_height=detection.get("frame_height"),
            raw_detection=detection,
            description=f"Edge Agent 上报: {payload['event_type']}",
        )
        locked_robot.today_alerts += 1
        locked_robot.save(update_fields=["today_alerts", "updated_at"])
        return event, True

    @staticmethod
    def build_timeline(event: InspectionEvent) -> dict:
        trajectory = []
        if event.task_execution_id:
            start = event.detected_at - timezone.timedelta(seconds=30)
            end = event.detected_at + timezone.timedelta(seconds=30)
            trajectory = list(
                TrajectoryPoint.objects.filter(
                    task_execution=event.task_execution,
                    sampled_at__gte=start,
                    sampled_at__lte=end,
                ).values("seq", "sampled_at", "x", "y", "yaw", "localization_status")
            )
        task_events = []
        if event.task_execution_id:
            task_events = list(
                event.task_execution.events.filter(
                    occurred_at__lte=event.detected_at + timezone.timedelta(seconds=30)
                )
                .order_by("-occurred_at")[:20]
                .values("state", "state_version", "event_type", "occurred_at", "reason_code")
            )
        return {
            "event_id": str(event.event_id),
            "task_execution_id": str(event.task_execution_id) if event.task_execution_id else None,
            "pose": {
                "frame_id": event.frame_id,
                "x": event.position_x,
                "y": event.position_y,
                "yaw": event.position_yaw,
            },
            "trajectory": trajectory,
            "task_events": list(reversed(task_events)),
            "media": {
                "snapshot_media_id": str(event.snapshot_asset.media_id) if event.snapshot_asset else None,
                "clip_media_id": str(event.clip_asset.media_id) if event.clip_asset else None,
            },
        }
