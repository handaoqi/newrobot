from __future__ import annotations

from decimal import Decimal

from django.db import transaction
from django.utils import timezone

from ..models import InspectionEvent, MediaAsset, Robot, TaskExecution, TrajectoryPoint


def _decimal(value):
    return Decimal(str(value)) if value is not None else None


class AlertService:
    @staticmethod
    @transaction.atomic
    def ingest_edge_alert(robot: Robot, payload: dict) -> tuple[InspectionEvent, bool]:
        existing = InspectionEvent.objects.filter(event_id=payload["event_id"]).first()
        if existing:
            return existing, False

        execution = None
        if payload.get("task_execution_id"):
            execution = TaskExecution.objects.filter(pk=payload["task_execution_id"], robot=robot).first()
        media = payload.get("media") or {}
        snapshot = None
        clip = None
        if media.get("snapshot_media_id"):
            snapshot = MediaAsset.objects.filter(media_id=media["snapshot_media_id"], robot=robot).first()
        if media.get("clip_media_id"):
            clip = MediaAsset.objects.filter(media_id=media["clip_media_id"], robot=robot).first()
        pose = payload.get("pose") or {}
        source = payload.get("source") or {}
        detection = payload.get("detection") or {}
        bbox = detection.get("bbox") or {}
        confidence = detection.get("confidence", 0)
        confidence = float(confidence) * 100 if float(confidence) <= 1 else float(confidence)
        event = InspectionEvent.objects.create(
            event_id=payload["event_id"],
            robot=robot,
            task_execution=execution,
            map_data=execution.map_data if execution else None,
            title=detection.get("label") or payload["event_type"],
            event_type=payload["event_type"],
            location=robot.location,
            detected_at=payload["occurred_at"],
            confidence=round(confidence, 2),
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
        robot.today_alerts += 1
        robot.save(update_fields=["today_alerts", "updated_at"])
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
