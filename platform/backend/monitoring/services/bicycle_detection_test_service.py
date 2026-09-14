from __future__ import annotations

import hashlib
from decimal import Decimal
from pathlib import Path

from django.conf import settings
from django.core.files.base import ContentFile
from django.utils import timezone

from ..models import BicycleDetectionTestImage, BicycleDetectionTestRun, InspectionEvent, MediaAsset


PHOTO_ALERT_SOURCE_COMPONENT = "event_center_photo_test"
PHOTO_ALERT_SOURCE_CODE = "PHOTO_DETECTION_ALERT"
PHOTO_ALERT_EVENT_TYPE = "vehicle_illegal_parking"
PHOTO_ALERT_TITLE = "自行车告警（照片测试）"
VEHICLE_ALERT_CLASSES = frozenset({"bicycle", "car", "motorcycle"})


def _eligible_detections(image: BicycleDetectionTestImage) -> list[dict]:
    """Return all independently-qualified COCO vehicle detections for one image."""
    diagnostics = image.diagnostics if isinstance(image.diagnostics, dict) else {}
    detections = diagnostics.get("detections")
    if isinstance(detections, list):
        eligible = [
            dict(item)
            for item in detections
            if isinstance(item, dict)
            and str(item.get("detected_class") or "").lower() in VEHICLE_ALERT_CLASSES
            and item.get("result_code") == "passed_single_frame"
        ]
        if eligible:
            return eligible

    # Backward compatibility for a result returned before the detailed list
    # was introduced.  The top-level result has already been evaluated by the
    # same detector thresholds.
    if (
        image.result_code == "passed_single_frame"
        and image.detected_class.lower() in VEHICLE_ALERT_CLASSES
    ):
        return [{
            "detected_class": image.detected_class,
            "confidence": image.confidence,
            "bbox": image.bbox if isinstance(image.bbox, dict) else {},
            "bbox_area": image.bbox_area,
            "result_code": image.result_code,
        }]
    return []


def _asset_url(asset: MediaAsset) -> str:
    media_url = settings.MEDIA_URL if str(settings.MEDIA_URL).startswith("/") else f"/{settings.MEDIA_URL}"
    path = f"{media_url.rstrip('/')}/{asset.file.name}"
    return f"{str(getattr(settings, 'PUBLIC_BASE_URL', '') or '').rstrip('/')}{path}"


def _persist_photo_alert_snapshot(image: BicycleDetectionTestImage, event: InspectionEvent) -> MediaAsset:
    """Copy transient diagnostic media into the normal, durable event store."""
    source = image.annotated_file or image.source_file
    if not source:
        raise ValueError("照片诊断没有可归档的图片")
    source.open("rb")
    try:
        content = source.read()
    finally:
        source.close()
    if not content:
        raise ValueError("照片诊断图片为空")
    file_name = Path(source.name).name or f"photo-alert-{image.id}.jpg"
    asset = MediaAsset.objects.create(
        robot=image.run.robot,
        media_type="snapshot",
        sequence_id=f"photo-test-{image.run_id}-{image.sequence}",
        event_time=event.detected_at,
        file=ContentFile(content, name=file_name),
        sha256=hashlib.sha256(content).hexdigest(),
        file_size=len(content),
        event_id=event.event_id,
        content_type=getattr(source, "content_type", "") or "image/jpeg",
    )
    asset.url = _asset_url(asset)
    asset.save(update_fields=["url", "updated_at"])
    return asset


def create_photo_detection_alert(image: BicycleDetectionTestImage) -> tuple[InspectionEvent | None, bool]:
    """Create exactly one operator-facing event for a qualified uploaded image.

    This is intentionally separate from live video ingestion: an operator has
    supplied one still image, so there is no three-frame confirmation or
    ten-second merge window.  The OneToOne relation makes report retries
    idempotent.
    """
    if image.alert_event_id:
        return image.alert_event, False
    eligible = _eligible_detections(image)
    if not eligible:
        return None, False

    primary = max(eligible, key=lambda item: float(item.get("confidence") or 0))
    bbox = primary.get("bbox") if isinstance(primary.get("bbox"), dict) else image.bbox
    bbox = bbox if isinstance(bbox, dict) else {}
    object_class = str(primary.get("detected_class") or image.detected_class).lower()
    confidence = Decimal(str(round(float(primary.get("confidence") or image.confidence or 0) * 100, 2)))
    diagnostics = image.diagnostics if isinstance(image.diagnostics, dict) else {}
    event = InspectionEvent.objects.create(
        robot=image.run.robot,
        title=PHOTO_ALERT_TITLE,
        event_type=PHOTO_ALERT_EVENT_TYPE,
        location=image.run.robot.location or image.run.robot.area or "未知区域",
        confidence=confidence,
        risk_level="medium",
        status="pending",
        description=f"事件中心上传图片单帧识别；实际类别：{object_class}",
        object_class=object_class,
        bbox_x=bbox.get("x"),
        bbox_y=bbox.get("y"),
        bbox_width=bbox.get("width"),
        bbox_height=bbox.get("height"),
        raw_detection={
            "source": "event_center_photo_test",
            "run_id": str(image.run_id),
            "image_id": str(image.id),
            "original_name": image.original_name,
            "primary_detection": primary,
            "qualified_detections": eligible,
            "diagnostics": diagnostics,
            "alert_aggregation": {"confirmed_frames": 1, "merged_reports": 1, "cooldown_seconds": 0},
        },
        source_component=PHOTO_ALERT_SOURCE_COMPONENT,
        source_code=PHOTO_ALERT_SOURCE_CODE,
    )
    asset = _persist_photo_alert_snapshot(image, event)
    event.snapshot_asset = asset
    event.snapshot_url = asset.url
    event.save(update_fields=["snapshot_asset", "snapshot_url", "updated_at"])
    image.alert_event = event
    image.save(update_fields=["alert_event", "updated_at"])
    robot = image.run.robot
    robot.today_alerts += 1
    robot.save(update_fields=["today_alerts", "updated_at"])
    return event, True


def purge_expired_bicycle_detection_tests(*, now=None) -> int:
    """Delete expired diagnostic media and their short-lived database records."""
    now = now or timezone.now()
    runs = BicycleDetectionTestRun.objects.filter(expires_at__lte=now).prefetch_related("images")
    deleted = 0
    for run in runs:
        for image in run.images.all():
            if image.source_file:
                image.source_file.delete(save=False)
            if image.annotated_file:
                image.annotated_file.delete(save=False)
        run.delete()
        deleted += 1
    return deleted


def expire_stalled_bicycle_detection_tests(*, now=None, timeout_seconds: int = 120) -> int:
    """Close jobs that an offline/broken device has not completed in time."""
    now = now or timezone.now()
    cutoff = now - timezone.timedelta(seconds=max(1, int(timeout_seconds)))
    runs = BicycleDetectionTestRun.objects.filter(
        status__in=["queued", "running"], created_at__lte=cutoff, expires_at__gt=now
    ).prefetch_related("images")
    expired = 0
    for run in runs:
        run.images.filter(status__in=["queued", "running"]).update(
            status="failed",
            result_code="failed",
            error_message="等待机器狗照片诊断超时（120 秒）",
            finished_at=now,
        )
        run.status = "failed"
        run.error_message = "等待机器狗照片诊断超时（120 秒）"
        run.finished_at = now
        run.save(update_fields=["status", "error_message", "finished_at", "updated_at"])
        expired += 1
    return expired
