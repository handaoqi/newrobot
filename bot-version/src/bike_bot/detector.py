from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
from ultralytics import YOLO

from .config import AppConfig
from .models import BoundingBox, DetectionPayload, now_iso
from .tracking import IoUTracker, TrackingDetection

LOGGER = logging.getLogger(__name__)


@dataclass
class FrameEvent:
    frame: Any
    detection: DetectionPayload


@dataclass
class DetectionResult:
    events: list[FrameEvent]
    preview_frame: Any
    target_count: int = 0


class SnapshotManager:
    def __init__(self, directory: str, public_base_url: str, jpeg_quality: int) -> None:
        self.directory = Path(directory)
        self.public_base_url = public_base_url.rstrip("/")
        self.jpeg_quality = jpeg_quality

    def save(self, frame) -> tuple[str, str | None] | None:
        timestamp = now_iso().replace(":", "").replace("+", "_")
        file_name = f"event-{timestamp}.jpg"
        path = self.directory / file_name
        ok = cv2.imwrite(str(path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return None
        if self.public_base_url:
            return str(path.resolve()), f"{self.public_base_url}/{file_name}"
        return str(path.resolve()), None


class YoloDetector:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.model = YOLO(config.model.path)
        classes = config.model.classes or ["bicycle"]
        self.target_labels = {item.lower() for item in classes}
        self.snapshot_manager = SnapshotManager(
            config.snapshot.directory,
            config.snapshot.public_base_url,
            config.snapshot.jpeg_quality,
        )
        self.tracker = IoUTracker(
            iou_threshold=config.detection.tracker_iou_threshold,
            track_ttl_seconds=config.detection.track_ttl_seconds,
            duplicate_alert_seconds=config.detection.duplicate_alert_seconds,
        )
        self._last_event_at = 0.0

    def open_capture(self) -> cv2.VideoCapture:
        source = self.config.video.source
        if isinstance(source, str) and source.startswith("rtsp://"):
            transport = self.config.video.rtsp_transport
            open_timeout_us = self.config.video.open_timeout_seconds * 1_000_000
            read_timeout_us = self.config.video.read_timeout_seconds * 1_000_000
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
                f"rtsp_transport;{transport}|stimeout;{open_timeout_us}|timeout;{read_timeout_us}"
            )
            LOGGER.info(
                "using ffmpeg rtsp options transport=%s open_timeout=%ss read_timeout=%ss",
                transport,
                self.config.video.open_timeout_seconds,
                self.config.video.read_timeout_seconds,
            )
            capture = cv2.VideoCapture(source, cv2.CAP_FFMPEG)
            if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
                capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, self.config.video.open_timeout_seconds * 1000)
            if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
                capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, self.config.video.read_timeout_seconds * 1000)
        else:
            capture = cv2.VideoCapture(source)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.video.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.video.height)
        return capture

    def should_emit_event(self) -> bool:
        now = time.time()
        if now - self._last_event_at < self.config.detection.event_cooldown_seconds:
            return False
        self._last_event_at = now
        return True

    def detect(self, frame) -> DetectionResult:
        results = self.model.predict(
            frame,
            conf=self.config.model.confidence,
            imgsz=self.config.model.image_size,
            device=self.config.model.device or None,
            verbose=False,
        )
        if not results:
            return DetectionResult(events=[], preview_frame=frame)

        result = results[0]
        names = result.names
        events: list[FrameEvent] = []
        preview_frame = frame.copy()
        target_detections: list[TrackingDetection] = []
        for box in result.boxes:
            class_id = int(box.cls[0].item())
            label = str(names[class_id]).lower()
            x1, y1, x2, y2 = [int(value) for value in box.xyxy[0].tolist()]
            width = max(0, x2 - x1)
            height = max(0, y2 - y1)
            confidence = float(box.conf[0].item())

            is_target = label in self.target_labels and width * height >= self.config.detection.min_box_area
            color = (0, 220, 0) if is_target else (160, 160, 160)
            cv2.rectangle(preview_frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                preview_frame,
                f"{label} {confidence:.2f}",
                (x1, max(25, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv2.LINE_AA,
            )

            if not is_target:
                continue

            target_detections.append(
                TrackingDetection(
                    bbox=(x1, y1, width, height),
                    label=label,
                    confidence=confidence,
                )
            )

        if not self.config.detection.tracking_enabled:
            for target in target_detections:
                if not self.should_emit_event():
                    continue
                x1, y1, width, height = target.bbox
                bbox = BoundingBox(x=x1, y=y1, width=width, height=height)
                detection = DetectionPayload(
                    type=self.config.detection.event_type,
                    label=self.config.detection.event_label,
                    confidence=round(target.confidence, 4),
                    risk_level=self.config.detection.risk_level,
                    object_class=target.label,
                    bbox=bbox,
                    event_time=now_iso(),
                )
                events.append(FrameEvent(frame=frame.copy(), detection=detection))
            return DetectionResult(
                events=events,
                preview_frame=preview_frame,
                target_count=len(target_detections),
            )

        tracked_targets = self.tracker.update(target_detections)
        for track in tracked_targets:
            x1, y1, width, height = track.bbox
            y2 = y1 + height
            cv2.putText(
                preview_frame,
                track.track_id,
                (x1, min(preview_frame.shape[0] - 10, y2 + 24)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 220, 0),
                2,
                cv2.LINE_AA,
            )

            if not self.tracker.should_alert(track.track_id):
                continue

            bbox = BoundingBox(x=x1, y=y1, width=width, height=height)
            detection = DetectionPayload(
                type=self.config.detection.event_type,
                label=self.config.detection.event_label,
                confidence=round(track.confidence, 4),
                risk_level=self.config.detection.risk_level,
                object_class=track.label,
                track_id=track.track_id,
                bbox=bbox,
                event_time=now_iso(),
            )
            events.append(FrameEvent(frame=frame.copy(), detection=detection))
        return DetectionResult(events=events, preview_frame=preview_frame, target_count=len(tracked_targets))

    def enrich_with_snapshot(self, event: FrameEvent) -> FrameEvent:
        snapshot = self.snapshot_manager.save(event.frame)
        if snapshot:
            local_path, snapshot_url = snapshot
            event.detection.local_snapshot_path = local_path
            if snapshot_url:
                event.detection.snapshot_url = snapshot_url
        return event

    def annotate_status(self, frame, *, fps: float, target_count: int) -> Any:
        annotated = frame.copy()
        lines = [f"target_count: {target_count}"]
        if self.config.display.show_fps:
            lines.append(f"fps: {fps:.2f}")

        for index, text in enumerate(lines):
            y = 30 + (index * 28)
            cv2.putText(
                annotated,
                text,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return annotated

    def show_preview(self, frame) -> bool:
        if not self.config.display.enable:
            return True

        preview_frame = frame
        width = preview_frame.shape[1]
        if width > self.config.display.max_width:
            scale = self.config.display.max_width / width
            preview_frame = cv2.resize(preview_frame, None, fx=scale, fy=scale)

        cv2.imshow(self.config.display.window_name, preview_frame)
        key = cv2.waitKey(1) & 0xFF
        return key not in (27, ord("q"))
