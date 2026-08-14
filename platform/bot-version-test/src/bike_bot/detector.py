from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any
from uuid import uuid4

import cv2

from .config import AppConfig, ModelConfig
from .models import BoundingBox, DetectionPayload, now_iso
from .tracking import IoUTracker, TrackingDetection

LOGGER = logging.getLogger(__name__)


def letterbox(frame, image_size: int) -> tuple[Any, float, int, int]:
    height, width = frame.shape[:2]
    scale = min(image_size / width, image_size / height)
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))
    resized = cv2.resize(frame, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)

    pad_x = (image_size - resized_width) // 2
    pad_y = (image_size - resized_height) // 2
    right = image_size - resized_width - pad_x
    bottom = image_size - resized_height - pad_y
    padded = cv2.copyMakeBorder(
        resized,
        pad_y,
        bottom,
        pad_x,
        right,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    return padded, scale, pad_x, pad_y


@dataclass
class FrameEvent:
    frame: Any
    detection: DetectionPayload


@dataclass
class DetectionResult:
    events: list[FrameEvent]
    preview_frame: Any
    target_count: int = 0
    tracked_objects: list[TrackedObject] | None = None


@dataclass
class RawDetection:
    label: str
    bbox: tuple[int, int, int, int]
    confidence: float


class SnapshotManager:
    def __init__(
        self,
        directory: str,
        public_base_url: str,
        jpeg_quality: int,
        max_files: int,
        max_total_bytes: int,
    ) -> None:
        self.directory = Path(directory)
        self.public_base_url = public_base_url.rstrip("/")
        self.jpeg_quality = jpeg_quality
        self.max_files = max_files
        self.max_total_bytes = max_total_bytes

    def save(self, frame) -> tuple[str, str | None] | None:
        timestamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%f%z")
        file_name = f"event-{timestamp}-{uuid4().hex[:8]}.jpg"
        path = self.directory / file_name
        ok = cv2.imwrite(str(path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return None
        self.cleanup()
        if self.public_base_url:
            return str(path.resolve()), f"{self.public_base_url}/{file_name}"
        return str(path.resolve()), None

    def cleanup(self) -> None:
        try:
            snapshots = sorted(
                (
                    item
                    for item in self.directory.glob("event-*.jpg")
                    if item.is_file()
                ),
                key=lambda item: item.stat().st_mtime,
                reverse=True,
            )
        except OSError as exc:
            LOGGER.warning("snapshot cleanup skipped: %s", exc)
            return

        total_bytes = 0
        to_remove: list[Path] = []
        for index, snapshot in enumerate(snapshots):
            try:
                size = snapshot.stat().st_size
            except OSError:
                continue
            if index == 0:
                total_bytes += size
                continue
            keep_by_count = self.max_files <= 0 or index < self.max_files
            keep_by_size = self.max_total_bytes <= 0 or total_bytes + size <= self.max_total_bytes
            if keep_by_count and keep_by_size:
                total_bytes += size
                continue
            to_remove.append(snapshot)

        for snapshot in to_remove:
            try:
                snapshot.unlink()
            except OSError as exc:
                LOGGER.warning("failed to remove old snapshot path=%s error=%s", snapshot, exc)


class YoloDetector:
    def __init__(
        self,
        config: AppConfig,
        *,
        model_config: ModelConfig | None = None,
        target_labels: list[str] | None = None,
        event_labels: list[str] | None = None,
        emit_events: bool = True,
    ) -> None:
        self.config = config
        self.model_config = model_config or config.model
        classes = self.model_config.classes or ["bicycle"]
        self.target_labels = {item.lower() for item in (target_labels or classes)}
        configured_event_labels = event_labels if event_labels is not None else (config.detection.event_classes or classes)
        self.event_labels = {item.lower() for item in configured_event_labels}
        self.emit_events = emit_events
        self.class_names = [item.lower() for item in classes]
        self.model_backend = self._resolve_backend(self.model_config.backend, self.model_config.path)
        self.model = self._load_model()
        self.snapshot_manager = SnapshotManager(
            config.snapshot.directory,
            config.snapshot.public_base_url,
            config.snapshot.jpeg_quality,
            config.snapshot.max_files,
            config.snapshot.max_total_bytes,
        )
        self.tracker = IoUTracker(
            iou_threshold=config.detection.tracker_iou_threshold,
            track_ttl_seconds=config.detection.track_ttl_seconds,
            duplicate_alert_seconds=config.detection.duplicate_alert_seconds,
        )
        self._last_event_at = 0.0

    def _resolve_backend(self, backend: str, model_path: str) -> str:
        normalized = backend.lower()
        if normalized != "auto":
            return normalized
        suffix = Path(model_path).suffix.lower()
        if suffix == ".onnx":
            return "opencv_dnn"
        return "ultralytics"

    def _load_model(self) -> Any:
        if self.model_backend == "opencv_dnn":
            try:
                net = cv2.dnn.readNetFromONNX(self.model_config.path)
                net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                LOGGER.info("loaded ONNX model with OpenCV DNN: %s", self.model_config.path)
                return net
            except cv2.error as exc:
                LOGGER.warning("OpenCV DNN failed to load ONNX, falling back to onnxruntime: %s", exc)
                self.model_backend = "onnxruntime"

        if self.model_backend == "onnxruntime":
            import onnxruntime as ort

            session_options = ort.SessionOptions()
            session_options.intra_op_num_threads = 2
            session_options.inter_op_num_threads = 1
            session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
            session = ort.InferenceSession(
                self.model_config.path,
                sess_options=session_options,
                providers=providers,
            )
            LOGGER.info(
                "loaded ONNX model with onnxruntime: %s providers=%s",
                self.model_config.path,
                session.get_providers(),
            )
            return session

        if self.model_backend == "ultralytics":
            from ultralytics import YOLO

            LOGGER.info("loaded Ultralytics model: %s", self.model_config.path)
            return YOLO(self.model_config.path)

        raise ValueError(f"unsupported model backend: {self.model_backend}")

    def open_capture(self) -> cv2.VideoCapture:
        source = self.config.video.source
        if isinstance(source, str) and source.startswith("rtsp://"):
            transport = self.config.video.rtsp_transport
            open_timeout_us = self.config.video.open_timeout_seconds * 1_000_000
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
                f"rtsp_transport;{transport}|stimeout;{open_timeout_us}"
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
        raw_detections = self._predict(frame)
        events: list[FrameEvent] = []
        preview_frame = frame.copy()
        target_detections: list[TrackingDetection] = []

        for raw in raw_detections:
            label = raw.label.lower()
            x1, y1, width, height = raw.bbox
            x2 = x1 + width
            y2 = y1 + height
            is_target = label in self.target_labels and width * height >= self.config.detection.min_box_area
            color = (0, 220, 0) if is_target else (160, 160, 160)
            cv2.rectangle(preview_frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                preview_frame,
                f"{label} {raw.confidence:.2f}",
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
                    bbox=raw.bbox,
                    label=label,
                    confidence=raw.confidence,
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
                tracked_objects=[],
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

            if not self.emit_events or track.label not in self.event_labels or not self.tracker.should_alert(track.track_id):
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
        return DetectionResult(
            events=events,
            preview_frame=preview_frame,
            target_count=len(tracked_targets),
            tracked_objects=tracked_targets,
        )

    def _predict(self, frame) -> list[RawDetection]:
        if self.model_backend == "opencv_dnn":
            return self._predict_opencv_dnn(frame)
        if self.model_backend == "onnxruntime":
            return self._predict_onnxruntime(frame)
        return self._predict_ultralytics(frame)

    def _predict_ultralytics(self, frame) -> list[RawDetection]:
        results = self.model.predict(
            frame,
            conf=self.model_config.confidence,
            imgsz=self.model_config.image_size,
            device=self.model_config.device or None,
            verbose=False,
        )
        if not results:
            return []

        result = results[0]
        names = result.names
        raw_detections: list[RawDetection] = []
        for box in result.boxes:
            class_id = int(box.cls[0].item())
            label = str(names[class_id]).lower()
            x1, y1, x2, y2 = [int(value) for value in box.xyxy[0].tolist()]
            width = max(0, x2 - x1)
            height = max(0, y2 - y1)
            confidence = float(box.conf[0].item())
            raw_detections.append(
                RawDetection(label=label, bbox=(x1, y1, width, height), confidence=confidence)
            )
        return raw_detections

    def _predict_opencv_dnn(self, frame) -> list[RawDetection]:
        input_image, scale, pad_x, pad_y = letterbox(frame, self.model_config.image_size)
        blob = cv2.dnn.blobFromImage(
            input_image,
            1 / 255.0,
            (self.model_config.image_size, self.model_config.image_size),
            swapRB=True,
            crop=False,
        )
        self.model.setInput(blob)
        outputs = self.model.forward()
        predictions = outputs[0] if isinstance(outputs, tuple) else outputs
        return self._parse_yolo_predictions(predictions, frame, scale, pad_x, pad_y)

    def _predict_onnxruntime(self, frame) -> list[RawDetection]:
        input_image, scale, pad_x, pad_y = letterbox(frame, self.model_config.image_size)
        blob = cv2.dnn.blobFromImage(
            input_image,
            1 / 255.0,
            (self.model_config.image_size, self.model_config.image_size),
            swapRB=True,
            crop=False,
        )
        input_name = self.model.get_inputs()[0].name
        output_name = self.model.get_outputs()[0].name
        predictions = self.model.run([output_name], {input_name: blob})[0]
        return self._parse_yolo_predictions(predictions, frame, scale, pad_x, pad_y)

    def _parse_yolo_predictions(self, predictions, frame, scale: float, pad_x: int, pad_y: int) -> list[RawDetection]:
        predictions = predictions.squeeze()
        if predictions.ndim != 2:
            return []
        if predictions.shape[0] < predictions.shape[1] and predictions.shape[0] <= 256:
            predictions = predictions.transpose()

        frame_h, frame_w = frame.shape[:2]
        boxes: list[list[int]] = []
        scores: list[float] = []
        class_ids: list[int] = []
        class_count = max(1, predictions.shape[1] - 4)

        for prediction in predictions:
            values = prediction.tolist()
            if len(values) < 5:
                continue
            cx, cy, box_w, box_h = values[:4]
            class_scores = values[4:]
            if class_count == 1:
                class_id = 0
                confidence = float(class_scores[0])
            else:
                class_id = max(range(len(class_scores)), key=lambda index: class_scores[index])
                confidence = float(class_scores[class_id])
            if confidence < self.model_config.confidence:
                continue

            x1 = int(round((cx - box_w / 2 - pad_x) / scale))
            y1 = int(round((cy - box_h / 2 - pad_y) / scale))
            width = int(round(box_w / scale))
            height = int(round(box_h / scale))
            x1 = max(0, min(frame_w - 1, x1))
            y1 = max(0, min(frame_h - 1, y1))
            width = max(0, min(frame_w - x1, width))
            height = max(0, min(frame_h - y1, height))
            if width <= 0 or height <= 0:
                continue
            boxes.append([x1, y1, width, height])
            scores.append(confidence)
            class_ids.append(class_id)

        keep = cv2.dnn.NMSBoxes(
            boxes,
            scores,
            self.model_config.confidence,
            self.model_config.nms_iou_threshold,
        )
        if len(keep) == 0:
            return []

        keep_indices = []
        for item in keep:
            if isinstance(item, (list, tuple)):
                item = item[0]
            elif hasattr(item, "item"):
                item = item.item()
            keep_indices.append(int(item))

        raw_detections: list[RawDetection] = []
        for index in keep_indices:
            class_id = class_ids[index]
            label = self.class_names[class_id] if class_id < len(self.class_names) else str(class_id)
            raw_detections.append(
                RawDetection(
                    label=label,
                    bbox=tuple(boxes[index]),
                    confidence=scores[index],
                )
            )
        return raw_detections

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
