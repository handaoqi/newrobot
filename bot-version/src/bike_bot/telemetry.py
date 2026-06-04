from __future__ import annotations

import json
import logging
import mimetypes
import time
from datetime import datetime
from itertools import count
from pathlib import Path
from threading import Lock
from uuid import uuid4

import requests

from .config import AppConfig
from .logging_utils import rotate_file_if_needed
from .models import DetectionPayload, TelemetryPayload, VideoInfo, now_iso
from .runtime import RuntimeState
from .sdk import RobotSdkClient

LOGGER = logging.getLogger(__name__)


def _milliseconds(seconds: float) -> float:
    return round(seconds * 1000, 1)


class SequenceGenerator:
    def __init__(self, robot_code: str) -> None:
        self.robot_code = robot_code
        self._counter = count(1)
        self._lock = Lock()

    def next(self) -> str:
        with self._lock:
            current = next(self._counter)
        timestamp = datetime.now().astimezone().strftime("%Y%m%d%H%M%S%f")
        return f"{self.robot_code}-{timestamp}-{current:06d}-{uuid4().hex[:8]}"


class TelemetryClient:
    def __init__(
        self,
        config: AppConfig,
        runtime_state: RuntimeState,
        sdk_client: RobotSdkClient | None = None,
    ) -> None:
        self.config = config
        self.runtime_state = runtime_state
        self.sdk_client = sdk_client
        self.sequence = SequenceGenerator(config.robot.code)
        self._log_lock = Lock()
        self._log_path = Path(config.storage.telemetry_log_path)
        self._video_lock = Lock()
        self._actual_frame_width: int | None = None
        self._actual_frame_height: int | None = None

    def update_frame_size(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        with self._video_lock:
            self._actual_frame_width = width
            self._actual_frame_height = height

    def build_payload(self, detections: list[DetectionPayload] | None = None) -> TelemetryPayload:
        self._refresh_sdk_status()
        snapshot = self.runtime_state.snapshot()
        stream_id = self.config.video.stream_id or f"dog_{self.config.robot.code}_{self.config.video.camera_id}"
        with self._video_lock:
            frame_width = self._actual_frame_width or self.config.video.width
            frame_height = self._actual_frame_height or self.config.video.height
        return TelemetryPayload(
            sequence_id=self.sequence.next(),
            robot_code=self.config.robot.code,
            robot_name=self.config.robot.name,
            reported_at=now_iso(),
            position=snapshot.position,
            motion=snapshot.motion,
            power=snapshot.power,
            network=snapshot.network,
            runtime=snapshot.runtime,
            video=VideoInfo(
                stream_id=stream_id,
                camera_id=self.config.video.camera_id,
                frame_width=frame_width,
                frame_height=frame_height,
                play_urls=self.config.video.play_urls,
            ),
            detections=detections or [],
        )

    def _refresh_sdk_status(self) -> None:
        if self.sdk_client is None:
            return
        if not self.sdk_client.is_remote_takeover_active:
            return

        sample = self.sdk_client.sample_status()
        self.runtime_state.update_status(
            battery_level=sample.battery_level,
            signal_strength=sample.signal_strength,
            runtime_status="online" if sample.connected else "offline",
            network_type="SDK",
        )

    def send(self, detections: list[DetectionPayload] | None = None) -> bool:
        total_started_at = time.perf_counter()
        payload = self.build_payload(detections=detections)
        upload_count, media_prepare_seconds, upload_seconds = self._upload_detection_media(
            payload.sequence_id,
            payload.detections,
        )
        payload_dict = payload.to_dict()
        headers = {
            "Content-Type": "application/json",
            "X-Device-Code": self.config.robot.code,
            "X-Timestamp": payload.reported_at,
        }
        if self.config.telemetry.device_key:
            headers["X-Device-Key"] = self.config.telemetry.device_key

        try:
            post_started_at = time.perf_counter()
            response = requests.post(
                self.config.telemetry.endpoint,
                json=payload_dict,
                headers=headers,
                timeout=self.config.telemetry.timeout_seconds,
                verify=self.config.telemetry.verify_tls,
            )
            post_seconds = time.perf_counter() - post_started_at
            response.raise_for_status()
            self._write_log(payload_dict, True, response.status_code, None)
            total_seconds = time.perf_counter() - total_started_at
            if payload.detections:
                LOGGER.info(
                    "edge_perf telemetry_sent sequence_id=%s detections=%d upload_count=%d "
                    "media_prepare_ms=%.1f upload_ms=%.1f post_ms=%.1f total_ms=%.1f status_code=%s",
                    payload.sequence_id,
                    len(payload.detections),
                    upload_count,
                    _milliseconds(media_prepare_seconds),
                    _milliseconds(upload_seconds),
                    _milliseconds(post_seconds),
                    _milliseconds(total_seconds),
                    response.status_code,
                )
            elif total_seconds >= 1.0:
                LOGGER.warning(
                    "edge_perf telemetry_slow_no_detection sequence_id=%s post_ms=%.1f total_ms=%.1f status_code=%s",
                    payload.sequence_id,
                    _milliseconds(post_seconds),
                    _milliseconds(total_seconds),
                    response.status_code,
                )
            LOGGER.info("telemetry sent sequence_id=%s detections=%d", payload.sequence_id, len(payload.detections))
            return True
        except requests.RequestException as exc:
            total_seconds = time.perf_counter() - total_started_at
            status_code = getattr(getattr(exc, "response", None), "status_code", None)
            self._write_log(payload_dict, False, status_code, str(exc))
            LOGGER.warning(
                "edge_perf telemetry_failed sequence_id=%s detections=%d upload_count=%d upload_ms=%.1f "
                "media_prepare_ms=%.1f total_ms=%.1f status_code=%s error=%s",
                payload.sequence_id,
                len(payload.detections),
                upload_count,
                _milliseconds(upload_seconds),
                _milliseconds(media_prepare_seconds),
                _milliseconds(total_seconds),
                status_code,
                exc,
            )
            return False

    def _upload_detection_media(self, sequence_id: str, detections: list[DetectionPayload]) -> tuple[int, float, float]:
        endpoint = self.config.telemetry.media_upload_endpoint
        if not endpoint:
            return 0, 0.0, 0.0

        upload_count = 0
        media_prepare_seconds = 0.0
        upload_seconds = 0.0
        for detection in detections:
            if not detection.local_snapshot_path:
                continue

            snapshot_path = Path(detection.local_snapshot_path)
            if not snapshot_path.exists():
                LOGGER.warning("snapshot not found, skip upload: %s", snapshot_path)
                continue

            prepare_started_at = time.perf_counter()
            sha256 = self._sha256_file(snapshot_path)
            mime_type = mimetypes.guess_type(snapshot_path.name)[0] or "image/jpeg"
            headers = {
                "X-Device-Code": self.config.robot.code,
                "X-Timestamp": now_iso(),
            }
            if self.config.telemetry.device_key:
                headers["X-Device-Key"] = self.config.telemetry.device_key

            data = {
                "robot_code": self.config.robot.code,
                "camera_id": self.config.video.camera_id,
                "media_type": "snapshot",
                "event_time": detection.event_time,
                "sequence_id": sequence_id,
                "sha256": sha256,
            }
            prepare_elapsed = time.perf_counter() - prepare_started_at
            media_prepare_seconds += prepare_elapsed
            upload_started_at = time.perf_counter()
            try:
                with snapshot_path.open("rb") as handle:
                    response = requests.post(
                        endpoint,
                        data=data,
                        files={"file": (snapshot_path.name, handle, mime_type)},
                        headers=headers,
                        timeout=self.config.telemetry.timeout_seconds,
                        verify=self.config.telemetry.verify_tls,
                    )
                elapsed = time.perf_counter() - upload_started_at
                upload_seconds += elapsed
                response.raise_for_status()
                detection.snapshot_url = response.json().get("url") or detection.snapshot_url
                upload_count += 1
                LOGGER.info(
                    "edge_perf snapshot_uploaded sequence_id=%s path=%s media_prepare_ms=%.1f "
                    "upload_ms=%.1f status_code=%s",
                    sequence_id,
                    snapshot_path,
                    _milliseconds(prepare_elapsed),
                    _milliseconds(elapsed),
                    response.status_code,
                )
            except requests.RequestException as exc:
                elapsed = time.perf_counter() - upload_started_at
                upload_seconds += elapsed
                LOGGER.warning("snapshot upload failed path=%s error=%s", snapshot_path, exc)
        return upload_count, media_prepare_seconds, upload_seconds

    @staticmethod
    def _sha256_file(path: Path) -> str:
        import hashlib

        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def _write_log(
        self,
        payload: dict,
        success: bool,
        status_code: int | None,
        error: str | None,
    ) -> None:
        entry = {
            "logged_at": now_iso(),
            "success": success,
            "status_code": status_code,
            "error": error,
            "payload": payload,
        }
        with self._log_lock:
            rotate_file_if_needed(
                self._log_path,
                self.config.storage.telemetry_log_max_bytes,
                self.config.storage.telemetry_log_backup_count,
            )
            with self._log_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(entry, ensure_ascii=False) + "\n")
