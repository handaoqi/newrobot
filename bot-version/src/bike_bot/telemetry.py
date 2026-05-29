from __future__ import annotations

import json
import logging
import mimetypes
from datetime import datetime
from itertools import count
from pathlib import Path
from threading import Lock
from uuid import uuid4

import requests

from .config import AppConfig
from .models import DetectionPayload, TelemetryPayload, VideoInfo, now_iso
from .runtime import RuntimeState

LOGGER = logging.getLogger(__name__)


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
    def __init__(self, config: AppConfig, runtime_state: RuntimeState) -> None:
        self.config = config
        self.runtime_state = runtime_state
        self.sequence = SequenceGenerator(config.robot.code)
        self._log_lock = Lock()
        self._log_path = Path(config.storage.telemetry_log_path)

    def build_payload(self, detections: list[DetectionPayload] | None = None) -> TelemetryPayload:
        snapshot = self.runtime_state.snapshot()
        stream_id = self.config.video.stream_id or f"dog_{self.config.robot.code}_{self.config.video.camera_id}"
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
                frame_width=self.config.video.width,
                frame_height=self.config.video.height,
                play_urls=self.config.video.play_urls,
            ),
            detections=detections or [],
        )

    def send(self, detections: list[DetectionPayload] | None = None) -> bool:
        payload = self.build_payload(detections=detections)
        self._upload_detection_media(payload.sequence_id, payload.detections)
        payload_dict = payload.to_dict()
        headers = {
            "Content-Type": "application/json",
            "X-Device-Code": self.config.robot.code,
            "X-Timestamp": payload.reported_at,
        }
        if self.config.telemetry.device_key:
            headers["X-Device-Key"] = self.config.telemetry.device_key

        try:
            response = requests.post(
                self.config.telemetry.endpoint,
                json=payload_dict,
                headers=headers,
                timeout=self.config.telemetry.timeout_seconds,
                verify=self.config.telemetry.verify_tls,
            )
            response.raise_for_status()
            self._write_log(payload_dict, True, response.status_code, None)
            LOGGER.info("telemetry sent sequence_id=%s detections=%d", payload.sequence_id, len(payload.detections))
            return True
        except requests.RequestException as exc:
            status_code = getattr(getattr(exc, "response", None), "status_code", None)
            self._write_log(payload_dict, False, status_code, str(exc))
            LOGGER.warning("telemetry send failed: %s", exc)
            return False

    def _upload_detection_media(self, sequence_id: str, detections: list[DetectionPayload]) -> None:
        endpoint = self.config.telemetry.media_upload_endpoint
        if not endpoint:
            return

        for detection in detections:
            if not detection.local_snapshot_path:
                continue

            snapshot_path = Path(detection.local_snapshot_path)
            if not snapshot_path.exists():
                LOGGER.warning("snapshot not found, skip upload: %s", snapshot_path)
                continue

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
                response.raise_for_status()
                detection.snapshot_url = response.json().get("url") or detection.snapshot_url
            except requests.RequestException as exc:
                LOGGER.warning("snapshot upload failed path=%s error=%s", snapshot_path, exc)

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
            with self._log_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(entry, ensure_ascii=False) + "\n")
