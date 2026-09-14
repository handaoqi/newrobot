from __future__ import annotations

from datetime import datetime, timezone
import json
import logging
from pathlib import Path
from queue import Empty, Queue
import threading
import time

import requests

from .media_client import MediaClient

LOGGER = logging.getLogger(__name__)
_DIRECT = {"http": None, "https": None}


class ObstacleEvidenceManager:
    """Capture one front-camera image per obstacle episode and upload it safely."""

    def __init__(self, config, media_client: MediaClient) -> None:
        self.config = config
        self.media_client = media_client
        self.spool_dir = Path(
            getattr(
                config,
                "evidence_spool_dir",
                "/home/dogrobot/runtime/nx-edge/data/edge-agent/obstacle-evidence",
            )
        )
        self._queue: Queue[dict] = Queue()
        self._scheduled: set[str] = set()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if not bool(getattr(self.config, "evidence_enabled", True)):
            return
        self.spool_dir.mkdir(parents=True, exist_ok=True)
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="obstacle-evidence",
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._thread = None

    def schedule(self, payload: dict) -> None:
        if not bool(getattr(self.config, "evidence_enabled", True)):
            return
        event_id = str(payload.get("event_id") or "").strip()
        if not event_id:
            return
        with self._lock:
            if event_id in self._scheduled:
                return
            self._scheduled.add(event_id)
        self._queue.put(dict(payload))

    def _run(self) -> None:
        next_spool_retry = 0.0
        while not self._stop.is_set():
            try:
                payload = self._queue.get(timeout=0.5)
            except Empty:
                payload = None
            if payload is not None:
                try:
                    self._capture_and_upload(payload)
                except Exception:
                    LOGGER.exception("obstacle evidence job failed")
            now = time.monotonic()
            if now >= next_spool_retry:
                try:
                    self._retry_spooled()
                except Exception:
                    LOGGER.exception("obstacle evidence spool retry failed")
                next_spool_retry = now + max(
                    5.0, float(getattr(self.config, "evidence_retry_seconds", 30.0))
                )

    def _capture_and_upload(self, payload: dict) -> None:
        event_id = str(payload["event_id"])
        image_path = self.spool_dir / f"{event_id}.jpg"
        metadata_path = self.spool_dir / f"{event_id}.json"
        if not image_path.exists():
            capture = self._capture_latest()
            if capture is None:
                LOGGER.error(
                    "obstacle evidence unavailable event_id=%s episode=%s",
                    event_id,
                    payload.get("obstacle_episode_id"),
                )
                return
            body, camera_id, captured_at, frame_age_ms = capture
            temporary = image_path.with_suffix(".jpg.tmp")
            temporary.write_bytes(body)
            temporary.replace(image_path)
            metadata = {
                "event_id": event_id,
                "task_execution_id": payload.get("task_execution_id"),
                "obstacle_episode_id": payload.get("obstacle_episode_id"),
                "camera_id": camera_id,
                "captured_at": captured_at,
                "frame_age_ms": frame_age_ms,
            }
            metadata_temporary = metadata_path.with_suffix(".json.tmp")
            metadata_temporary.write_text(
                json.dumps(metadata, ensure_ascii=False), encoding="utf-8"
            )
            metadata_temporary.replace(metadata_path)
        self._upload_spooled(image_path, metadata_path)

    def _capture_latest(self) -> tuple[bytes, str, str, float] | None:
        url = str(
            getattr(
                self.config,
                "evidence_snapshot_url",
                "http://127.0.0.1:9101/v1/snapshot/latest",
            )
        )
        timeout = max(0.2, float(getattr(self.config, "evidence_timeout_seconds", 2.0)))
        for retry in range(3):
            try:
                response = requests.get(url, timeout=timeout, proxies=_DIRECT)
                response.raise_for_status()
                if not str(response.headers.get("Content-Type") or "").lower().startswith("image/jpeg"):
                    raise ValueError("evidence endpoint did not return image/jpeg")
                captured_unix = float(response.headers.get("X-Captured-At-Unix") or time.time())
                captured_at = datetime.fromtimestamp(captured_unix, tz=timezone.utc).isoformat()
                frame_age_ms = float(response.headers.get("X-Frame-Age-Ms") or 0.0)
                return (
                    response.content,
                    str(response.headers.get("X-Camera-Id") or "front"),
                    captured_at,
                    frame_age_ms,
                )
            except (requests.RequestException, TypeError, ValueError):
                if retry == 2:
                    LOGGER.warning("front evidence capture failed", exc_info=True)
                    break
                if self._stop.wait(0.25):
                    break
        return None

    def _retry_spooled(self) -> None:
        if not self.spool_dir.exists():
            return
        for metadata_path in sorted(self.spool_dir.glob("*.json")):
            if self._stop.is_set():
                return
            image_path = metadata_path.with_suffix(".jpg")
            if image_path.exists():
                self._upload_spooled(image_path, metadata_path)

    def _upload_spooled(self, image_path: Path, metadata_path: Path) -> None:
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            self.media_client.upload_snapshot(
                str(image_path),
                str(metadata["event_id"]),
                str(metadata.get("task_execution_id") or "") or None,
                camera_id=str(metadata.get("camera_id") or "front"),
                sequence_id=str(metadata.get("obstacle_episode_id") or ""),
                event_time=str(metadata.get("captured_at") or "") or None,
            )
        except Exception:
            LOGGER.warning("obstacle evidence upload deferred path=%s", image_path, exc_info=True)
            return
        image_path.unlink(missing_ok=True)
        metadata_path.unlink(missing_ok=True)
        LOGGER.info("obstacle evidence uploaded event_id=%s", metadata.get("event_id"))
