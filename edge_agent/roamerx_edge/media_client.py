from __future__ import annotations

import hashlib
from pathlib import Path
import json

import requests

from .config import MediaConfig


class MediaClient:
    def __init__(self, config: MediaConfig, robot_id: str) -> None:
        self.config = config
        self.robot_id = robot_id

    def upload_snapshot(self, path: str, event_id: str, task_execution_id: str | None = None) -> dict:
        return self._upload(path, "snapshot", event_id, task_execution_id)

    def upload_clip(self, path: str, event_id: str, task_execution_id: str | None = None) -> dict:
        return self._upload(path, "clip", event_id, task_execution_id)

    def upload_map_package(self, path: str, metadata: dict) -> dict:
        if not self.config.map_upload_url:
            raise RuntimeError("media.map_upload_url is not configured")
        file_path = Path(path)
        headers = {"X-Device-Id": self.config.device_id, "X-Device-Key": self.config.device_key}
        data = {
            "robot_code": self.robot_id,
            "map_name": metadata.get("map_name", ""),
            "metadata": json.dumps(metadata, ensure_ascii=False),
        }
        with file_path.open("rb") as stream:
            response = requests.post(
                self.config.map_upload_url,
                data=data,
                files={"map_package": (file_path.name, stream, "application/zip")},
                headers=headers,
                timeout=60,
            )
        response.raise_for_status()
        return response.json()

    def _upload(self, path: str, media_type: str, event_id: str, task_execution_id: str | None) -> dict:
        file_path = Path(path)
        digest = hashlib.sha256(file_path.read_bytes()).hexdigest()
        data = {
            "robot_code": self.robot_id,
            "media_type": media_type,
            "event_id": event_id,
            "sha256": digest,
        }
        if task_execution_id:
            data["task_execution_id"] = task_execution_id
        headers = {"X-Device-Id": self.config.device_id, "X-Device-Key": self.config.device_key}
        with file_path.open("rb") as stream:
            response = requests.post(
                self.config.upload_url,
                data=data,
                files={"file": (file_path.name, stream)},
                headers=headers,
                timeout=15,
            )
        response.raise_for_status()
        return response.json()
