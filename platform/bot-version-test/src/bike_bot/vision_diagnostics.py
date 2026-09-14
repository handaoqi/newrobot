from __future__ import annotations

import json
import logging
import os
import threading
from typing import Any

import cv2
import numpy as np
import requests

from .config import AppConfig
from .detector import YoloDetector

LOGGER = logging.getLogger(__name__)


class BicycleDiagnosticClient:
    """Device-side transport for operator image diagnostics, isolated from alerts."""

    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.api_base = self._derive_api_base(config.telemetry.endpoint)
        self.session = requests.Session()

    @staticmethod
    def _derive_api_base(endpoint: str) -> str:
        if "/api/" in endpoint:
            return endpoint.split("/api/", 1)[0].rstrip("/") + "/api"
        return endpoint.rstrip("/")

    def _headers(self) -> dict[str, str]:
        headers = {
            "X-Device-Code": self.config.robot.code,
            "X-Device-Id": os.getenv("BIKE_BOT_DEVICE_ID", self.config.robot.code),
        }
        if self.config.telemetry.device_key:
            headers["X-Device-Key"] = self.config.telemetry.device_key
        return headers

    def poll_once(self) -> dict | None:
        response = self.session.get(
            f"{self.api_base}/device/bicycle-detection-tests/poll/",
            params={"robot_code": self.config.robot.code},
            headers=self._headers(),
            timeout=self.config.telemetry.timeout_seconds,
            verify=self.config.telemetry.verify_tls,
        )
        if response.status_code == 204:
            return None
        response.raise_for_status()
        return response.json()

    def download_image(self, task: dict) -> Any:
        response = self.session.get(
            str(task["image_url"]),
            headers=self._headers(),
            timeout=self.config.telemetry.timeout_seconds,
            verify=self.config.telemetry.verify_tls,
        )
        response.raise_for_status()
        image = cv2.imdecode(np.frombuffer(response.content, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("图片解码失败")
        return image

    def report(self, task: dict, *, status: str, diagnostics: dict, annotated=None, error_message: str = "") -> None:
        endpoint = (
            f"{self.api_base}/device/bicycle-detection-tests/{task['run_id']}"
            f"/images/{task['image_id']}/report/"
        )
        data = {
            "status": status,
            "result_code": str(diagnostics.get("result_code") or ""),
            "detected_class": str(diagnostics.get("detected_class") or ""),
            "confidence": diagnostics.get("confidence") or "",
            "diagnostics": json.dumps(diagnostics, ensure_ascii=False),
            "error_message": error_message,
        }
        files = None
        if annotated is not None:
            ok, encoded = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if ok:
                files = {"annotated_image": ("annotated.jpg", encoded.tobytes(), "image/jpeg")}
        response = self.session.post(
            endpoint,
            data=data,
            files=files,
            headers=self._headers(),
            timeout=max(10, self.config.telemetry.timeout_seconds),
            verify=self.config.telemetry.verify_tls,
        )
        response.raise_for_status()

    def close(self) -> None:
        self.session.close()


def vision_diagnostic_worker(stop_event: threading.Event, detector: YoloDetector, config: AppConfig) -> None:
    client = BicycleDiagnosticClient(config)
    try:
        while not stop_event.is_set():
            task = None
            try:
                task = client.poll_once()
                if task is None:
                    stop_event.wait(1.0)
                    continue
                frame = client.download_image(task)
                diagnostics, annotated = detector.diagnose_image(frame)
                client.report(task, status="finished", diagnostics=diagnostics, annotated=annotated)
                LOGGER.info(
                    "bicycle diagnostic complete run_id=%s image_id=%s result=%s class=%s",
                    task["run_id"], task["image_id"], diagnostics.get("result_code"), diagnostics.get("detected_class", ""),
                )
            except Exception as exc:
                LOGGER.exception("bicycle diagnostic worker failed")
                if task is not None:
                    try:
                        client.report(
                            task,
                            status="failed",
                            diagnostics={"result_code": "failed"},
                            error_message=str(exc),
                        )
                    except Exception:
                        LOGGER.exception("bicycle diagnostic failure report failed")
                stop_event.wait(2.0)
    finally:
        client.close()
