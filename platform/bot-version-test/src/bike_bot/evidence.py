from __future__ import annotations

import logging
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import cv2

from .config import EvidenceConfig

LOGGER = logging.getLogger(__name__)


class EvidenceRequestHandler(BaseHTTPRequestHandler):
    server_version = "BikeBotEvidenceServer/1.0"

    def do_GET(self) -> None:
        if self.path.split("?", 1)[0].rstrip("/") != "/v1/snapshot/latest":
            self.send_error(404, "not found")
            return
        config: EvidenceConfig = self.server.evidence_config  # type: ignore[attr-defined]
        capture = self.server.latest_capture  # type: ignore[attr-defined]
        camera_id: str = self.server.camera_id  # type: ignore[attr-defined]
        sample = capture.latest_sample(max_age_seconds=config.max_frame_age_seconds)
        if sample is None:
            self.send_error(503, "fresh frame unavailable")
            return
        ok, encoded = cv2.imencode(
            ".jpg",
            sample.frame,
            [cv2.IMWRITE_JPEG_QUALITY, max(1, min(100, int(config.jpeg_quality)))],
        )
        if not ok:
            self.send_error(500, "jpeg encode failed")
            return
        body = encoded.tobytes()
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Camera-Id", camera_id)
        self.send_header("X-Captured-At-Unix", f"{sample.captured_at_unix:.6f}")
        self.send_header("X-Frame-Age-Ms", f"{sample.age_seconds * 1000.0:.1f}")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: Any) -> None:
        LOGGER.debug("evidence_server " + format, *args)


class EvidenceServer:
    def __init__(self, config: EvidenceConfig, latest_capture, camera_id: str) -> None:
        if config.host not in {"127.0.0.1", "localhost", "::1"}:
            raise ValueError("evidence server must bind to loopback")
        self.config = config
        self.httpd = ThreadingHTTPServer((config.host, config.port), EvidenceRequestHandler)
        self.httpd.evidence_config = config
        self.httpd.latest_capture = latest_capture
        self.httpd.camera_id = camera_id

    def serve_forever(self) -> None:
        LOGGER.info("starting evidence server host=%s port=%s", self.config.host, self.config.port)
        self.httpd.serve_forever(poll_interval=0.5)

    def shutdown(self) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()
