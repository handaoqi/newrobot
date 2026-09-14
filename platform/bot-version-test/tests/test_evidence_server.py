import threading
import time
from urllib.error import HTTPError
from urllib.request import urlopen

import numpy as np
import pytest

from bike_bot.config import EvidenceConfig
from bike_bot.evidence import EvidenceServer


class FakeCapture:
    def __init__(self, sample):
        self.sample = sample

    def latest_sample(self, *, max_age_seconds):
        return self.sample


def test_evidence_server_returns_latest_front_jpeg():
    sample = type("Sample", (), {
        "frame": np.zeros((24, 32, 3), dtype=np.uint8),
        "captured_at_unix": time.time(),
        "age_seconds": 0.025,
    })()
    server = EvidenceServer(
        EvidenceConfig(enabled=True, host="127.0.0.1", port=0),
        FakeCapture(sample),
        "front",
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with urlopen(
            f"http://127.0.0.1:{server.httpd.server_port}/v1/snapshot/latest",
            timeout=2,
        ) as response:
            assert response.status == 200
            assert response.headers["Content-Type"] == "image/jpeg"
            assert response.headers["X-Camera-Id"] == "front"
            assert response.read().startswith(b"\xff\xd8")
    finally:
        server.shutdown()
        thread.join(timeout=2)


def test_evidence_server_rejects_missing_or_stale_frame():
    server = EvidenceServer(
        EvidenceConfig(enabled=True, host="127.0.0.1", port=0),
        FakeCapture(None),
        "front",
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with pytest.raises(HTTPError) as exc:
            urlopen(
                f"http://127.0.0.1:{server.httpd.server_port}/v1/snapshot/latest",
                timeout=2,
            )
        assert exc.value.code == 503
    finally:
        server.shutdown()
        thread.join(timeout=2)


def test_evidence_server_refuses_non_loopback_bind():
    with pytest.raises(ValueError, match="loopback"):
        EvidenceServer(EvidenceConfig(host="0.0.0.0", port=0), FakeCapture(None), "front")
