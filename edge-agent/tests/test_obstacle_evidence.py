from types import SimpleNamespace

from roamerx_edge.obstacle_evidence import ObstacleEvidenceManager


class FakeResponse:
    content = b"jpeg-body"
    headers = {
        "Content-Type": "image/jpeg",
        "X-Camera-Id": "front",
        "X-Captured-At-Unix": "1789344000.0",
        "X-Frame-Age-Ms": "25.0",
    }

    def raise_for_status(self):
        return None


class FakeMediaClient:
    def __init__(self):
        self.uploads = []
        self.fail = False

    def upload_snapshot(self, *args, **kwargs):
        self.uploads.append((args, kwargs))
        if self.fail:
            raise RuntimeError("cloud unavailable")
        return {"media_id": "media-1"}


def config(tmp_path):
    return SimpleNamespace(
        evidence_enabled=True,
        evidence_snapshot_url="http://127.0.0.1:9101/v1/snapshot/latest",
        evidence_timeout_seconds=2.0,
        evidence_retry_seconds=30.0,
        evidence_spool_dir=str(tmp_path),
    )


def test_capture_uploads_one_snapshot_and_cleans_spool(tmp_path, monkeypatch):
    media = FakeMediaClient()
    manager = ObstacleEvidenceManager(config(tmp_path), media)
    monkeypatch.setattr("roamerx_edge.obstacle_evidence.requests.get", lambda *args, **kwargs: FakeResponse())
    payload = {
        "event_id": "7daec1bc-d466-4f14-bfeb-35f47ad02361",
        "task_execution_id": "aa17b030-c4f0-4faf-94a8-195d7ee09f74",
        "obstacle_episode_id": "episode-1",
    }

    manager._capture_and_upload(payload)

    assert len(media.uploads) == 1
    args, kwargs = media.uploads[0]
    assert args[1:] == (payload["event_id"], payload["task_execution_id"])
    assert kwargs["camera_id"] == "front"
    assert kwargs["sequence_id"] == "episode-1"
    assert not list(tmp_path.iterdir())


def test_schedule_deduplicates_same_episode_event(tmp_path):
    manager = ObstacleEvidenceManager(config(tmp_path), FakeMediaClient())
    payload = {"event_id": "7daec1bc-d466-4f14-bfeb-35f47ad02361"}

    manager.schedule(payload)
    manager.schedule(payload)

    assert manager._queue.qsize() == 1


def test_failed_upload_keeps_snapshot_for_later_retry(tmp_path, monkeypatch):
    media = FakeMediaClient()
    media.fail = True
    manager = ObstacleEvidenceManager(config(tmp_path), media)
    monkeypatch.setattr("roamerx_edge.obstacle_evidence.requests.get", lambda *args, **kwargs: FakeResponse())
    payload = {
        "event_id": "7daec1bc-d466-4f14-bfeb-35f47ad02361",
        "task_execution_id": "aa17b030-c4f0-4faf-94a8-195d7ee09f74",
        "obstacle_episode_id": "episode-1",
    }

    manager._capture_and_upload(payload)
    assert (tmp_path / f"{payload['event_id']}.jpg").exists()
    assert (tmp_path / f"{payload['event_id']}.json").exists()

    media.fail = False
    manager._retry_spooled()
    assert len(media.uploads) == 2
    assert not list(tmp_path.iterdir())
