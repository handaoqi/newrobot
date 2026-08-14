import json
import pytest

from roamerx_dev_agent.config import VoiceConfig
from roamerx_dev_agent.local_asr_client import LocalASRClient, LocalASRError
from roamerx_dev_agent.voice_listener import VoiceCommandListener


class FakeResponse:
    def __init__(self, payload):
        self.payload = payload

    def read(self):
        return json.dumps(self.payload, ensure_ascii=False).encode("utf-8")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


def test_transcribe_posts_16khz_wav_and_returns_text(monkeypatch):
    captured = {}

    def fake_urlopen(request, timeout):
        captured["content_type"] = request.headers["Content-type"]
        captured["body"] = request.data
        captured["timeout"] = timeout
        return FakeResponse({"text": "小太阳检查导航"})

    monkeypatch.setattr(
        "roamerx_dev_agent.local_asr_client._LOCAL_ONLY_OPENER",
        type("LocalOnlyOpener", (), {"open": staticmethod(fake_urlopen)})(),
    )
    client = LocalASRClient(VoiceConfig(local_asr_timeout_seconds=9))

    assert client.transcribe(b"\x00\x00" * 160) == "小太阳检查导航"
    assert "multipart/form-data" in captured["content_type"]
    assert b"RIFF" in captured["body"]
    assert b'name="model"' in captured["body"]
    assert captured["timeout"] == 9


def test_transcribe_rejects_invalid_response(monkeypatch):
    monkeypatch.setattr(
        "roamerx_dev_agent.local_asr_client._LOCAL_ONLY_OPENER",
        type("LocalOnlyOpener", (), {
            "open": staticmethod(lambda *_args, **_kwargs: FakeResponse(["not-a-dict"]))
        })(),
    )
    client = LocalASRClient(VoiceConfig())

    with pytest.raises(LocalASRError, match="invalid payload"):
        client.transcribe(b"\x00\x00" * 160)


def test_listener_marks_successful_local_transcript():
    published = []
    listener = VoiceCommandListener(VoiceConfig(), published.append)
    listener._local_asr = type("LocalASR", (), {"transcribe": lambda _self, _pcm: "小太阳检查导航"})()

    listener._publish(b"\x00\x00" * 160)

    assert published[0]["asr_engine"] == "nx-sensevoice"
    assert published[0]["transcript"] == "小太阳检查导航"


def test_listener_discards_empty_local_asr_result():
    published = []
    listener = VoiceCommandListener(VoiceConfig(), published.append)
    listener._local_asr = type("LocalASR", (), {"transcribe": lambda _self, _pcm: ""})()

    listener._publish(b"\x00\x00" * 160)

    assert published == []
