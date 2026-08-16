import json
from collections import deque
import pytest

from roamerx_dev_agent.config import VoiceConfig
from roamerx_dev_agent.local_asr_client import LocalASRClient, LocalASRError
from roamerx_dev_agent.voice_ack_player import VoiceAckPlayer
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
    listener._local_asr = type(
        "LocalASR", (), {"transcribe": lambda _self, _pcm, **_kwargs: "小太阳检查导航"}
    )()

    listener._publish(b"\x00\x00" * 160)

    assert published[0]["asr_engine"] == "nx-sensevoice"
    assert published[0]["transcript"] == "小太阳检查导航"


def test_listener_publishes_empty_local_asr_result_as_diagnostic_event():
    published = []
    listener = VoiceCommandListener(VoiceConfig(), published.append)
    listener._local_asr = type("LocalASR", (), {"transcribe": lambda _self, _pcm, **_kwargs: ""})()

    listener._publish(b"\x00\x00" * 160)

    assert published[0]["asr_engine"] == "nx-sensevoice"
    assert published[0]["transcript"] == ""


def test_listener_uses_learned_noise_floor_before_opening_a_segment():
    listener = VoiceCommandListener(
        VoiceConfig(rms_threshold=18, noise_floor_chunks=8, noise_rms_multiplier=2.2),
        lambda _payload: None,
    )

    for _ in range(4):
        loud, _threshold = listener._is_loud(17)
        assert not loud

    loud, threshold = listener._is_loud(30)
    assert threshold >= 37
    assert not loud
    loud, _threshold = listener._is_loud(60)
    assert loud


def test_listener_keeps_idle_audio_before_a_speech_segment():
    listener = VoiceCommandListener(VoiceConfig(pre_roll_chunks=2), lambda _payload: None)
    first = b"a" * listener._CHUNK_BYTES
    second = b"b" * listener._CHUNK_BYTES
    current = b"c" * listener._CHUNK_BYTES
    pre_roll = deque([first, second], maxlen=2)

    speech, chunks = listener._open_segment(pre_roll, current)

    assert chunks == 3
    assert bytes(speech) == first + second + current


def test_local_alsa_ack_plays_on_nx_usb_audio_device():
    command, options = VoiceAckPlayer(
        VoiceConfig(
            capture_mode="local_alsa",
            alsa_device="dsnoop:CARD=Device,DEV=0",
            playback_device="hw:0,0",
        )
    )._command(
        "http://example.test/ack.mp3"
    )

    assert command == [
        "/usr/bin/ffplay", "-nodisp", "-autoexit", "-loglevel", "warning", "http://example.test/ack.mp3"
    ]
    assert options["env"]["SDL_AUDIODRIVER"] == "alsa"
    assert options["env"]["AUDIODEV"] == "plughw:0,0"
