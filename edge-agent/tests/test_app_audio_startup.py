from __future__ import annotations

import threading

from roamerx_edge.app import EdgeAgentApplication


class _AudioControl:
    def __init__(self, failures: dict[str, int] | None = None) -> None:
        self.failures = dict(failures or {})
        self.calls: list[tuple[str, int]] = []

    def set_volume(self, target: str, volume: int) -> dict:
        self.calls.append((target, volume))
        if self.failures.get(target, 0):
            self.failures[target] -= 1
            raise RuntimeError("audio service is not ready")
        return {"target": target, "volume": volume}


def _bare_application(audio_control: _AudioControl) -> EdgeAgentApplication:
    application = object.__new__(EdgeAgentApplication)
    application.stop_event = threading.Event()
    application.audio_control_adapter = audio_control
    return application


def test_boot_speaker_volume_sets_both_speakers_to_100(monkeypatch):
    monkeypatch.setattr("roamerx_edge.app.BOOT_SPEAKER_RETRY_DELAYS_SECONDS", (0.0,))
    audio_control = _AudioControl()

    _bare_application(audio_control)._set_boot_speaker_volumes()

    assert set(audio_control.calls) == {("speaker_nx", 100), ("speaker_3588", 100)}


def test_boot_speaker_volume_retries_only_the_speaker_that_is_not_ready(monkeypatch):
    monkeypatch.setattr("roamerx_edge.app.BOOT_SPEAKER_RETRY_DELAYS_SECONDS", (0.0, 0.0))
    audio_control = _AudioControl({"speaker_3588": 1})

    _bare_application(audio_control)._set_boot_speaker_volumes()

    assert audio_control.calls.count(("speaker_nx", 100)) == 1
    assert audio_control.calls.count(("speaker_3588", 100)) == 2
