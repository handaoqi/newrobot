from __future__ import annotations

import argparse
import ctypes
import sys
import time
import wave
from pathlib import Path


SND_PCM_STREAM_PLAYBACK = 0
SND_PCM_ACCESS_RW_INTERLEAVED = 3
SND_PCM_FORMAT_S16_LE = 2
CLOCK_TAI = getattr(time, "CLOCK_TAI", 11)
TIMER_ABSTIME = 1


class Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]


class AlsaPlayer:
    def __init__(self, device: str, pcm_path: Path) -> None:
        with wave.open(str(pcm_path), "rb") as source:
            if (source.getsampwidth(), source.getframerate(), source.getnchannels()) != (2, 48000, 2):
                raise RuntimeError("synchronized audio must be 48kHz stereo S16_LE PCM")
            self.frames = source.readframes(source.getnframes())
        if not self.frames:
            raise RuntimeError("synchronized audio is empty")

        self.lib = ctypes.CDLL("libasound.so.2")
        self.pcm = ctypes.c_void_p()
        self._configure_signatures()
        self._check(self.lib.snd_pcm_open(ctypes.byref(self.pcm), device.encode(), SND_PCM_STREAM_PLAYBACK, 0), "open")
        self._check(
            self.lib.snd_pcm_set_params(
                self.pcm,
                SND_PCM_FORMAT_S16_LE,
                SND_PCM_ACCESS_RW_INTERLEAVED,
                2,
                48000,
                1,
                50_000,
            ),
            "configure",
        )
        self._prime()

    def _configure_signatures(self) -> None:
        self.lib.snd_pcm_open.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
        self.lib.snd_pcm_set_params.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_int, ctypes.c_uint]
        self.lib.snd_pcm_get_params.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong)]
        self.lib.snd_pcm_sw_params_malloc.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.lib.snd_pcm_sw_params_current.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.lib.snd_pcm_sw_params_set_start_threshold.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
        self.lib.snd_pcm_sw_params.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.lib.snd_pcm_writei.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
        self.lib.snd_pcm_writei.restype = ctypes.c_long
        self.lib.snd_pcm_start.argtypes = [ctypes.c_void_p]
        self.lib.snd_pcm_drain.argtypes = [ctypes.c_void_p]
        self.lib.snd_pcm_close.argtypes = [ctypes.c_void_p]
        self.lib.snd_pcm_sw_params_free.argtypes = [ctypes.c_void_p]
        self.lib.snd_strerror.argtypes = [ctypes.c_int]
        self.lib.snd_strerror.restype = ctypes.c_char_p

    def _check(self, result: int, action: str) -> None:
        if result < 0:
            detail = self.lib.snd_strerror(result).decode(errors="replace")
            raise RuntimeError(f"ALSA {action} failed: {detail}")

    def _prime(self) -> None:
        buffer_frames = ctypes.c_ulong()
        period_frames = ctypes.c_ulong()
        self._check(self.lib.snd_pcm_get_params(self.pcm, ctypes.byref(buffer_frames), ctypes.byref(period_frames)), "get params")
        sw_params = ctypes.c_void_p()
        self._check(self.lib.snd_pcm_sw_params_malloc(ctypes.byref(sw_params)), "allocate sw params")
        try:
            self._check(self.lib.snd_pcm_sw_params_current(self.pcm, sw_params), "read sw params")
            self._check(
                self.lib.snd_pcm_sw_params_set_start_threshold(self.pcm, sw_params, buffer_frames.value + 1),
                "set start threshold",
            )
            self._check(self.lib.snd_pcm_sw_params(self.pcm, sw_params), "apply sw params")
        finally:
            self.lib.snd_pcm_sw_params_free(sw_params)

        self.initial_frames = min(len(self.frames) // 4, buffer_frames.value)
        initial = ctypes.create_string_buffer(self.frames[: self.initial_frames * 4])
        written = self.lib.snd_pcm_writei(self.pcm, initial, self.initial_frames)
        self._check(int(written), "prime")
        if written != self.initial_frames:
            raise RuntimeError(f"ALSA prime was short: {written}/{self.initial_frames} frames")

    def play_at(self, target_tai_ns: int) -> int:
        libc = ctypes.CDLL(None)
        target = Timespec(target_tai_ns // 1_000_000_000, target_tai_ns % 1_000_000_000)
        result = libc.clock_nanosleep(CLOCK_TAI, TIMER_ABSTIME, ctypes.byref(target), None)
        if result:
            raise RuntimeError(f"clock_nanosleep failed: {result}")
        self._check(self.lib.snd_pcm_start(self.pcm), "start")
        actual_tai_ns = time.clock_gettime_ns(CLOCK_TAI)

        offset = self.initial_frames * 4
        while offset < len(self.frames):
            chunk = self.frames[offset : offset + 4096 * 4]
            payload = ctypes.create_string_buffer(chunk)
            frame_count = len(chunk) // 4
            written = self.lib.snd_pcm_writei(self.pcm, payload, frame_count)
            self._check(int(written), "write")
            offset += int(written) * 4
        self._check(self.lib.snd_pcm_drain(self.pcm), "drain")
        return actual_tai_ns

    def close(self) -> None:
        if self.pcm:
            self.lib.snd_pcm_close(self.pcm)
            self.pcm = ctypes.c_void_p()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", required=True)
    parser.add_argument("--file", required=True, type=Path)
    args = parser.parse_args()
    player = AlsaPlayer(args.device, args.file)
    try:
        print("READY", flush=True)
        command = sys.stdin.readline().strip().split()
        if len(command) != 2 or command[0] != "START":
            raise RuntimeError("expected START <tai_ns>")
        target_tai_ns = int(command[1])
        actual_tai_ns = player.play_at(target_tai_ns)
        print(f"STARTED {actual_tai_ns} DELTA_NS {actual_tai_ns - target_tai_ns}", flush=True)
        return 0
    finally:
        player.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
