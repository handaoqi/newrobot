from __future__ import annotations

import logging
import shlex
import shutil
import subprocess
import time
from pathlib import Path

from .config import AppConfig
from .logging_utils import rotate_file_if_needed

LOGGER = logging.getLogger(__name__)
AUDIO_PROBE_INTERVAL_SECONDS = 15


def _milliseconds(seconds: float) -> float:
    return round(seconds * 1000, 1)


class StreamPusher:
    def __init__(self, config: AppConfig) -> None:
        self.config = config

    def resolve_ffmpeg_path(self) -> str:
        ffmpeg_path = self.config.stream.ffmpeg_path
        if shutil.which(ffmpeg_path):
            return ffmpeg_path

        try:
            import imageio_ffmpeg
        except ImportError as exc:
            raise FileNotFoundError(
                f"ffmpeg executable not found: {ffmpeg_path}. "
                "Install ffmpeg or install imageio-ffmpeg in the current Python environment."
            ) from exc

        return imageio_ffmpeg.get_ffmpeg_exe()

    def remote_audio_enabled(self) -> bool:
        return self.config.stream.audio_enabled and self.config.stream.audio_mode == "remote_pulse"

    def build_command(self, include_remote_audio: bool | None = None) -> list[str]:
        stream = self.config.stream
        video = self.config.video
        if include_remote_audio is None:
            include_remote_audio = self.remote_audio_enabled()
        if not stream.rtmp_url:
            raise ValueError("stream.rtmp_url is required when stream.enable is true")

        command = [
            self.resolve_ffmpeg_path(),
            "-hide_banner",
            "-loglevel",
            "warning",
        ]
        if isinstance(video.source, str) and video.source.startswith("rtsp://"):
            command.extend(["-rtsp_transport", video.rtsp_transport])

        command.extend(["-i", str(video.source)])
        if include_remote_audio:
            command.extend([
                "-thread_queue_size", "512",
                "-f", "s16le",
                "-ar", str(stream.audio_sample_rate),
                "-ac", str(stream.audio_channels),
                "-i", "pipe:0",
                "-map", "0:v:0",
                "-map", "1:a:0",
            ])
        elif not stream.audio_enabled or self.remote_audio_enabled():
            command.append("-an")

        if stream.video_codec == "copy":
            command.extend(["-c:v", "copy"])
        else:
            command.extend(["-c:v", stream.video_codec, "-preset", "veryfast", "-tune", "zerolatency"])

        if include_remote_audio:
            command.extend([
                "-c:a", "aac",
                "-b:a", stream.audio_bitrate,
                "-ar", str(stream.audio_sample_rate),
                "-ac", str(stream.audio_channels),
            ])

        command.extend(stream.extra_args or [])
        command.extend(["-f", "flv", stream.rtmp_url])
        return command

    def build_remote_audio_command(self) -> list[str]:
        playback = self.config.audio_playback
        stream = self.config.stream
        if not playback.remote_host or not playback.remote_user:
            raise ValueError("remote PulseAudio capture requires audio_playback remote_host and remote_user")
        command = [
            "ssh",
            "-T",
            "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=6",
            "-o", "ServerAliveInterval=5",
            "-p", str(playback.remote_port),
        ]
        if playback.identity_file:
            command.extend(["-i", playback.identity_file])
        pulse_server = playback.pulse_server or "/run/user/1000/pulse/native"
        source = stream.audio_source or "@DEFAULT_SOURCE@"
        remote_command = (
            "amixer -c 1 sset 'Capture Feature Unit' 2 cap >/dev/null 2>&1 || true; "
            f"PULSE_SERVER={shlex.quote(pulse_server)} exec parec --raw "
            f"--device={shlex.quote(source)} --format=s16le "
            f"--rate={int(stream.audio_sample_rate)} --channels={int(stream.audio_channels)}"
        )
        command.extend([f"{playback.remote_user}@{playback.remote_host}", remote_command])
        return command

    def build_remote_audio_probe_command(self) -> list[str]:
        playback = self.config.audio_playback
        command = [
            "ssh",
            "-T",
            "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=6",
            "-p", str(playback.remote_port),
        ]
        if playback.identity_file:
            command.extend(["-i", playback.identity_file])
        pulse_server = playback.pulse_server or "/run/user/1000/pulse/native"
        remote_command = f"PULSE_SERVER={shlex.quote(pulse_server)} pactl list short sources"
        command.extend([f"{playback.remote_user}@{playback.remote_host}", remote_command])
        return command

    def remote_audio_available(self) -> bool:
        if not self.remote_audio_enabled():
            return False
        try:
            result = subprocess.run(
                self.build_remote_audio_probe_command(),
                capture_output=True,
                text=True,
                timeout=8,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            LOGGER.warning("remote audio probe failed; using video-only stream: %s", exc)
            return False
        if result.returncode != 0:
            LOGGER.warning(
                "remote audio probe exited code=%s; using video-only stream",
                result.returncode,
            )
            return False

        sources = {
            fields[1]
            for line in result.stdout.splitlines()
            if len(fields := line.split("\t")) > 1
        }
        configured_source = self.config.stream.audio_source or "@DEFAULT_SOURCE@"
        if configured_source == "@DEFAULT_SOURCE@":
            return any(not source.endswith(".monitor") for source in sources)
        return configured_source in sources

    @staticmethod
    def _terminate_process(process: subprocess.Popen | None) -> None:
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)

    def run_forever(self, stop_event) -> None:
        while not stop_event.is_set():
            use_remote_audio = self.remote_audio_available()
            if self.remote_audio_enabled() and not use_remote_audio:
                LOGGER.warning("configured audio source is unavailable; starting video-only stream")
            command = self.build_command(include_remote_audio=use_remote_audio)
            LOGGER.info("starting zlm stream push: %s", " ".join(command))
            started_at = time.perf_counter()
            log_path = Path(self.config.storage.stream_log_path)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            rotate_file_if_needed(
                log_path,
                self.config.storage.log_max_bytes,
                self.config.storage.log_backup_count,
            )
            with log_path.open("a", encoding="utf-8") as stream_log:
                audio_process = None
                ffmpeg_stdin = None
                if use_remote_audio:
                    audio_command = self.build_remote_audio_command()
                    LOGGER.info("starting remote PulseAudio capture host=%s source=%s", self.config.audio_playback.remote_host, self.config.stream.audio_source)
                    audio_process = subprocess.Popen(
                        audio_command,
                        stdout=subprocess.PIPE,
                        stderr=stream_log,
                    )
                    ffmpeg_stdin = audio_process.stdout
                process = subprocess.Popen(
                    command,
                    stdin=ffmpeg_stdin,
                    stdout=stream_log,
                    stderr=subprocess.STDOUT,
                )
                if audio_process and audio_process.stdout:
                    audio_process.stdout.close()
                LOGGER.info(
                    "edge_perf stream_process_started pid=%s rtmp_url=%s ffmpeg_log=%s",
                    process.pid,
                    self.config.stream.rtmp_url,
                    log_path,
                )
                while process.poll() is None:
                    if audio_process and audio_process.poll() is not None:
                        LOGGER.warning(
                            "remote PulseAudio capture exited code=%s; falling back to video-only stream",
                            audio_process.returncode,
                        )
                        self._terminate_process(process)
                        break
                    if not use_remote_audio and self.remote_audio_enabled():
                        if stop_event.wait(AUDIO_PROBE_INTERVAL_SECONDS):
                            self._terminate_process(process)
                            return
                        if self.remote_audio_available():
                            LOGGER.info("remote audio source is available; restarting stream with audio")
                            self._terminate_process(process)
                            break
                        continue
                    if stop_event.wait(1):
                        LOGGER.info("stopping zlm stream push")
                        self._terminate_process(process)
                        self._terminate_process(audio_process)
                        return
                self._terminate_process(audio_process)

            runtime_seconds = time.perf_counter() - started_at
            LOGGER.warning(
                "edge_perf stream_process_exited code=%s runtime_ms=%.1f retrying_in_s=%s",
                process.returncode,
                _milliseconds(runtime_seconds),
                self.config.stream.reconnect_interval_seconds,
            )
            stop_event.wait(self.config.stream.reconnect_interval_seconds)
