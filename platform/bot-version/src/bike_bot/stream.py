from __future__ import annotations

import logging
import shutil
import subprocess
import time
from pathlib import Path

from .config import AppConfig
from .logging_utils import rotate_file_if_needed

LOGGER = logging.getLogger(__name__)


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

    def build_command(self) -> list[str]:
        stream = self.config.stream
        video = self.config.video
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
        if not stream.audio_enabled:
            command.append("-an")

        if stream.video_codec == "copy":
            command.extend(["-c:v", "copy"])
        else:
            command.extend(["-c:v", stream.video_codec, "-preset", "veryfast", "-tune", "zerolatency"])

        command.extend(stream.extra_args or [])
        command.extend(["-f", "flv", stream.rtmp_url])
        return command

    def run_forever(self, stop_event) -> None:
        while not stop_event.is_set():
            command = self.build_command()
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
                process = subprocess.Popen(
                    command,
                    stdout=stream_log,
                    stderr=subprocess.STDOUT,
                )
                LOGGER.info(
                    "edge_perf stream_process_started pid=%s rtmp_url=%s ffmpeg_log=%s",
                    process.pid,
                    self.config.stream.rtmp_url,
                    log_path,
                )
                while process.poll() is None:
                    if stop_event.wait(1):
                        LOGGER.info("stopping zlm stream push")
                        process.terminate()
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill()
                        return

            runtime_seconds = time.perf_counter() - started_at
            LOGGER.warning(
                "edge_perf stream_process_exited code=%s runtime_ms=%.1f retrying_in_s=%s",
                process.returncode,
                _milliseconds(runtime_seconds),
                self.config.stream.reconnect_interval_seconds,
            )
            stop_event.wait(self.config.stream.reconnect_interval_seconds)
