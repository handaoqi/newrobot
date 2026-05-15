from __future__ import annotations

import logging
import subprocess
import time

from .config import AppConfig

LOGGER = logging.getLogger(__name__)


class StreamPusher:
    def __init__(self, config: AppConfig) -> None:
        self.config = config

    def build_command(self) -> list[str]:
        stream = self.config.stream
        video = self.config.video
        if not stream.rtmp_url:
            raise ValueError("stream.rtmp_url is required when stream.enable is true")

        command = [
            stream.ffmpeg_path,
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
            process = subprocess.Popen(command)
            while process.poll() is None:
                if stop_event.wait(1):
                    LOGGER.info("stopping zlm stream push")
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                    return

            LOGGER.warning(
                "zlm stream push exited code=%s, retrying in %ss",
                process.returncode,
                self.config.stream.reconnect_interval_seconds,
            )
            stop_event.wait(self.config.stream.reconnect_interval_seconds)
