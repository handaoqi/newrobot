from __future__ import annotations

import logging
import shlex
import shutil
import subprocess
import threading
import time
from pathlib import Path

from .config import AppConfig
from .logging_utils import rotate_file_if_needed

LOGGER = logging.getLogger(__name__)
AUDIO_PROBE_INTERVAL_SECONDS = 15
AUDIO_CAPTURE_CONTROL_MODES = {"remote_pulse", "local_alsa"}


def _milliseconds(seconds: float) -> float:
    return round(seconds * 1000, 1)


class StreamPusher:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self._audio_state_lock = threading.Lock()
        self._audio_state_changed = threading.Event()
        self._audio_state_path = Path(config.stream.audio_control_state_path)
        self._audio_capture_enabled = self._load_audio_capture_state()

    def _load_audio_capture_state(self) -> bool:
        """Restore an operator's capture choice across a pusher restart."""
        default = bool(self.config.stream.audio_start_enabled)
        if self.config.stream.audio_mode not in AUDIO_CAPTURE_CONTROL_MODES:
            return default
        try:
            saved = self._audio_state_path.read_text(encoding="utf-8").strip().lower()
        except FileNotFoundError:
            return default
        except OSError as exc:
            LOGGER.warning("cannot read persisted stream audio state path=%s error=%s", self._audio_state_path, exc)
            return default
        if saved == "enabled":
            return True
        if saved == "disabled":
            return False
        LOGGER.warning("ignoring invalid persisted stream audio state path=%s value=%r", self._audio_state_path, saved)
        return default

    def supports_audio_capture_control(self) -> bool:
        return bool(
            self.config.stream.enable
            and self.config.stream.audio_enabled
            and self.config.stream.audio_mode in AUDIO_CAPTURE_CONTROL_MODES
        )

    def audio_capture_enabled(self) -> bool:
        with self._audio_state_lock:
            return self._audio_capture_enabled

    def set_audio_capture_enabled(self, enabled: bool) -> dict[str, bool | str]:
        """Switch the configured stream-audio capture without restarting the bot service."""
        if not self.supports_audio_capture_control():
            raise RuntimeError("stream audio capture control is not configured")
        if not isinstance(enabled, bool):
            raise ValueError("enabled must be a boolean")
        if enabled:
            self._verify_local_audio_capture()
        with self._audio_state_lock:
            changed = self._audio_capture_enabled != enabled
            self._audio_capture_enabled = enabled
            try:
                self._audio_state_path.parent.mkdir(parents=True, exist_ok=True)
                temporary_path = self._audio_state_path.with_suffix(self._audio_state_path.suffix + ".tmp")
                temporary_path.write_text("enabled\n" if enabled else "disabled\n", encoding="utf-8")
                temporary_path.replace(self._audio_state_path)
            except OSError as exc:
                LOGGER.warning("cannot persist stream audio state path=%s error=%s", self._audio_state_path, exc)
            if changed:
                self._audio_state_changed.set()
        LOGGER.info(
            "stream audio capture requested enabled=%s changed=%s mode=%s",
            enabled,
            changed,
            self.config.stream.audio_mode,
        )
        return {"enabled": enabled, "changed": changed, "mode": self.config.stream.audio_mode}

    def _verify_local_audio_capture(self) -> None:
        """Fail fast before replacing a healthy video-only pusher with audio."""
        if self.config.stream.audio_mode != "local_alsa":
            return
        stream = self.config.stream
        command = [
            self.resolve_ffmpeg_path(), "-nostdin", "-hide_banner", "-loglevel", "error",
            "-f", "alsa", "-ar", str(stream.audio_sample_rate), "-ac", str(stream.audio_channels),
            "-i", stream.audio_device, "-t", "0.25", "-f", "null", "-",
        ]
        try:
            result = subprocess.run(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                timeout=3,
                start_new_session=True,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("麦克风未启动，请打开音响电源后重试") from exc
        if result.returncode != 0:
            detail = (result.stderr or "").strip().splitlines()[-1:] or ["ALSA capture unavailable"]
            LOGGER.warning("local audio capture probe failed device=%s detail=%s", stream.audio_device, detail[0])
            raise RuntimeError("麦克风未启动，请打开音响电源后重试")

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

    def audio_capture_enabled_for_stream(self) -> bool:
        return self.supports_audio_capture_control() and self.audio_capture_enabled()

    def build_command(self, include_audio: bool | None = None) -> list[str]:
        stream = self.config.stream
        video = self.config.video
        if include_audio is None:
            include_audio = self.audio_capture_enabled_for_stream()
        if not stream.rtmp_url:
            raise ValueError("stream.rtmp_url is required when stream.enable is true")

        command = [
            self.resolve_ffmpeg_path(),
            "-hide_banner",
            "-loglevel",
            "warning",
        ]
        if isinstance(video.source, str) and video.source.startswith("rtsp://"):
            # Keep the RTSP reader close to the camera head.  This stream is
            # displayed live, so the default probe and packet queues are more
            # harmful than a short reconnect when a packet is lost.
            command.extend([
                "-fflags", "nobuffer",
                "-avioflags", "direct",
                "-probesize", "32",
                "-analyzeduration", "0",
                "-max_delay", "0",
                "-rtsp_transport", video.rtsp_transport,
            ])

        command.extend(["-i", str(video.source)])
        if include_audio:
            if stream.audio_mode == "remote_pulse":
                command.extend([
                    "-thread_queue_size", "512",
                    "-f", "s16le",
                    "-ar", str(stream.audio_sample_rate),
                    "-ac", str(stream.audio_channels),
                    "-i", "pipe:0",
                ])
            elif stream.audio_mode == "local_alsa":
                # The ALSA device is a dsnoop endpoint in this deployment, so
                # the voice assistant and the RTMP pusher read the exact same
                # 48 kHz stereo capture without one process monopolising it.
                command.extend([
                    "-thread_queue_size", "512",
                    "-f", "alsa",
                    "-ar", str(stream.audio_sample_rate),
                    "-ac", str(stream.audio_channels),
                    "-i", stream.audio_device,
                ])
            else:
                raise ValueError(f"unsupported stream audio mode: {stream.audio_mode}")
            command.extend(["-map", "0:v:0", "-map", "1:a:0"])
        elif not stream.audio_enabled or stream.audio_mode in AUDIO_CAPTURE_CONTROL_MODES:
            command.append("-an")

        if stream.video_codec == "copy":
            command.extend(["-c:v", "copy"])
        else:
            command.extend(["-c:v", stream.video_codec, "-preset", "veryfast", "-tune", "zerolatency"])

        if include_audio:
            command.extend([
                "-c:a", "aac",
                "-b:a", stream.audio_bitrate,
                "-ar", str(stream.audio_sample_rate),
                "-ac", str(stream.audio_channels),
            ])

        command.extend([
            "-flush_packets", "1",
            "-flvflags", "no_duration_filesize",
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
            f"{self._remote_capture_mixer_command(enabled=True)}; "
            f"PULSE_SERVER={shlex.quote(pulse_server)} exec parec --raw "
            f"--device={shlex.quote(source)} --format=s16le "
            f"--rate={int(stream.audio_sample_rate)} --channels={int(stream.audio_channels)}"
        )
        command.extend([f"{playback.remote_user}@{playback.remote_host}", remote_command])
        return command

    def _remote_capture_mixer_command(self, *, enabled: bool) -> str:
        stream = self.config.stream
        card = int(stream.audio_capture_card)
        control = shlex.quote(stream.audio_capture_control)
        if enabled:
            volume = shlex.quote(stream.audio_capture_volume)
            return f"amixer -c {card} sset {control} {volume} cap >/dev/null 2>&1 || true"
        return f"amixer -c {card} sset {control} nocap >/dev/null 2>&1 || true"

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
        if not self.audio_capture_enabled_for_stream():
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

    def _disable_remote_capture_hardware(self) -> None:
        """Release the 3588 USB capture endpoint after the parec consumer exits."""
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
        command.extend([
            f"{playback.remote_user}@{playback.remote_host}",
            self._remote_capture_mixer_command(enabled=False),
        ])
        try:
            result = subprocess.run(command, timeout=8, check=False)
            if result.returncode != 0:
                LOGGER.warning("remote capture hardware disable exited code=%s", result.returncode)
        except (OSError, subprocess.TimeoutExpired) as exc:
            LOGGER.warning("remote capture hardware disable failed: %s", exc)

    def run_forever(self, stop_event) -> None:
        if self.config.stream.audio_mode == "remote_pulse" and self.supports_audio_capture_control() and not self.audio_capture_enabled():
            self._disable_remote_capture_hardware()
        while not stop_event.is_set():
            use_remote_audio = False
            if self.config.stream.audio_mode == "remote_pulse":
                use_remote_audio = self.remote_audio_available()
                use_audio = use_remote_audio
            else:
                use_audio = self.audio_capture_enabled_for_stream()
            if self.config.stream.audio_mode == "remote_pulse" and self.audio_capture_enabled_for_stream() and not use_audio:
                LOGGER.warning("configured audio source is unavailable; starting video-only stream")
            command = self.build_command(include_audio=use_audio)
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
                restarted_for_audio_state = False
                while process.poll() is None:
                    if self._audio_state_changed.is_set():
                        self._audio_state_changed.clear()
                        restarted_for_audio_state = True
                        LOGGER.info("restarting zlm stream after remote audio capture state change")
                        self._terminate_process(process)
                        break
                    if audio_process and audio_process.poll() is not None:
                        LOGGER.warning(
                            "remote PulseAudio capture exited code=%s; falling back to video-only stream",
                            audio_process.returncode,
                        )
                        self._terminate_process(process)
                        break
                    if self.config.stream.audio_mode == "remote_pulse" and not use_remote_audio and self.audio_capture_enabled_for_stream():
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
                if use_remote_audio and not self.audio_capture_enabled_for_stream():
                    self._disable_remote_capture_hardware()

            runtime_seconds = time.perf_counter() - started_at
            LOGGER.warning(
                "edge_perf stream_process_exited code=%s runtime_ms=%.1f retrying_in_s=%s",
                process.returncode,
                _milliseconds(runtime_seconds),
                self.config.stream.reconnect_interval_seconds,
            )
            if not restarted_for_audio_state and not self._audio_state_changed.is_set():
                stop_event.wait(self.config.stream.reconnect_interval_seconds)
