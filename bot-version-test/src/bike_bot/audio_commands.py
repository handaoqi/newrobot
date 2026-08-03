from __future__ import annotations

import logging
import os
import re
import shlex
import shutil
import subprocess
import tempfile
import threading
from pathlib import Path
from urllib.parse import urlparse

import requests
import paramiko

from .config import AppConfig

LOGGER = logging.getLogger(__name__)


class PlaybackSuperseded(RuntimeError):
    pass


class AudioCommandClient:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.api_base = self._derive_api_base(config.telemetry.endpoint)
        self.cache_dir = Path("data/audio-cache")
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self._playback_lock = threading.Lock()
        self._active_cancel: threading.Event | None = None
        self._active_processes: list[subprocess.Popen] = []
        self._active_thread: threading.Thread | None = None

    @staticmethod
    def _derive_api_base(endpoint: str) -> str:
        if "/api/" in endpoint:
            return endpoint.split("/api/", 1)[0].rstrip("/") + "/api"
        return endpoint.rstrip("/")

    def poll_once(self) -> dict | None:
        response = requests.get(
            f"{self.api_base}/device/commands/poll/",
            params={"robot_code": self.config.robot.code},
            headers=self._headers(),
            timeout=self.config.telemetry.timeout_seconds,
            verify=self.config.telemetry.verify_tls,
        )
        if response.status_code == 204:
            return None
        response.raise_for_status()
        return response.json()

    def replace_command(self, command: dict) -> None:
        """Interrupt the old announcement and start the newest one immediately."""
        self.interrupt_current()
        cancel_event = threading.Event()
        thread = threading.Thread(
            target=self.handle_command,
            args=(command, cancel_event),
            daemon=True,
            name=f"audio-command-{command.get('id', 'unknown')}",
        )
        with self._playback_lock:
            self._active_cancel = cancel_event
            self._active_processes = []
            self._active_thread = thread
        thread.start()

    def interrupt_current(self) -> None:
        with self._playback_lock:
            cancel_event = self._active_cancel
            processes = list(self._active_processes)
        if cancel_event is None:
            return
        cancel_event.set()
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
        if self.config.audio_playback.remote_host:
            try:
                self._stop_remote_playback()
            except Exception:
                LOGGER.exception("failed to stop superseded remote audio")

    def shutdown(self) -> None:
        self.interrupt_current()

    def handle_command(self, command: dict, cancel_event: threading.Event | None = None) -> None:
        cancel_event = cancel_event or threading.Event()
        command_id = command["id"]
        action = command.get("action")
        if action != "play_audio":
            LOGGER.warning("unsupported cloud command action=%s id=%s", action, command_id)
            return

        payload = command.get("payload") or {}
        audio_url = str(payload.get("audio_url") or "").strip()
        if not audio_url:
            self.report(command_id, "failed", {}, "audio_url is required")
            return

        self.report(command_id, "running", {"audio_url": audio_url, "playback_policy": "latest_wins"}, "")
        started = subprocess.getoutput("date -Is")
        local_path = None
        try:
            local_path = self._download_audio(audio_url)
            if cancel_event.is_set():
                raise PlaybackSuperseded("replaced before playback")
            player = self._play_audio(local_path, cancel_event)
            if cancel_event.is_set():
                raise PlaybackSuperseded("replaced during playback")
            self.report(
                command_id,
                "finished",
                {"audio_url": audio_url, "local_path": str(local_path), "player": player, "started_at": started},
                "",
            )
        except PlaybackSuperseded as exc:
            self.report(
                command_id,
                "superseded",
                {"audio_url": audio_url, "started_at": started, "playback_policy": "latest_wins"},
                str(exc),
            )
        except Exception as exc:
            if cancel_event.is_set():
                self.report(
                    command_id,
                    "superseded",
                    {"audio_url": audio_url, "started_at": started, "playback_policy": "latest_wins"},
                    "replaced by a newer announcement",
                )
                return
            LOGGER.exception("audio command failed id=%s url=%s", command_id, audio_url)
            self.report(command_id, "failed", {"audio_url": audio_url, "started_at": started}, str(exc))
        finally:
            if local_path is not None:
                local_path.unlink(missing_ok=True)

    def report(self, command_id: int, status: str, response_payload: dict, error_message: str) -> None:
        response = requests.post(
            f"{self.api_base}/device/commands/{command_id}/report/",
            json={
                "robot_code": self.config.robot.code,
                "status": status,
                "response_payload": response_payload,
                "error_message": error_message,
            },
            headers=self._headers(),
            timeout=self.config.telemetry.timeout_seconds,
            verify=self.config.telemetry.verify_tls,
        )
        response.raise_for_status()

    def _headers(self) -> dict[str, str]:
        headers = {
            "X-Device-Code": self.config.robot.code,
            "X-Device-Id": os.getenv("BIKE_BOT_DEVICE_ID", self.config.robot.code),
        }
        if self.config.telemetry.device_key:
            headers["X-Device-Key"] = self.config.telemetry.device_key
        return headers

    def _download_audio(self, audio_url: str) -> Path:
        parsed = urlparse(audio_url)
        suffix = Path(parsed.path).suffix or ".audio"
        with requests.get(
            audio_url,
            stream=True,
            timeout=self.config.telemetry.timeout_seconds,
            verify=self.config.telemetry.verify_tls,
        ) as response:
            response.raise_for_status()
            with tempfile.NamedTemporaryFile(prefix="cloud-audio-", suffix=suffix, dir=self.cache_dir, delete=False) as tmp:
                for chunk in response.iter_content(chunk_size=64 * 1024):
                    if chunk:
                        tmp.write(chunk)
                return Path(tmp.name)

    def _register_processes(self, cancel_event: threading.Event, processes: list[subprocess.Popen]) -> None:
        with self._playback_lock:
            if self._active_cancel is cancel_event:
                self._active_processes = processes

    def _clear_processes(self, cancel_event: threading.Event) -> None:
        with self._playback_lock:
            if self._active_cancel is cancel_event:
                self._active_processes = []

    def _play_audio(self, local_path: Path, cancel_event: threading.Event | None = None) -> str:
        cancel_event = cancel_event or threading.Event()
        if self.config.audio_playback.remote_host:
            return self._play_audio_remote(local_path, cancel_event)

        usb_device = self._detect_usb_audio_device()
        ffmpeg_available = bool(shutil.which("ffmpeg"))
        aplay_available = bool(shutil.which("aplay"))
        if usb_device and ffmpeg_available and aplay_available:
            LOGGER.info("playing audio through USB ALSA device %s: %s", usb_device, local_path)
            ffmpeg = subprocess.Popen(
                [
                    "ffmpeg",
                    "-nostdin",
                    "-loglevel",
                    "error",
                    "-i",
                    str(local_path),
                    "-vn",
                    "-f",
                    "s16le",
                    "-acodec",
                    "pcm_s16le",
                    "-ar",
                    "48000",
                    "-ac",
                    "2",
                    "-",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            playback = None
            try:
                playback = subprocess.Popen(
                    [
                        "aplay",
                        "-q",
                        "-D",
                        usb_device,
                        "-t",
                        "raw",
                        "-f",
                        "S16_LE",
                        "-r",
                        "48000",
                        "-c",
                        "2",
                    ],
                    stdin=ffmpeg.stdout,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                self._register_processes(cancel_event, [ffmpeg, playback])
                _, playback_stderr_bytes = playback.communicate(timeout=120)
                if ffmpeg.stdout:
                    ffmpeg.stdout.close()
                ffmpeg.wait(timeout=5)
                ffmpeg_stderr = (ffmpeg.stderr.read() if ffmpeg.stderr else b"").decode(
                    "utf-8", errors="replace"
                ).strip()
                aplay_stderr = (playback_stderr_bytes or b"").decode("utf-8", errors="replace").strip()
                if cancel_event.is_set():
                    raise PlaybackSuperseded("replaced during USB playback")
                if playback.returncode != 0 or ffmpeg.returncode != 0:
                    details = "; ".join(part for part in (aplay_stderr, ffmpeg_stderr) if part)
                    raise RuntimeError(
                        f"USB audio playback failed (aplay={playback.returncode}, ffmpeg={ffmpeg.returncode})"
                        + (f": {details}" if details else "")
                    )
                return f"ffmpeg|aplay:{usb_device}"
            finally:
                self._clear_processes(cancel_event)
                if playback is not None and playback.poll() is None:
                    playback.terminate()
                if ffmpeg.stdout:
                    ffmpeg.stdout.close()
                if ffmpeg.poll() is None:
                    ffmpeg.terminate()
                    ffmpeg.wait(timeout=5)

        players = [
            ("ffplay", ["ffplay", "-nodisp", "-autoexit", "-loglevel", "warning", str(local_path)]),
            ("mpg123", ["mpg123", "-q", str(local_path)]),
            ("aplay", ["aplay", "-q", str(local_path)]),
        ]
        for name, command in players:
            if shutil.which(name):
                LOGGER.info("playing audio with %s: %s", name, local_path)
                process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self._register_processes(cancel_event, [process])
                try:
                    _, stderr = process.communicate(timeout=120)
                    if cancel_event.is_set():
                        raise PlaybackSuperseded(f"replaced during {name} playback")
                    if process.returncode != 0:
                        raise RuntimeError(stderr.decode("utf-8", errors="replace").strip() or f"{name} failed")
                finally:
                    self._clear_processes(cancel_event)
                    if process.poll() is None:
                        process.terminate()
                return name
        raise RuntimeError("no supported audio player found: ffplay, mpg123, or aplay")

    def _remote_pidfile(self) -> str:
        safe_code = re.sub(r"[^A-Za-z0-9_.-]", "_", self.config.robot.code)
        return f"{self.config.audio_playback.remote_temp_dir.rstrip('/')}/roamerx-audio-{safe_code}.pid"

    def _connect_remote(self):
        remote = self.config.audio_playback
        if not remote.remote_user or not remote.identity_file:
            raise RuntimeError("remote audio playback requires remote_user and identity_file")
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(
            hostname=remote.remote_host,
            port=remote.remote_port,
            username=remote.remote_user,
            key_filename=remote.identity_file,
            look_for_keys=False,
            allow_agent=False,
            timeout=10,
            auth_timeout=10,
        )
        return client

    def _stop_remote_playback(self) -> None:
        pidfile = shlex.quote(self._remote_pidfile())
        client = self._connect_remote()
        try:
            command = (
                f"if [ -r {pidfile} ]; then "
                f"pid=$(cat {pidfile}); "
                "case \"$pid\" in ''|*[!0-9]*) ;; *) kill -TERM -- \"-$pid\" 2>/dev/null || kill -TERM \"$pid\" 2>/dev/null || true ;; esac; "
                f"rm -f {pidfile}; fi"
            )
            _, stdout, stderr = client.exec_command(command, timeout=10)
            exit_code = stdout.channel.recv_exit_status()
            if exit_code != 0:
                raise RuntimeError(stderr.read().decode("utf-8", errors="replace").strip())
        finally:
            client.close()

    def _play_audio_remote(self, local_path: Path, cancel_event: threading.Event) -> str:
        remote = self.config.audio_playback
        remote_path = f"{remote.remote_temp_dir.rstrip('/')}/bike-bot-audio-{os.getpid()}-{local_path.name}"
        pidfile = self._remote_pidfile()
        client = self._connect_remote()
        try:
            sftp = client.open_sftp()
            try:
                sftp.put(str(local_path), remote_path)
                sftp.chmod(remote_path, 0o600)
            finally:
                sftp.close()
            env_parts = []
            if remote.pulse_server:
                env_parts.append(f"PULSE_SERVER={shlex.quote(remote.pulse_server)}")
            if remote.pulse_sink:
                env_parts.append(f"PULSE_SINK={shlex.quote(remote.pulse_sink)}")
            env_parts.append("SDL_AUDIODRIVER=pulseaudio")
            player = shlex.quote(remote.player_path)
            quoted_path = shlex.quote(remote_path)
            quoted_pidfile = shlex.quote(pidfile)
            command = (
                "set -eu; "
                f"cleanup() {{ rm -f {quoted_path}; "
                f"if [ -r {quoted_pidfile} ] && [ \"$(cat {quoted_pidfile})\" = \"$player_pid\" ]; "
                f"then rm -f {quoted_pidfile}; fi; }}; trap cleanup EXIT; "
                + " ".join(env_parts)
                + f" setsid {player} -nodisp -autoexit -loglevel warning {quoted_path} & "
                + f"player_pid=$!; echo \"$player_pid\" > {quoted_pidfile}; wait \"$player_pid\""
            )
            _, stdout, stderr = client.exec_command(command, timeout=130)
            exit_code = stdout.channel.recv_exit_status()
            error_text = stderr.read().decode("utf-8", errors="replace").strip()
            if cancel_event.is_set():
                raise PlaybackSuperseded("replaced during remote playback")
            if exit_code != 0:
                raise RuntimeError(f"remote audio playback failed ({exit_code}): {error_text}")
            return f"ssh-ffplay:{remote.remote_user}@{remote.remote_host}:{remote.pulse_sink or 'default'}"
        finally:
            client.close()

    @staticmethod
    def _detect_usb_audio_device() -> str:
        try:
            result = subprocess.run(["aplay", "-l"], check=False, text=True, capture_output=True, timeout=5)
        except Exception:
            return ""

        for line in result.stdout.splitlines():
            if "USB" not in line.upper():
                continue
            match = re.search(r"card\s+\d+:\s*([^\s]+).*device\s+(\d+):", line)
            if match:
                return f"hw:CARD={match.group(1)},DEV={match.group(2)}"
        return ""
