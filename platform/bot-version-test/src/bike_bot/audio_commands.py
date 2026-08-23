from __future__ import annotations

import logging
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from urllib.parse import urlparse

import requests
import paramiko

from .config import AppConfig

LOGGER = logging.getLogger(__name__)

ALERT_AUDIO_FILTER = (
    "highpass=f=220,"
    "acompressor=threshold=0.06:ratio=12:attack=2:release=60:makeup=4,"
    "volume=12dB,"
    "asoftclip=type=hard:threshold=0.55:output=1.6:oversample=4,"
    "alimiter=limit=0.98:attack=1:release=20"
)


class PlaybackSuperseded(RuntimeError):
    pass


class AudioCommandClient:
    def __init__(self, config: AppConfig, stream_pusher=None) -> None:
        self.config = config
        self.stream_pusher = stream_pusher
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
        if action == "set_stream_audio":
            self.handle_stream_audio_command(command)
            return
        if action != "play_audio":
            message = f"unsupported cloud command action={action}"
            LOGGER.warning("%s id=%s", message, command_id)
            self.report(command_id, "failed", {}, message)
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
            alert_mode = bool(payload.get("alert_skill")) or str(payload.get("source") or "") in {
                "vision_bicycle_auto",
                "patrol_obstacle_speech",
            }
            player = self._play_audio(
                local_path,
                cancel_event,
                alert_mode=alert_mode,
                dual_output=bool(payload.get("dual_output", alert_mode)),
                allow_single_fallback=not bool(payload.get("preview")),
            )
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

    def handle_stream_audio_command(self, command: dict) -> None:
        """Apply remote live-stream audio capture and report its result."""
        command_id = command["id"]
        enabled = (command.get("payload") or {}).get("enabled")
        if not isinstance(enabled, bool):
            self.report(command_id, "failed", {}, "enabled must be a boolean")
            return
        if self.stream_pusher is None:
            self.report(command_id, "failed", {}, "stream audio control is unavailable")
            return
        self.report(command_id, "running", {"enabled": enabled}, "")
        try:
            result = self.stream_pusher.set_audio_capture_enabled(enabled)
            self.report(command_id, "finished", result, "")
        except Exception as exc:
            LOGGER.exception("stream audio command failed id=%s enabled=%s", command_id, enabled)
            self.report(command_id, "failed", {"enabled": enabled}, str(exc))

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
                self._active_processes.extend(process for process in processes if process not in self._active_processes)

    def _clear_processes(self, cancel_event: threading.Event, processes: list[subprocess.Popen]) -> None:
        with self._playback_lock:
            if self._active_cancel is cancel_event:
                self._active_processes = [process for process in self._active_processes if process not in processes]

    def _play_audio(
        self,
        local_path: Path,
        cancel_event: threading.Event | None = None,
        *,
        alert_mode: bool = False,
        dual_output: bool = False,
        allow_single_fallback: bool = True,
    ) -> str:
        cancel_event = cancel_event or threading.Event()
        if self.config.audio_playback.remote_host:
            if alert_mode and dual_output:
                return self._play_audio_dual(
                    local_path,
                    cancel_event,
                    allow_single_fallback=allow_single_fallback,
                )
            return self._play_audio_remote(local_path, cancel_event, alert_mode=alert_mode)

        return self._play_audio_local(local_path, cancel_event, alert_mode=alert_mode)

    def _play_audio_dual(
        self,
        local_path: Path,
        cancel_event: threading.Event,
        *,
        allow_single_fallback: bool,
    ) -> str:
        if not self.config.audio_playback.sync_enabled:
            return self._play_audio_dual_legacy(local_path, cancel_event)
        return self._play_audio_dual_scheduled(
            local_path,
            cancel_event,
            allow_single_fallback=allow_single_fallback,
        )

    def _play_audio_dual_legacy(self, local_path: Path, cancel_event: threading.Event) -> str:
        results: dict[str, str] = {}
        errors: list[BaseException] = []
        remote_ready = threading.Event()
        remote_done = threading.Event()
        start_playback = threading.Event()

        def play_local() -> None:
            try:
                if not start_playback.wait(timeout=25):
                    raise RuntimeError("timed out waiting for 3588 audio readiness")
                if cancel_event.is_set():
                    raise PlaybackSuperseded("replaced before dual playback")
                results["nx"] = self._play_audio_local(local_path, cancel_event, alert_mode=True)
            except BaseException as exc:
                errors.append(exc)

        def play_remote() -> None:
            try:
                results["3588"] = self._play_audio_remote(
                    local_path,
                    cancel_event,
                    alert_mode=True,
                    ready_event=remote_ready,
                    start_event=start_playback,
                )
            except BaseException as exc:
                errors.append(exc)
            finally:
                remote_done.set()

        local_thread = threading.Thread(target=play_local, daemon=True, name="alert-audio-nx")
        remote_thread = threading.Thread(target=play_remote, daemon=True, name="alert-audio-3588")
        local_thread.start()
        remote_thread.start()
        while not remote_ready.wait(timeout=0.1) and not remote_done.is_set():
            if cancel_event.is_set():
                break
        if not remote_ready.is_set() and not errors:
            errors.append(RuntimeError("3588 audio did not become ready"))
        start_playback.set()
        local_thread.join(timeout=130)
        remote_thread.join(timeout=130)
        if local_thread.is_alive():
            errors.append(RuntimeError("NX audio playback timed out"))
        if remote_thread.is_alive():
            errors.append(RuntimeError("3588 audio playback timed out"))
        if errors:
            if any(isinstance(error, PlaybackSuperseded) for error in errors):
                raise PlaybackSuperseded("replaced during dual-speaker playback")
            raise RuntimeError("dual-speaker playback failed: " + "; ".join(str(error) for error in errors))
        return f"dual(nx={results.get('nx')},3588={results.get('3588')})"

    def _play_audio_dual_scheduled(
        self,
        local_path: Path,
        cancel_event: threading.Event,
        *,
        allow_single_fallback: bool,
    ) -> str:

        pcm_path = self._prepare_sync_pcm(local_path, cancel_event)
        remote_path = f"{self.config.audio_playback.remote_temp_dir.rstrip('/')}/roamerx-sync-{os.getpid()}.wav"
        remote_helper = f"{self.config.audio_playback.remote_temp_dir.rstrip('/')}/roamerx-sync-audio-player.py"
        local_process: subprocess.Popen | None = None
        client = self._connect_remote()
        remote_stdin = remote_stdout = remote_stderr = None
        remote_sink = ""
        local_ready = threading.Event()
        remote_ready = threading.Event()
        ready_lines: dict[str, str] = {}
        try:
            self._assert_synchronized_clocks(client)
            remote_device, remote_sink = self._remote_audio_endpoints(client)
            local_device = self.config.audio_playback.local_alsa_device or self._detect_usb_audio_device()
            if not local_device:
                raise RuntimeError("NX USB audio device is unavailable")

            helper_path = Path(__file__).with_name("sync_audio_player.py")
            sftp = client.open_sftp()
            try:
                sftp.put(str(pcm_path), remote_path)
                sftp.put(str(helper_path), remote_helper)
                sftp.chmod(remote_path, 0o600)
                sftp.chmod(remote_helper, 0o700)
            finally:
                sftp.close()

            if remote_sink:
                self._remote_command(client, self._pulse_command(f"pactl suspend-sink {shlex.quote(remote_sink)} 1"))

            local_process = subprocess.Popen(
                [sys.executable, str(helper_path), "--device", local_device, "--file", str(pcm_path)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            self._register_processes(cancel_event, [local_process])

            pidfile = shlex.quote(self._remote_pidfile())
            remote_command = (
                f"setsid sh -c {shlex.quote(f'echo $$ > {pidfile}; exec python3 {shlex.quote(remote_helper)} --device {shlex.quote(remote_device)} --file {shlex.quote(remote_path)}')}"
            )
            remote_stdin, remote_stdout, remote_stderr = client.exec_command(remote_command, timeout=130)

            def read_ready(name: str, stream, event: threading.Event) -> None:
                ready_lines[name] = stream.readline().strip()
                event.set()

            threading.Thread(target=read_ready, args=("nx", local_process.stdout, local_ready), daemon=True).start()
            threading.Thread(target=read_ready, args=("3588", remote_stdout, remote_ready), daemon=True).start()
            deadline = time.monotonic() + self.config.audio_playback.sync_ready_timeout_seconds
            while time.monotonic() < deadline and not (local_ready.is_set() and remote_ready.is_set()):
                if cancel_event.wait(0.02):
                    raise PlaybackSuperseded("replaced before synchronized playback")

            nx_ok = local_ready.is_set() and ready_lines.get("nx") == "READY"
            remote_ok = remote_ready.is_set() and ready_lines.get("3588") == "READY"
            if not (nx_ok and remote_ok) and not allow_single_fallback:
                raise RuntimeError(
                    f"dual-speaker readiness failed: nx={ready_lines.get('nx', 'timeout')}, "
                    f"3588={ready_lines.get('3588', 'timeout')}"
                )
            if not nx_ok and not remote_ok:
                raise RuntimeError("neither speaker became ready")

            common_target = time.clock_gettime_ns(time.CLOCK_TAI) + int(
                self.config.audio_playback.sync_lead_time_ms * 1_000_000
            )
            active_outputs: list[str] = []
            if nx_ok:
                nx_target = common_target - int(
                    self.config.audio_playback.local_latency_compensation_ms * 1_000_000
                )
                local_process.stdin.write(f"START {nx_target}\n")
                local_process.stdin.flush()
                active_outputs.append("nx")
            elif local_process.poll() is None:
                local_process.terminate()
            if remote_ok:
                remote_target = common_target - int(
                    self.config.audio_playback.remote_latency_compensation_ms * 1_000_000
                )
                remote_stdin.write(f"START {remote_target}\n")
                remote_stdin.flush()
                active_outputs.append("3588")
            else:
                self._stop_remote_playback()

            finish_deadline = time.monotonic() + 130
            while time.monotonic() < finish_deadline:
                local_done = "nx" not in active_outputs or local_process.poll() is not None
                remote_done = "3588" not in active_outputs or remote_stdout.channel.exit_status_ready()
                if local_done and remote_done:
                    break
                if cancel_event.wait(0.05):
                    raise PlaybackSuperseded("replaced during synchronized playback")
            else:
                raise RuntimeError("synchronized audio playback timed out")

            local_output = local_process.stdout.read().strip() if nx_ok else ""
            remote_output = remote_stdout.read().decode("utf-8", errors="replace").strip() if remote_ok else ""
            local_error = local_process.stderr.read().strip() if nx_ok else ready_lines.get("nx", "")
            remote_error = remote_stderr.read().decode("utf-8", errors="replace").strip() if remote_ok else ready_lines.get("3588", "")
            if nx_ok and local_process.returncode != 0:
                raise RuntimeError(f"NX synchronized playback failed: {local_error}")
            if remote_ok and remote_stdout.channel.recv_exit_status() != 0:
                raise RuntimeError(f"3588 synchronized playback failed: {remote_error}")

            starts = self._parse_sync_starts({"nx": local_output, "3588": remote_output})
            skew_ms = abs(starts["nx"] - starts["3588"]) / 1_000_000 if len(starts) == 2 else None
            mode = "dual" if len(active_outputs) == 2 else f"fallback-{active_outputs[0]}"
            LOGGER.info("synchronized audio mode=%s trigger_skew_ms=%s", mode, skew_ms)
            return f"scheduled-{mode}(trigger_skew_ms={skew_ms if skew_ms is not None else 'n/a'})"
        finally:
            if local_process is not None:
                self._clear_processes(cancel_event, [local_process])
                if local_process.poll() is None:
                    local_process.terminate()
            try:
                if remote_sink:
                    self._remote_command(client, self._pulse_command(f"pactl suspend-sink {shlex.quote(remote_sink)} 0"), check=False)
                self._remote_command(
                    client,
                    f"rm -f {shlex.quote(remote_path)} {shlex.quote(remote_helper)} {shlex.quote(self._remote_pidfile())}",
                    check=False,
                )
            finally:
                client.close()
                pcm_path.unlink(missing_ok=True)

    def _prepare_sync_pcm(self, local_path: Path, cancel_event: threading.Event) -> Path:
        with tempfile.NamedTemporaryFile(prefix="sync-audio-", suffix=".wav", dir=self.cache_dir, delete=False) as tmp:
            pcm_path = Path(tmp.name)
        command = [
            self._ffmpeg_executable(), "-y", "-nostdin", "-loglevel", "error", "-i", str(local_path), "-vn",
            "-af", ALERT_AUDIO_FILTER,
            "-acodec", "pcm_s16le", "-ar", "48000", "-ac", "2", str(pcm_path),
        ]
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self._register_processes(cancel_event, [process])
        try:
            _, stderr = process.communicate(timeout=30)
            if cancel_event.is_set():
                raise PlaybackSuperseded("replaced while preparing synchronized audio")
            if process.returncode != 0:
                raise RuntimeError(stderr.decode("utf-8", errors="replace").strip() or "audio conversion failed")
            return pcm_path
        except Exception:
            pcm_path.unlink(missing_ok=True)
            raise
        finally:
            self._clear_processes(cancel_event, [process])

    def _assert_synchronized_clocks(self, client) -> None:
        local = subprocess.run(["chronyc", "tracking"], capture_output=True, text=True, timeout=5, check=False)
        _, remote_stdout, remote_stderr = client.exec_command("chronyc tracking", timeout=5)
        remote_text = remote_stdout.read().decode("utf-8", errors="replace")
        if local.returncode != 0 or remote_stdout.channel.recv_exit_status() != 0:
            detail = remote_stderr.read().decode("utf-8", errors="replace").strip()
            raise RuntimeError(f"Chrony status unavailable: {detail}")
        limit = self.config.audio_playback.sync_clock_offset_limit_ms / 1000
        offsets = [self._chrony_offset(local.stdout), self._chrony_offset(remote_text)]
        if any(offset > limit for offset in offsets):
            raise RuntimeError(f"clock offset exceeds {limit * 1000:.3f}ms: {offsets}")

    @staticmethod
    def _ffmpeg_executable() -> str:
        candidates = [
            Path(__file__).resolve().parents[3] / "bot-version" / "venv" / "lib" / "python3.10"
            / "site-packages" / "imageio_ffmpeg" / "binaries" / "ffmpeg-linux-aarch64-v7.0.2",
            Path("/usr/bin/ffmpeg"),
        ]
        for candidate in candidates:
            if candidate.is_file() and os.access(candidate, os.X_OK):
                return str(candidate)
        raise RuntimeError("full FFmpeg executable is unavailable")

    @staticmethod
    def _chrony_offset(output: str) -> float:
        match = re.search(r"System time\s*:\s*([0-9.]+) seconds", output)
        if not match:
            raise RuntimeError("Chrony output has no system offset")
        return abs(float(match.group(1)))

    def _remote_audio_endpoints(self, client) -> tuple[str, str]:
        _, cards_stdout, _ = client.exec_command("cat /proc/asound/cards", timeout=5)
        cards = cards_stdout.read().decode("utf-8", errors="replace")
        match = re.search(r"^\s*\d+\s+\[([^\]]+)\].*USB-Audio", cards, flags=re.MULTILINE)
        device = self.config.audio_playback.remote_alsa_device or (
            f"hw:CARD={match.group(1).strip()},DEV=0" if match else ""
        )
        if not device:
            raise RuntimeError("3588 USB ALSA device is unavailable")
        _, sinks_stdout, _ = client.exec_command(self._pulse_command("pactl list short sinks"), timeout=5)
        sinks = [
            fields[1]
            for line in sinks_stdout.read().decode("utf-8", errors="replace").splitlines()
            if len(fields := line.split("\t")) > 1
        ]
        configured = self.config.audio_playback.pulse_sink
        sink = configured if configured in sinks else next((item for item in sinks if "usb-" in item.lower()), "")
        return device, sink

    def _pulse_command(self, command: str) -> str:
        prefix = f"PULSE_SERVER={shlex.quote(self.config.audio_playback.pulse_server)} " if self.config.audio_playback.pulse_server else ""
        return prefix + command

    @staticmethod
    def _remote_command(client, command: str, *, check: bool = True) -> str:
        _, stdout, stderr = client.exec_command(command, timeout=10)
        output = stdout.read().decode("utf-8", errors="replace")
        code = stdout.channel.recv_exit_status()
        if check and code != 0:
            raise RuntimeError(stderr.read().decode("utf-8", errors="replace").strip() or command)
        return output

    @staticmethod
    def _parse_sync_starts(outputs: dict[str, str]) -> dict[str, int]:
        starts = {}
        for name, output in outputs.items():
            match = re.search(r"STARTED\s+(\d+)", output)
            if match:
                starts[name] = int(match.group(1))
        return starts

    def _play_audio_local(
        self,
        local_path: Path,
        cancel_event: threading.Event,
        *,
        alert_mode: bool = False,
    ) -> str:

        usb_device = self._detect_usb_audio_device()
        system_ffplay = Path("/usr/bin/ffplay")
        if usb_device and system_ffplay.exists():
            if alert_mode and shutil.which("amixer"):
                card_match = re.search(r"CARD=([^,]+)", usb_device)
                if card_match:
                    subprocess.run(
                        [
                            "bash", "-lc",
                            "controls=$(amixer -c \"$1\" scontrols); "
                            "control=$(printf '%s\\n' \"$controls\" | sed -n \"s/^Simple mixer control '\\([^']*\\)'.*/\\1/p\" "
                            "| grep -E '^(PCM|Playback Feature Unit)$' | head -n1); "
                            "test -n \"$control\" && amixer -q -c \"$1\" sset \"$control\" unmute",
                            "_", card_match.group(1),
                        ],
                        check=False,
                        timeout=5,
                    )
            command = [str(system_ffplay), "-nodisp", "-autoexit", "-loglevel", "warning"]
            if alert_mode:
                command.extend(["-af", ALERT_AUDIO_FILTER])
            command.append(str(local_path))
            environment = os.environ.copy()
            environment.update({"SDL_AUDIODRIVER": "alsa", "AUDIODEV": usb_device})
            process = subprocess.Popen(command, env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self._register_processes(cancel_event, [process])
            try:
                _, stderr = process.communicate(timeout=120)
                if cancel_event.is_set():
                    raise PlaybackSuperseded("replaced during NX USB playback")
                if process.returncode != 0:
                    raise RuntimeError(
                        stderr.decode("utf-8", errors="replace").strip() or "NX USB ffplay failed"
                    )
                return f"ffplay-alsa:{usb_device}"
            finally:
                self._clear_processes(cancel_event, [process])
                if process.poll() is None:
                    process.terminate()

        ffmpeg_available = bool(shutil.which("ffmpeg"))
        aplay_available = bool(shutil.which("aplay"))
        if usb_device and ffmpeg_available and aplay_available:
            LOGGER.info("playing audio through USB ALSA device %s: %s", usb_device, local_path)
            if alert_mode and shutil.which("amixer"):
                card_match = re.search(r"CARD=([^,]+)", usb_device)
                if card_match:
                    subprocess.run(
                        ["amixer", "-q", "-c", card_match.group(1), "sset", "PCM", "unmute"],
                        check=False,
                        timeout=5,
                    )
            ffmpeg_command = [
                "ffmpeg", "-nostdin", "-loglevel", "error", "-i", str(local_path), "-vn",
            ]
            if alert_mode:
                ffmpeg_command.extend(["-af", ALERT_AUDIO_FILTER])
            ffmpeg_command.extend([
                "-f", "s16le", "-acodec", "pcm_s16le", "-ar", "48000", "-ac", "2", "-",
            ])
            ffmpeg = subprocess.Popen(
                ffmpeg_command,
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
                self._clear_processes(cancel_event, [process for process in (ffmpeg, playback) if process is not None])
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
                    self._clear_processes(cancel_event, [process])
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

    def _play_audio_remote(
        self,
        local_path: Path,
        cancel_event: threading.Event,
        *,
        alert_mode: bool = False,
        ready_event: threading.Event | None = None,
        start_event: threading.Event | None = None,
    ) -> str:
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
            pulse_env = (
                f"PULSE_SERVER={shlex.quote(remote.pulse_server)} " if remote.pulse_server else ""
            )
            _, sink_stdout, _ = client.exec_command(
                f"{pulse_env}pactl list short sinks",
                timeout=10,
            )
            available_sinks = [
                line.split("\t", 2)[1]
                for line in sink_stdout.read().decode("utf-8", errors="replace").splitlines()
                if "\t" in line
            ]
            resolved_sink = remote.pulse_sink if remote.pulse_sink in available_sinks else ""
            if not resolved_sink:
                resolved_sink = next((name for name in available_sinks if "usb-" in name.lower()), "")
            if not resolved_sink:
                raise RuntimeError("remote USB audio sink is unavailable")
            env_parts = []
            if remote.pulse_server:
                env_parts.append(f"PULSE_SERVER={shlex.quote(remote.pulse_server)}")
            env_parts.append(f"PULSE_SINK={shlex.quote(resolved_sink)}")
            env_parts.append("SDL_AUDIODRIVER=pulseaudio")
            player = shlex.quote(remote.player_path)
            quoted_path = shlex.quote(remote_path)
            quoted_pidfile = shlex.quote(pidfile)
            playback_command = f"exec {player} -nodisp -autoexit -loglevel warning {quoted_path}"
            if alert_mode:
                sink = shlex.quote(resolved_sink)
                playback_command = (
                    f"pactl set-sink-mute {sink} 0; "
                    f"exec {player} -nodisp -autoexit -loglevel warning "
                    f"-af {shlex.quote(ALERT_AUDIO_FILTER)} {quoted_path}"
                )
            command = (
                "set -eu; "
                f"cleanup() {{ rm -f {quoted_path}; "
                f"if [ -r {quoted_pidfile} ] && [ \"$(cat {quoted_pidfile})\" = \"$player_pid\" ]; "
                f"then rm -f {quoted_pidfile}; fi; }}; trap cleanup EXIT; "
                + " ".join(env_parts)
                + f" setsid sh -c {shlex.quote(playback_command)} & "
                + f"player_pid=$!; echo \"$player_pid\" > {quoted_pidfile}; wait \"$player_pid\""
            )
            if ready_event is not None:
                ready_event.set()
            if start_event is not None:
                if not start_event.wait(timeout=25):
                    raise RuntimeError("timed out waiting for synchronized audio start")
                if cancel_event.is_set():
                    raise PlaybackSuperseded("replaced before remote playback")
            _, stdout, stderr = client.exec_command(command, timeout=130)
            exit_code = stdout.channel.recv_exit_status()
            error_text = stderr.read().decode("utf-8", errors="replace").strip()
            if cancel_event.is_set():
                raise PlaybackSuperseded("replaced during remote playback")
            if exit_code != 0:
                raise RuntimeError(f"remote audio playback failed ({exit_code}): {error_text}")
            return f"ssh-ffplay:{remote.remote_user}@{remote.remote_host}:{resolved_sink}"
        finally:
            client.close()

    @staticmethod
    def _detect_usb_audio_device() -> str:
        try:
            cards = Path("/proc/asound/cards").read_text(encoding="utf-8", errors="replace")
        except Exception:
            return ""

        match = re.search(r"^\s*\d+\s+\[([^\]]+)\].*USB-Audio", cards, flags=re.MULTILINE)
        if match:
            return f"hw:CARD={match.group(1).strip()},DEV=0"
        return ""
