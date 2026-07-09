from __future__ import annotations

import logging
import re
import shutil
import subprocess
import tempfile
from pathlib import Path
from urllib.parse import urlparse

import requests

from .config import AppConfig

LOGGER = logging.getLogger(__name__)


class AudioCommandClient:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.api_base = self._derive_api_base(config.telemetry.endpoint)
        self.cache_dir = Path("data/audio-cache")
        self.cache_dir.mkdir(parents=True, exist_ok=True)

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

    def handle_command(self, command: dict) -> None:
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

        self.report(command_id, "running", {"audio_url": audio_url}, "")
        started = subprocess.getoutput("date -Is")
        try:
            local_path = self._download_audio(audio_url)
            player = self._play_audio(local_path)
            self.report(
                command_id,
                "finished",
                {"audio_url": audio_url, "local_path": str(local_path), "player": player, "started_at": started},
                "",
            )
        except Exception as exc:
            LOGGER.exception("audio command failed id=%s url=%s", command_id, audio_url)
            self.report(command_id, "failed", {"audio_url": audio_url, "started_at": started}, str(exc))

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
        headers = {"X-Device-Code": self.config.robot.code}
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

    def _play_audio(self, local_path: Path) -> str:
        usb_device = self._detect_usb_audio_device()
        if usb_device and shutil.which("ffmpeg") and shutil.which("aplay"):
            LOGGER.info("playing audio through USB ALSA device %s: %s", usb_device, local_path)
            ffmpeg = subprocess.Popen(
                ["ffmpeg", "-nostdin", "-loglevel", "warning", "-i", str(local_path), "-f", "wav", "-"],
                stdout=subprocess.PIPE,
            )
            try:
                subprocess.run(["aplay", "-q", "-D", usb_device, "-"], stdin=ffmpeg.stdout, check=True, timeout=120)
                return f"ffmpeg|aplay:{usb_device}"
            finally:
                if ffmpeg.stdout:
                    ffmpeg.stdout.close()
                ffmpeg.wait(timeout=5)

        players = [
            ("ffplay", ["ffplay", "-nodisp", "-autoexit", "-loglevel", "warning", str(local_path)]),
            ("mpg123", ["mpg123", "-q", str(local_path)]),
            ("aplay", ["aplay", "-q", str(local_path)]),
        ]
        for name, command in players:
            if shutil.which(name):
                LOGGER.info("playing audio with %s: %s", name, local_path)
                subprocess.run(command, check=True, timeout=120)
                return name
        raise RuntimeError("no supported audio player found: ffplay, mpg123, or aplay")

    @staticmethod
    def _detect_usb_audio_device() -> str:
        try:
            result = subprocess.run(["aplay", "-l"], check=False, text=True, capture_output=True, timeout=5)
        except Exception:
            return ""

        for line in result.stdout.splitlines():
            if "USB" not in line.upper():
                continue
            match = re.search(r"card\s+(\d+):.*device\s+(\d+):", line)
            if match:
                return f"plughw:{match.group(1)},{match.group(2)}"
        return ""
