from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass
class RobotConfig:
    code: str
    name: str


@dataclass
class LocationConfig:
    name: str
    latitude: float | None = None
    longitude: float | None = None


@dataclass
class VideoConfig:
    source: str | int
    width: int = 1280
    height: int = 720
    camera_id: str = "front"
    stream_id: str = ""
    play_urls: dict[str, str] | None = None
    rtsp_transport: str = "tcp"
    open_timeout_seconds: int = 8
    read_timeout_seconds: int = 8
    reconnect_interval_seconds: int = 3


@dataclass
class StreamConfig:
    enable: bool = False
    ffmpeg_path: str = "ffmpeg"
    rtmp_url: str = ""
    reconnect_interval_seconds: int = 5
    video_codec: str = "copy"
    audio_enabled: bool = False
    extra_args: list[str] | None = None


@dataclass
class ModelConfig:
    path: str
    backend: str = "auto"
    confidence: float = 0.55
    nms_iou_threshold: float = 0.45
    image_size: int = 960
    device: str = ""
    classes: list[str] | None = None


@dataclass
class DetectionConfig:
    event_type: str
    event_label: str
    risk_level: str = "medium"
    event_cooldown_seconds: int = 10
    min_box_area: int = 4000
    tracking_enabled: bool = True
    tracker_iou_threshold: float = 0.3
    track_ttl_seconds: int = 30
    duplicate_alert_seconds: int = 300


@dataclass
class TelemetryConfig:
    endpoint: str
    media_upload_endpoint: str = ""
    timeout_seconds: int = 5
    verify_tls: bool = False
    device_key: str = ""
    heartbeat_interval_seconds: int = 5
    status_interval_seconds: int = 2


@dataclass
class SnapshotConfig:
    directory: str = "snapshots"
    public_base_url: str = ""
    jpeg_quality: int = 90


@dataclass
class DisplayConfig:
    enable: bool = True
    window_name: str = "Bike Bot Detection"
    show_fps: bool = True
    max_width: int = 1280


@dataclass
class StorageConfig:
    telemetry_log_path: str = "data/telemetry/telemetry.jsonl"


@dataclass
class RuntimeConfig:
    mode: str = "auto"
    status: str = "online"
    battery_level: int = 100
    charging: bool = False
    signal_strength: int = 100
    network_type: str = "WiFi"
    speed: float = 0.0
    heading: float = 0.0


@dataclass
class AppConfig:
    robot: RobotConfig
    location: LocationConfig
    video: VideoConfig
    stream: StreamConfig
    model: ModelConfig
    detection: DetectionConfig
    telemetry: TelemetryConfig
    snapshot: SnapshotConfig
    display: DisplayConfig
    storage: StorageConfig
    runtime: RuntimeConfig

    @classmethod
    def from_file(cls, path: str | Path) -> "AppConfig":
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        video_source = data["video"]["source"]
        if isinstance(video_source, str) and video_source.isdigit():
            video_source = int(video_source)

        return cls(
            robot=RobotConfig(**data["robot"]),
            location=LocationConfig(**data["location"]),
            video=VideoConfig(source=video_source, **{k: v for k, v in data["video"].items() if k != "source"}),
            stream=StreamConfig(**data.get("stream", {})),
            model=ModelConfig(**data["model"]),
            detection=DetectionConfig(**data["detection"]),
            telemetry=TelemetryConfig(**data["telemetry"]),
            snapshot=SnapshotConfig(**data["snapshot"]),
            display=DisplayConfig(**data.get("display", {})),
            storage=StorageConfig(**data.get("storage", {})),
            runtime=RuntimeConfig(**data["runtime"]),
        )

    def ensure_directories(self) -> None:
        Path(self.snapshot.directory).mkdir(parents=True, exist_ok=True)
        Path(self.storage.telemetry_log_path).parent.mkdir(parents=True, exist_ok=True)
