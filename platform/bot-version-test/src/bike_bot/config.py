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
    # Audio capture is available to operators, but the live stream always
    # starts video-only until an explicit field-listening command arrives.
    audio_start_enabled: bool = False
    audio_mode: str = "input"
    audio_source: str = "@DEFAULT_SOURCE@"
    # ALSA PCM device used when audio_mode is local_alsa.  Keep the default
    # direct so existing installations retain their current behaviour; the
    # test deployment explicitly uses the shared dsnoop device.
    audio_device: str = "default"
    audio_sample_rate: int = 48000
    audio_channels: int = 1
    audio_bitrate: str = "64k"
    audio_capture_card: int = 2
    audio_capture_control: str = "Mic"
    audio_capture_volume: str = "100%"
    audio_control_state_path: str = "data/stream-audio-state"
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
    tensorrt_enabled: bool = True
    tensorrt_fp16: bool = True
    tensorrt_engine_cache_path: str = "data/trt-cache/yolo11n"


@dataclass
class DetectionConfig:
    event_type: str
    event_label: str
    risk_level: str = "medium"
    event_cooldown_seconds: int = 10
    event_confirm_frames: int = 3
    min_box_area: int = 4000
    tracking_enabled: bool = True
    tracker_iou_threshold: float = 0.3
    track_ttl_seconds: int = 30
    duplicate_alert_seconds: int = 300
    event_classes: list[str] | None = None
    person_detection_enabled: bool = False
    person_report_interval_seconds: float = 0.4
    person_track_hold_seconds: float = 0.75
    inference_rate_hz: float = 2.0
    person_follow_inference_rate_hz: float = 5.0


@dataclass
class TelemetryConfig:
    endpoint: str
    media_upload_endpoint: str = ""
    person_detection_endpoint: str = ""
    local_person_detection_path: str = "/run/roamerx/person_detections.json"
    timeout_seconds: int = 5
    verify_tls: bool = False
    device_key: str = ""
    heartbeat_enabled: bool = True
    status_enabled: bool = True
    heartbeat_interval_seconds: int = 5
    status_interval_seconds: int = 2


@dataclass
class AudioPlaybackConfig:
    enabled: bool = False
    skip_patrol_speech_sources: tuple = ()
    remote_host: str = ""
    remote_port: int = 22
    remote_user: str = ""
    identity_file: str = ""
    remote_temp_dir: str = "/tmp"
    pulse_server: str = ""
    pulse_sink: str = ""
    player_path: str = "/usr/bin/ffplay"
    sync_enabled: bool = True
    sync_lead_time_ms: int = 800
    sync_ready_timeout_seconds: float = 2.0
    sync_clock_offset_limit_ms: float = 1.0
    local_alsa_device: str = ""
    remote_alsa_device: str = ""
    local_latency_compensation_ms: float = 0.0
    remote_latency_compensation_ms: float = 0.0


@dataclass
class ControlConfig:
    enable: bool = True
    host: str = "0.0.0.0"
    port: int = 9100
    dry_run: bool = True
    sdk_enabled: bool = True
    sdk_lib_path: str = ""
    local_ip: str = "127.0.0.1"
    local_port: int = 43988
    robot_ip: str = "127.0.0.1"


@dataclass
class SnapshotConfig:
    directory: str = "snapshots"
    public_base_url: str = ""
    jpeg_quality: int = 90
    max_files: int = 500
    max_total_bytes: int = 1024 * 1024 * 1024


@dataclass
class DisplayConfig:
    enable: bool = True
    window_name: str = "Bike Bot Detection"
    show_fps: bool = True
    max_width: int = 1280


@dataclass
class StorageConfig:
    telemetry_log_path: str = "data/telemetry/telemetry.jsonl"
    telemetry_log_max_bytes: int = 20 * 1024 * 1024
    telemetry_log_backup_count: int = 5
    app_log_path: str = "data/logs/bike_bot.log"
    stream_log_path: str = "data/logs/ffmpeg-stream.log"
    run_stream_log_path: str = "data/logs/run_stream.log"
    log_max_bytes: int = 10 * 1024 * 1024
    log_backup_count: int = 5
    audio_status_dir: str = "/home/dogrobot/runtime/nx-edge/data/audio-status"


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
    person_model: ModelConfig | None
    detection: DetectionConfig
    telemetry: TelemetryConfig
    audio_playback: AudioPlaybackConfig
    control: ControlConfig
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
            person_model=ModelConfig(**data["person_model"]) if data.get("person_model") else None,
            detection=DetectionConfig(**data["detection"]),
            telemetry=TelemetryConfig(**data["telemetry"]),
            audio_playback=AudioPlaybackConfig(**data.get("audio_playback", {})),
            control=ControlConfig(**data.get("control", {})),
            snapshot=SnapshotConfig(**data["snapshot"]),
            display=DisplayConfig(**data.get("display", {})),
            storage=StorageConfig(**data.get("storage", {})),
            runtime=RuntimeConfig(**data["runtime"]),
        )

    def ensure_directories(self) -> None:
        Path(self.snapshot.directory).mkdir(parents=True, exist_ok=True)
        Path(self.storage.telemetry_log_path).parent.mkdir(parents=True, exist_ok=True)
        Path(self.storage.app_log_path).parent.mkdir(parents=True, exist_ok=True)
        Path(self.storage.stream_log_path).parent.mkdir(parents=True, exist_ok=True)
        Path(self.storage.run_stream_log_path).parent.mkdir(parents=True, exist_ok=True)
        Path(self.storage.audio_status_dir).mkdir(parents=True, exist_ok=True)
        cache_path = self.model.tensorrt_engine_cache_path
        if cache_path:
            try:
                Path(cache_path).mkdir(parents=True, exist_ok=True)
            except OSError:
                pass
