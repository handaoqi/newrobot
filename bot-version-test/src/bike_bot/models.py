from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Any


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


@dataclass
class Position:
    name: str
    latitude: float | None = None
    longitude: float | None = None


@dataclass
class Motion:
    speed: float | None = None
    heading: float | None = None


@dataclass
class Power:
    battery_level: int
    charging: bool = False


@dataclass
class Network:
    signal_strength: int
    network_type: str | None = None


@dataclass
class RuntimeInfo:
    mode: str
    status: str


@dataclass
class BoundingBox:
    x: int
    y: int
    width: int
    height: int


@dataclass
class VideoInfo:
    stream_id: str
    camera_id: str = "front"
    frame_width: int | None = None
    frame_height: int | None = None
    frame_timestamp: str = field(default_factory=now_iso)
    play_urls: dict[str, str] | None = None

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        return {key: value for key, value in result.items() if value not in (None, "", {})}


@dataclass
class DetectionPayload:
    type: str
    label: str
    confidence: float
    risk_level: str
    object_class: str | None = None
    track_id: str | None = None
    bbox: BoundingBox | None = None
    snapshot_url: str | None = None
    local_snapshot_path: str | None = None
    event_time: str = field(default_factory=now_iso)

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        if self.bbox is None:
            result.pop("bbox", None)
        if self.snapshot_url is None:
            result.pop("snapshot_url", None)
        if self.object_class is None:
            result.pop("object_class", None)
        if self.track_id is None:
            result.pop("track_id", None)
        result.pop("local_snapshot_path", None)
        return result


@dataclass
class TelemetryPayload:
    sequence_id: str
    robot_code: str
    robot_name: str | None
    reported_at: str
    position: Position
    motion: Motion
    power: Power
    network: Network
    runtime: RuntimeInfo
    video: VideoInfo | None = None
    detections: list[DetectionPayload] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        if self.video is None:
            payload.pop("video", None)
        else:
            payload["video"] = self.video.to_dict()
        payload["detections"] = [item.to_dict() for item in self.detections]
        if self.robot_name is None:
            payload.pop("robot_name", None)
        return payload
