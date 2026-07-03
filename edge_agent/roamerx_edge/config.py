from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass
class RobotConfig:
    id: str
    agent_version: str = "0.1.0"
    current_map_id: str = ""
    current_map_version: str = ""


@dataclass
class MqttConfig:
    host: str
    port: int = 8883
    username: str = ""
    password: str = ""
    ca_file: str = ""
    cert_file: str = ""
    key_file: str = ""
    keepalive_seconds: int = 20
    session_expiry_seconds: int = 86400


@dataclass
class RosConfig:
    localization_topic: str = "/localization_info"
    odometry_topic: str = "/odom/localization_odom"
    battery_topic: str = ""
    follow_waypoints_action: str = "/follow_waypoints"


@dataclass
class TelemetryConfig:
    status_interval_seconds: float = 2
    heartbeat_interval_seconds: float = 10
    trajectory_flush_seconds: float = 5
    trajectory_batch_size: int = 20


@dataclass
class SafetyConfig:
    stop_speed_threshold_mps: float = 0.03
    stop_confirmation_seconds: float = 1.0
    low_battery_percent: int = 20


@dataclass
class StorageConfig:
    sqlite_path: str = "data/edge.db"


@dataclass
class MediaConfig:
    upload_url: str = ""
    map_upload_url: str = ""
    device_id: str = ""
    device_key: str = ""


@dataclass
class MappingConfig:
    map_dir: str = "/home/robot/.jszr/map"
    ros_setup: str = "/opt/ros/humble/setup.bash"
    workspace_setup: str = "~/genisom_roamerx_open/install/setup.bash"
    slam_command: str = "ros2 run robot_slam mapping --ros-args -p config:=$HOME/genisom_roamerx_open/install/robot_slam/share/robot_slam/config/config.yaml"
    service_name: str = "/slam_state_service"
    service_type: str = "robots_dog_msgs/srv/MapState"
    start_data: int = 3
    save_data: int = 5
    command_timeout_seconds: int = 15
    save_wait_seconds: int = 8


@dataclass
class EdgeConfig:
    robot: RobotConfig
    mqtt: MqttConfig
    ros: RosConfig
    telemetry: TelemetryConfig
    safety: SafetyConfig
    storage: StorageConfig
    media: MediaConfig
    mapping: MappingConfig

    @classmethod
    def load(cls, path: str | Path) -> "EdgeConfig":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        return cls(
            robot=RobotConfig(**raw["robot"]),
            mqtt=MqttConfig(**raw["mqtt"]),
            ros=RosConfig(**raw.get("ros", {})),
            telemetry=TelemetryConfig(**raw.get("telemetry", {})),
            safety=SafetyConfig(**raw.get("safety", {})),
            storage=StorageConfig(**raw.get("storage", {})),
            media=MediaConfig(**raw.get("media", {})),
            mapping=MappingConfig(**raw.get("mapping", {})),
        )
