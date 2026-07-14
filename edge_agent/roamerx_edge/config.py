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
    scan_matching_status_topic: str = "/status"
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
    final_waypoint_tolerance_m: float = 0.35


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
    save_wait_seconds: int = 7200
    dynamic_filter_enabled: bool = True
    dynamic_filter_voxel_size_m: float = 0.12
    dynamic_filter_min_points_per_voxel: int = 3
    dynamic_filter_clear_radius_cells: int = 1
    visibility_filter_enabled: bool = True
    visibility_filter_voxel_size_m: float = 0.3
    visibility_filter_min_free_observations: int = 1
    visibility_filter_max_hit_observations: int = 10
    visibility_filter_output_suffix: str = "keyframe_visibility_loose03x"
    auto_activate_uploaded_map: bool = True
    upload_point_cloud: bool = False
    preview_max_size: int = 1200
    map_set_enabled: bool = True
    submap_segment_length_m: float = 250.0
    submap_step_length_m: float = 190.0
    submap_margin_m: float = 12.0


@dataclass
class NavigationStackConfig:
    script_path: str = "/home/robot/genisom_roamerx_open/script/robot/start_navigation_real.sh"
    command_timeout_seconds: int = 45


@dataclass
class TeleopControlConfig:
    script_path: str = "/home/robot/genisom_roamerx_open/script/robot/start_teleop_control.sh"
    command_timeout_seconds: int = 20


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
    navigation_stack: NavigationStackConfig
    teleop_control: TeleopControlConfig

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
            navigation_stack=NavigationStackConfig(**raw.get("navigation_stack", {})),
            teleop_control=TeleopControlConfig(**raw.get("teleop_control", {})),
        )
