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
    scan_topic: str = "/laser_scan"
    cmd_vel_raw_topic: str = "/cmd_vel_raw"
    cmd_vel_topic: str = "/cmd_vel"


@dataclass
class TelemetryConfig:
    status_interval_seconds: float = 2
    heartbeat_interval_seconds: float = 10
    trajectory_flush_seconds: float = 5
    trajectory_batch_size: int = 20
    system_probe_interval_seconds: float = 10
    system_probe_stale_seconds: float = 30
    battery_ssh_host: str = "3588"
    modem_at_device: str = "/dev/ttyUSB2"
    charging_current_threshold_ma: int = 300
    charging_overheat_threshold_c: float = 45.0
    charger_speaker_sink: str = "alsa_output.usb-SD_Audio_Device_2502171729-00.analog-stereo"
    nx_speaker_card: int = 2
    nx_speaker_control: str = "PCM"


@dataclass
class SafetyConfig:
    stop_speed_threshold_mps: float = 0.03
    stop_confirmation_seconds: float = 1.0
    localization_stable_seconds: float = 3.0
    localization_loss_samples: int = 5
    standup_confirmation_timeout_seconds: float = 12.0
    low_battery_percent: int = 20
    final_waypoint_tolerance_m: float = 0.35


@dataclass
class ObstacleSpeechConfig:
    enabled: bool = True
    obstacle_clear_seconds: float = 3.0
    no_progress_seconds: float = 5.0
    min_progress_m: float = 0.5
    obstacle_max_distance_m: float = 0.9
    collision_limit_ratio: float = 0.6
    minimum_blocked_task_seconds: float = 300.0
    navigation_retry_seconds: float = 5.0


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
    log_dir: str = "/tmp/roamerx_mapping_logs"
    ros_setup: str = "/opt/ros/humble/setup.bash"
    workspace_setup: str = "~/genisom_roamerx_open/install/setup.bash"
    slam_command: str = "ros2 run robot_slam mapping --ros-args --params-file $HOME/genisom_roamerx_open/install/robot_slam/share/robot_slam/config/config.yaml"
    service_name: str = "/slam_state_service"
    service_type: str = "robots_dog_msgs/srv/MapState"
    start_data: int = 3
    save_data: int = 5
    command_timeout_seconds: int = 15
    save_wait_seconds: int = 7200
    dynamic_filter_enabled: bool = False
    dynamic_filter_voxel_size_m: float = 0.12
    dynamic_filter_min_points_per_voxel: int = 3
    dynamic_filter_clear_radius_cells: int = 1
    visibility_filter_enabled: bool = False
    visibility_filter_voxel_size_m: float = 0.3
    visibility_filter_min_free_observations: int = 1
    visibility_filter_max_hit_observations: int = 10
    visibility_filter_max_source_bytes: int = 268435456
    visibility_filter_output_suffix: str = "keyframe_visibility_loose03x"
    auto_activate_uploaded_map: bool = True
    upload_point_cloud: bool = False
    preview_max_size: int = 1200
    rosbag_script: str = "/home/robot/genisom_roamerx_open/script/robot/mapping_rosbag.sh"
    rosbag_stop_timeout_seconds: int = 45
    sensor_start_script: str = "/home/robot/genisom_roamerx_open/script/robot/ensure_mapping_sensors.sh"
    sensor_start_timeout_seconds: int = 40


@dataclass
class NavigationStackConfig:
    script_path: str = "/home/robot/genisom_roamerx_open/script/robot/start_navigation_real.sh"
    command_timeout_seconds: int = 45
    rosbag_script: str = "/home/robot/genisom_roamerx_open/script/robot/navigation_rosbag.sh"
    rosbag_stop_timeout_seconds: int = 45


@dataclass
class TeleopControlConfig:
    script_path: str = "/home/robot/genisom_roamerx_open/script/robot/start_teleop_control.sh"
    command_timeout_seconds: int = 20


@dataclass
class SensorControlConfig:
    lidar_restart_script: str = "/home/robot/genisom_roamerx_open/script/robot/restart_livox_sensor.sh"
    rtk_restart_script: str = "/home/robot/genisom_roamerx_open/script/robot/start_rtk_ntrip.sh"
    command_timeout_seconds: int = 30


@dataclass
class ChargeControlConfig:
    remote_host: str = "3588"
    service_name: str = "roamerx-charge-pile"
    working_directory: str = "/home/firefly/charge_pile_v1.0.3b/charge_pile_xg_lib_v1.0.3b/dog_send_three_states"
    executable: str = "/home/firefly/charge_pile_v1.0.3b/charge_pile_xg_lib_v1.0.3b/dog_send_three_states/dog_lying_down"
    command_timeout_seconds: int = 35
    full_battery_percent: int = 100
    full_confirmation_samples: int = 3
    cooling_stop_eggs: tuple[str, ...] = (
        "push_image", "spline_daemon", "motion_control", "dog_task",
        "imu_daemon", "ecal2ros", "monitor", "zenoh_route",
    )
    normal_start_eggs: tuple[str, ...] = (
        "zenoh_route", "imu_daemon", "ecal2ros", "motion_control",
        "spline_daemon", "dog_task", "monitor", "push_image",
    )


@dataclass
class PowerModeConfig:
    state_path: str = "/home/robot/edge_agent/data/power_mode.json"
    cooling_marker_path: str = "/home/robot/edge_agent/data/cooling_standby"
    monitoring_service: str = "roamerx-bike-bot.service"
    navigation_script: str = "/home/robot/genisom_roamerx_open/script/robot/start_navigation_real.sh"
    sensor_start_script: str = "/home/robot/genisom_roamerx_open/script/robot/ensure_navigation_sensors.sh"
    cooling_power_mode: int = 1
    normal_power_mode: int = 0
    normal_start_timeout_seconds: int = 120
    reboot_delay_seconds: int = 12
    controller_host: str = "3588"
    controller_runtime_eggs: tuple[str, ...] = (
        "push_image", "spline_daemon", "motion_control", "dog_task",
        "imu_daemon", "ecal2ros", "monitor", "zenoh_route",
    )


@dataclass
class AudioControlConfig:
    remote_host: str = "3588"
    remote_sink: str = "alsa_output.usb-SD_Audio_Device_2502171729-00.analog-stereo"
    nx_card: int = 2
    nx_control: str = "PCM"
    command_timeout_seconds: int = 10


@dataclass
class EdgeConfig:
    robot: RobotConfig
    mqtt: MqttConfig
    ros: RosConfig
    telemetry: TelemetryConfig
    safety: SafetyConfig
    obstacle_speech: ObstacleSpeechConfig
    storage: StorageConfig
    media: MediaConfig
    mapping: MappingConfig
    navigation_stack: NavigationStackConfig
    teleop_control: TeleopControlConfig
    sensor_control: SensorControlConfig
    charge_control: ChargeControlConfig
    power_mode: PowerModeConfig
    audio_control: AudioControlConfig

    @classmethod
    def load(cls, path: str | Path) -> "EdgeConfig":
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        return cls(
            robot=RobotConfig(**raw["robot"]),
            mqtt=MqttConfig(**raw["mqtt"]),
            ros=RosConfig(**raw.get("ros", {})),
            telemetry=TelemetryConfig(**raw.get("telemetry", {})),
            safety=SafetyConfig(**raw.get("safety", {})),
            obstacle_speech=ObstacleSpeechConfig(**raw.get("obstacle_speech", {})),
            storage=StorageConfig(**raw.get("storage", {})),
            media=MediaConfig(**raw.get("media", {})),
            mapping=MappingConfig(**raw.get("mapping", {})),
            navigation_stack=NavigationStackConfig(**raw.get("navigation_stack", {})),
            teleop_control=TeleopControlConfig(**raw.get("teleop_control", {})),
            sensor_control=SensorControlConfig(**raw.get("sensor_control", {})),
            charge_control=ChargeControlConfig(**raw.get("charge_control", {})),
            power_mode=PowerModeConfig(**raw.get("power_mode", {})),
            audio_control=AudioControlConfig(**raw.get("audio_control", {})),
        )
