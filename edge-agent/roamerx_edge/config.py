from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .imu_cross_check import ImuCrossCheckConfig


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
    navigate_through_poses_action: str = "/navigate_through_poses"
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
    cooling_system_probe_interval_seconds: float = 15
    system_probe_stale_seconds: float = 30
    battery_ssh_host: str = "3588"
    modem_at_device: str = "/dev/ttyUSB2"
    charging_current_threshold_ma: int = 300
    charging_overheat_threshold_c: float = 49.0
    battery_rated_capacity_wh: float = 216.0
    legacy_charge_status_interval_seconds: float = 12.0
    charger_speaker_sink: str = "alsa_output.usb-SD_Audio_Device_2502171729-00.analog-stereo"
    nx_speaker_card: int = 0
    nx_speaker_control: str = "PCM"
    # The filesystem holding maps and rosbags, and where mapping_rosbag.sh
    # leaves its retention report. Both grow without an operator watching, so
    # occupancy and any deletion have to reach the platform.
    storage_probe_path: str = "/home/dogrobot/runtime/nx-edge/data"
    storage_retention_report_glob: str = "/home/dogrobot/runtime/nx-edge/data/rosbags/*/.retention.json"


@dataclass
class SafetyConfig:
    stop_speed_threshold_mps: float = 0.03
    stop_confirmation_seconds: float = 1.0
    localization_stable_seconds: float = 3.0
    localization_loss_samples: int = 5
    ndt_failure_score: float = 0.5
    ndt_failure_samples: int = 3
    localization_recovery_attempts: int = 3
    localization_recovery_retry_seconds: float = 5.0
    localization_recovery_cycle_seconds: float = 30.0
    # Number of full recovery cycles before the agent gives up and escalates.
    # 0 keeps the historical unbounded retry - sometimes standing still and waiting
    # for a human is the safest outcome - but the escalation alert still fires so the
    # situation is visible instead of a task silently paused forever.
    localization_recovery_max_cycles: int = 0
    standup_confirmation_timeout_seconds: float = 12.0
    low_battery_percent: int = 20
    final_waypoint_tolerance_m: float = 0.45
    docking_goal_tolerance_m: float = 0.08
    docking_goal_yaw_tolerance_rad: float = 0.0872665


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
class WaypointSpeechConfig:
    status_dir: str = "/home/dogrobot/runtime/nx-edge/data/audio-status"
    timeout_seconds: float = 120.0
    poll_interval_seconds: float = 0.2


@dataclass
class StorageConfig:
    sqlite_path: str = "data/edge.db"


@dataclass
class MediaConfig:
    upload_url: str = ""
    map_upload_url: str = ""
    device_id: str = ""
    device_key: str = ""
    map_upload_timeout_seconds: int = 1800


@dataclass
class MappingConfig:
    map_dir: str = "/home/dogrobot/runtime/nx-edge/data/jszr/map"
    log_dir: str = "/tmp/roamerx_mapping_logs"
    ros_setup: str = "/opt/ros/humble/setup.bash"
    workspace_setup: str = "/home/dogrobot/robot/install/setup.bash"
    slam_binary: str = "/home/dogrobot/robot/install/robot_slam/lib/robot_slam/mapping"
    enu_binary: str = "/home/dogrobot/robot/install/robot_slam/lib/robot_slam/slam_enu_converter"
    slam_params_file: str = "/home/dogrobot/robot/install/robot_slam/share/robot_slam/config/config.yaml"
    unified_launch_file: str = "/home/dogrobot/robot/install/robot_slam/share/robot_slam/launch/unified_mapping.launch.py"
    mapping_unit_file: str = "/etc/systemd/system/roamerx-mapping.service"
    mapping_session_params_file: str = "/home/dogrobot/runtime/nx-edge/conf/mapping-origin-session.yaml"
    mapping_environment_file: str = "/home/dogrobot/runtime/nx-edge/conf/mapping-session.env"
    slam_command: str = (
        "exec ros2 launch robot_slam unified_mapping.launch.py"
    )
    mapping_unit: str = "roamerx-mapping.service"
    deployment_manifest: str = "/home/dogrobot/runtime/nx-edge/conf/mapping-deployment.json"
    deployment_manifest_required: bool = False
    navigation_script: str = "/home/dogrobot/robot/script/robot/start_navigation_real.sh"
    service_name: str = "/slam_state_service"
    service_type: str = "robots_dog_msgs/srv/MapState"
    start_service: str = "/slam/start_mapping"
    start_service_type: str = "std_srvs/srv/Trigger"
    save_service: str = "/slam/save_map"
    save_service_type: str = "std_srvs/srv/Trigger"
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
    # Scan Context detection and diagnostics always run after a save. Applying
    # accepted loop closures to the factor graph is an explicit policy choice;
    # keep it off until Map Management has reviewed the candidate thresholds.
    auto_loop_optimization_enabled: bool = False
    localization_scan_context_check_binary: str = (
        "/home/dogrobot/robot/install/localization/lib/localization/"
        "localization_scan_context_check"
    )
    pcd2grid_binary: str = (
        "/home/dogrobot/robot/install/robot_slam/lib/robot_slam/pcd2grid_streaming"
    )
    auto_activate_uploaded_map: bool = True
    # A cloud map must remain usable for 3D NDT localization on a robot that
    # does not already have the source session directory.
    upload_point_cloud: bool = True
    preview_max_size: int = 1200
    rosbag_script: str = "/home/dogrobot/robot/script/robot/mapping_rosbag.sh"
    rosbag_stop_timeout_seconds: int = 45
    divergence_post_record_seconds: float = 3.0
    sensor_start_script: str = "/home/dogrobot/robot/script/robot/ensure_mapping_sensors.sh"
    sensor_start_timeout_seconds: int = 40
    warmup_data: int = 6
    indoor_warmup_data: int = 7
    origin_file: str = ""
    origin_state_file: str = ""
    origin_lock_duration_seconds: float = 10.0
    origin_lock_max_spread_m: float = 0.02
    origin_lock_sample_interval_seconds: float = 1.0
    origin_lock_no_signal_timeout_seconds: float = 3.0
    origin_topic_timeout_seconds: float = 1.0
    origin_lock_ttl_seconds: int = 1800
    origin_fix_topic: str = "/fix"
    origin_rtk_topic: str = "/rtk_pvh"
    origin_ntrip_status_topic: str = "/rtk/ntrip_status"
    heading_min_baseline_m: float = 0.20
    heading_max_std_deg: float = 5.0
    heading_max_age_seconds: float = 1.5
    # The receiver reports the primary-to-secondary antenna baseline, which
    # points opposite base_link +X on the current robot installation.
    heading_offset_deg: float = 180.0


@dataclass
class NavigationStackConfig:
    script_path: str = "/home/dogrobot/robot/script/robot/start_navigation_real.sh"
    command_timeout_seconds: int = 45
    rosbag_script: str = "/home/dogrobot/robot/script/robot/navigation_rosbag.sh"
    rosbag_stop_timeout_seconds: int = 45


@dataclass
class TeleopControlConfig:
    script_path: str = "/home/dogrobot/robot/script/robot/start_teleop_control.sh"
    command_timeout_seconds: int = 20


@dataclass
class PersonFollowConfig:
    detection_snapshot_path: str = "/run/roamerx/person_detections.json"
    detection_stale_seconds: float = 1.0
    control_interval_seconds: float = 0.15
    obstacle_stop_distance_m: float = 0.8


@dataclass
class SensorControlConfig:
    lidar_restart_script: str = "/home/dogrobot/robot/script/robot/restart_livox_sensor.sh"
    rtk_restart_script: str = "/home/dogrobot/robot/script/robot/start_rtk_ntrip.sh"
    command_timeout_seconds: int = 30


@dataclass
class ChargeControlConfig:
    remote_host: str = "3588"
    service_name: str = "roamerx-charge-pile"
    working_directory: str = "/home/firefly/charge_pile_v1.0.3b/charge_pile_xg_lib_v1.0.3b/dog_send_three_states"
    executable: str = "/home/firefly/charge_pile_v1.0.3b/charge_pile_xg_lib_v1.0.3b/dog_send_three_states/dog_lying_down"
    return_executable: str = "/home/firefly/charge_pile_v1.0.3b/charge_pile_xg_lib_v1.0.3b/dog_send_three_states/dog_returning"
    controller_service_name: str = "robot-launch.service"
    arbiter_path: str = "/usr/local/sbin/roamerx-charge-pile-arbiter"
    arc_platform_egg: str = "arc_platform"
    ros_setup: str = "/opt/ros/humble/setup.bash"
    ros_domain_id: int = 24
    rmw_implementation: str = "rmw_zenoh_cpp"
    arc_state_topic: str = "/arc/arc_state"
    dock_state_topic: str = "/arc/dock_state"
    pile_command_topic: str = "/arc/pile_ele_cmd"
    pile_result_topic: str = "/arc/pile_ele_result"
    arc_command_timeout_seconds: int = 18
    command_timeout_seconds: int = 90
    full_battery_percent: int = 95
    full_confirmation_samples: int = 3
    thermal_recovery_delay_seconds: float = 10.0
    thermal_retry_cooldown_seconds: float = 60.0
    low_battery_start_percent: int = 20
    low_battery_rearm_percent: int = 25
    low_battery_confirmation_samples: int = 2
    low_battery_start_cooldown_seconds: float = 60.0
    manual_disconnect_auto_charge_pause_seconds: float = 300.0
    cooling_stop_eggs: tuple[str, ...] = (
        "push_image", "spline_daemon", "motion_control", "dog_task",
        "imu_daemon", "ecal2ros", "monitor", "zenoh_route",
    )
    normal_start_eggs: tuple[str, ...] = (
        "arc_platform", "monitor", "time_sync", "spline_daemon",
        "motion_control", "zenoh_route", "dog_task", "push_image",
        "ecal2ros", "imu_daemon",
    )
    cooling_stop_services: tuple[str, ...] = (
        "rkaiq_3A.service", "rknn_server.service", "lightdm.service",
    )
    normal_start_services: tuple[str, ...] = (
        "rkaiq_3A.service", "rknn_server.service", "lightdm.service",
    )


@dataclass
class PowerModeConfig:
    state_path: str = "/home/dogrobot/runtime/nx-edge/data/edge-agent/power_mode.json"
    cooling_marker_path: str = "/home/dogrobot/runtime/nx-edge/data/edge-agent/cooling_standby"
    monitoring_service: str = "roamerx-bike-bot.service"
    teleop_bridge_service: str = "roamerx-teleop-bridge.service"
    navigation_script: str = "/home/dogrobot/robot/script/robot/start_navigation_real.sh"
    normal_start_timeout_seconds: int = 240
    always_on_services: tuple[str, ...] = (
        "roamerx-dev-agent.service", "roamerx-robot-mcp.service",
        "roamerx-zenoh.service", "roamerx-5g-share.service",
    )
    controller_host: str = "3588"
    controller_always_eggs: tuple[str, ...] = (
        # Keep only battery telemetry alive while charging. All sensor and
        # motion related eggs are stopped in cooling standby.
        "power_daemon",
    )
    controller_runtime_eggs: tuple[str, ...] = (
        "time_sync", "arc_platform", "monitor", "spline_daemon", "motion_control",
        "zenoh_route", "dog_task", "push_image", "ecal2ros", "imu_daemon",
    )
    controller_always_services: tuple[str, ...] = (
        "robot-launch.service",
    )
    controller_runtime_services: tuple[str, ...] = (
        "rkaiq_3A.service", "rknn_server.service", "lightdm.service",
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
    waypoint_speech: WaypointSpeechConfig
    storage: StorageConfig
    media: MediaConfig
    mapping: MappingConfig
    navigation_stack: NavigationStackConfig
    teleop_control: TeleopControlConfig
    person_follow: PersonFollowConfig
    sensor_control: SensorControlConfig
    charge_control: ChargeControlConfig
    power_mode: PowerModeConfig
    audio_control: AudioControlConfig
    # Defaulted so existing constructions of EdgeConfig keep working; the
    # cross-check is pure monitoring and has no required configuration.
    imu_cross_check: ImuCrossCheckConfig = field(default_factory=ImuCrossCheckConfig)

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
            waypoint_speech=WaypointSpeechConfig(**raw.get("waypoint_speech", {})),
            storage=StorageConfig(**raw.get("storage", {})),
            media=MediaConfig(**raw.get("media", {})),
            mapping=MappingConfig(**raw.get("mapping", {})),
            navigation_stack=NavigationStackConfig(**raw.get("navigation_stack", {})),
            teleop_control=TeleopControlConfig(**raw.get("teleop_control", {})),
            person_follow=PersonFollowConfig(**raw.get("person_follow", {})),
            sensor_control=SensorControlConfig(**raw.get("sensor_control", {})),
            charge_control=ChargeControlConfig(**raw.get("charge_control", {})),
            power_mode=PowerModeConfig(**raw.get("power_mode", {})),
            audio_control=AudioControlConfig(**raw.get("audio_control", {})),
            imu_cross_check=ImuCrossCheckConfig(**raw.get("imu_cross_check", {})),
        )
