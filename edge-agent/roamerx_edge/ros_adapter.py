from __future__ import annotations

import logging
import json
import math
import threading
import time
from collections import deque
from typing import Callable

from .config import MappingConfig, RosConfig, SafetyConfig
from .imu_cross_check import ImuCrossCheck, ImuCrossCheckConfig
from .protocol import ProtocolError
from .rtk_origin import RtkOriginPayloadCache
from .safety_policy import RuntimeSafetyState
from .telemetry_collector import TelemetryCollector

LOGGER = logging.getLogger(__name__)


def follow_path_patrol_params(
    *,
    final_approach: bool,
    local_obstacles: bool,
) -> dict[str, bool | float]:
    """MPPI settings for a patrol goal.

    Cruise must not hug a slightly jagged through-poses polyline: PathAlign plus
    path orientations turns click noise into left/right steering. CostCritic is
    only useful when the local obstacle layer is actually painting. Final
    approach slows down so the DiffDrive turning radius fits the 0.35 m window.
    """
    vx_max = 0.15 if final_approach else 0.30
    vx_min = -0.15 if final_approach else -0.12
    wz_max = 0.50 if final_approach else 0.35
    return {
        "FollowPath.vx_max": vx_max,
        "FollowPath.vx_min": vx_min,
        "FollowPath.vy_max": 0.5,
        "FollowPath.wz_max": wz_max,
        "FollowPath.wz_std": 0.08,
        "FollowPath.GoalCritic.enabled": bool(final_approach),
        "FollowPath.GoalAngleCritic.enabled": False,
        "FollowPath.PreferForwardCritic.enabled": not final_approach,
        "FollowPath.CostCritic.enabled": bool(local_obstacles),
        "FollowPath.PathAlignCritic.enabled": bool(final_approach),
        "FollowPath.PathAlignCritic.offset_from_furthest": 4,
        "FollowPath.PathAlignCritic.use_path_orientations": False,
        "FollowPath.PathFollowCritic.enabled": True,
        "FollowPath.PathAngleCritic.enabled": True,
        "FollowPath.PathAngleCritic.max_angle_to_furthest": 0.40,
    }

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import FollowWaypoints, NavigateThroughPoses
    from nav_msgs.msg import Odometry
    from action_msgs.srv import CancelGoal
    from lifecycle_msgs.srv import GetState
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan, NavSatFix
    from robots_dog_msgs.msg import Localization, UniRtkPvh
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from rcl_interfaces.msg import Parameter as ParameterMessage, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    Node = object

try:
    if ROS_AVAILABLE:
        from robots_dog_msgs.msg import HighLevelRobotState
    else:
        HighLevelRobotState = None
except ImportError:
    # Carries the 3588 BMI088 gyro. Only used for cross-checking the lidar IMU,
    # so a missing definition must not cost the agent anything else.
    HighLevelRobotState = None

try:
    if ROS_AVAILABLE:
        from localization.msg import ScanMatchingStatus
    else:
        ScanMatchingStatus = None
except ImportError:
    # Localization quality telemetry is useful but must not take the complete
    # Edge Agent (including mapping/RTK control) offline while a ROS install is
    # being upgraded or repaired.
    ScanMatchingStatus = None


class RosAdapter(Node):
    def __init__(
        self,
        ros_config: RosConfig,
        safety_config: SafetyConfig,
        telemetry: TelemetryCollector,
        safety_state: RuntimeSafetyState,
        mapping_config: MappingConfig | None = None,
        imu_cross_check_config: ImuCrossCheckConfig | None = None,
    ) -> None:
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS2 Python packages are not available")
        super().__init__("roamerx_edge_agent")
        self.ros_config = ros_config
        self.safety_config = safety_config
        self.telemetry = telemetry
        self.safety_state = safety_state
        self._goal_handle = None
        self._nav_cancel_action = ros_config.follow_waypoints_action
        self._sent_pose_count = 0
        self._result_cb: Callable | None = None
        self._feedback_cb: Callable | None = None
        self._localization_failure_cb: Callable | None = None
        self._localization_recovery_cb: Callable | None = None
        self._trusted_pose_cb: Callable | None = None
        self._mapping_divergence_cb: Callable | None = None
        self._imu_cross_check_cb: Callable | None = None
        self._imu_cross_check = ImuCrossCheck(imu_cross_check_config)
        self._imu_cross_check_lock = threading.Lock()
        self._last_trusted_pose: dict | None = None
        self._last_trusted_pose_report_monotonic = 0.0
        self._localization_sample_condition = threading.Condition()
        self._localization_sample_sequence = 0
        self._localization_status_samples = deque(maxlen=100)
        self._localization_lost_count = 0
        self._localization_failure_notified = False
        self._lio_motion_anomaly_notified = False
        self._ndt_failure_count = 0
        self._ndt_failure_notified = False
        self._localization_recovery_pending = False
        self._localization_recovery_armed = False
        self._latest_speed = 0.0
        self._raw_forward_command = 0.0
        self._actual_forward_command = 0.0
        self._raw_lateral_command = 0.0
        self._actual_lateral_command = 0.0
        self._raw_turn_command = 0.0
        self._actual_turn_command = 0.0
        self._front_obstacle_distance_m = None
        self._robot_motion_state = "unknown"
        self._robot_motion_state_sequence = 0
        self._robot_motion_condition = threading.Condition()
        self._robot_standing_event = threading.Event()
        self._remote_control_event = threading.Event()
        # The telemetry loop and MQTT command threads both probe Nav2.  rclpy
        # action/service clients are not safe to drive through overlapping
        # blocking discovery calls on the same node; concurrent probes can
        # therefore report every lifecycle node as unavailable even while an
        # external ROS probe sees the complete stack active.
        self._nav_ready_probe_lock = threading.Lock()
        self._rtk_origin_cache = RtkOriginPayloadCache(
            stale_after_seconds=(mapping_config.heading_max_age_seconds if mapping_config else 1.5)
        )
        self.create_subscription(
            Localization,
            ros_config.localization_topic,
            self._on_localization,
            10,
        )
        self.create_subscription(
            Odometry,
            ros_config.odometry_topic,
            self._on_odometry,
            20,
        )
        self.create_subscription(Twist, ros_config.cmd_vel_raw_topic, self._on_cmd_vel_raw, 10)
        self.create_subscription(Twist, ros_config.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(LaserScan, ros_config.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, "/sensor_health", self._on_sensor_health, 2)
        self.create_subscription(String, "/localization/decision", self._on_localization_decision, 10)
        self._subscribe_imu_cross_check()
        if mapping_config is not None:
            self.create_subscription(
                NavSatFix,
                mapping_config.origin_fix_topic,
                self._on_origin_fix,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                UniRtkPvh,
                mapping_config.origin_rtk_topic,
                self._on_origin_pvh,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String,
                mapping_config.origin_ntrip_status_topic,
                self._on_origin_ntrip,
                qos_profile_sensor_data,
            )
            divergence_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(
                String, "/slam/divergence_event", self._on_mapping_divergence_event, divergence_qos
            )
        if ros_config.scan_matching_status_topic and ScanMatchingStatus is not None:
            self.create_subscription(
                ScanMatchingStatus,
                ros_config.scan_matching_status_topic,
                self._on_scan_matching_status,
                qos_profile_sensor_data,
            )
        elif ros_config.scan_matching_status_topic:
            LOGGER.warning(
                "localization ScanMatchingStatus message is unavailable; quality telemetry subscription is disabled"
            )
        self._action_client = ActionClient(self, FollowWaypoints, ros_config.follow_waypoints_action)
        through_poses_action = getattr(
            ros_config, "navigate_through_poses_action", "/navigate_through_poses"
        )
        self._through_poses_client = ActionClient(self, NavigateThroughPoses, through_poses_action)
        self._through_poses_action = through_poses_action
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 8)
        self._rtk_initial_pose_client = self.create_client(Trigger, "/localization/seed_from_rtk")
        self._global_relocalize_client = self.create_client(
            Trigger, "/localization/global_relocalize"
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._teleop_cmd_vel_pub = self.create_publisher(Twist, "/teleop_cmd_vel", 10)
        self._teleop_action_pub = self.create_publisher(String, "/teleop_action", 10)
        self._remote_teleop_action_pub = self.create_publisher(String, "/remote_teleop_action", 10)
        self._localization_policy_pub = self.create_publisher(String, "/localization/policy", 10)
        goal_yaw_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._goal_yaw_required_pub = self.create_publisher(Bool, "/navigation/require_goal_yaw", goal_yaw_qos)
        self._fine_control_pub = self.create_publisher(Bool, "/navigation/fine_control", goal_yaw_qos)
        self.create_subscription(String, "/robot_motion_state", self._on_robot_motion_state, 10)

    def _on_robot_motion_state(self, msg) -> None:
        with self._robot_motion_condition:
            self._robot_motion_state = str(msg.data)
            self._robot_motion_state_sequence += 1
            self._robot_motion_condition.notify_all()
        if self._robot_motion_state == "standing":
            self._robot_standing_event.set()
        elif self._robot_motion_state == "remote_control":
            self._remote_control_event.set()

    def _on_localization_decision(self, msg) -> None:
        try:
            payload = json.loads(str(msg.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            LOGGER.warning("invalid /localization/decision payload")
            return
        if not isinstance(payload, dict):
            return
        self.telemetry.on_localization_decision(payload)
        anomaly = bool(payload.get("lio_motion_anomaly"))
        if not anomaly:
            self._lio_motion_anomaly_notified = False
            return
        if (
            self._lio_motion_anomaly_notified
            or not self._localization_failure_cb
        ):
            return
        self._lio_motion_anomaly_notified = True
        self._localization_failure_notified = True
        self._localization_recovery_armed = True
        threading.Thread(
            target=self._localization_failure_cb,
            args=("lio_motion_anomaly",),
            daemon=True,
            name="lio-motion-anomaly-handler",
        ).start()

    @staticmethod
    def _header_payload(header) -> dict:
        stamp = getattr(header, "stamp", None)
        return {
            "stamp": {
                "sec": int(getattr(stamp, "sec", 0)),
                "nanosec": int(getattr(stamp, "nanosec", 0)),
            },
            "frame_id": str(getattr(header, "frame_id", "") or ""),
        }

    def _on_origin_fix(self, msg) -> None:
        self._rtk_origin_cache.update(
            "fix",
            {
                "header": self._header_payload(getattr(msg, "header", None)),
                "status": {"status": int(getattr(getattr(msg, "status", None), "status", -1))},
                "latitude": float(getattr(msg, "latitude", math.nan)),
                "longitude": float(getattr(msg, "longitude", math.nan)),
                "altitude": float(getattr(msg, "altitude", math.nan)),
            },
        )

    def _on_origin_pvh(self, msg) -> None:
        bestnav = getattr(msg, "bestnav", None)
        heading = getattr(msg, "heading", None)
        self._rtk_origin_cache.update(
            "pvh",
            {
                "header": self._header_payload(getattr(msg, "header", None)),
                "bestnav": {
                    "p_sol_status": int(getattr(bestnav, "p_sol_status", -1)),
                    "pos_type": int(getattr(bestnav, "pos_type", 0)),
                    "latitude_deg": float(getattr(bestnav, "latitude_deg", math.nan)),
                    "longitude_deg": float(getattr(bestnav, "longitude_deg", math.nan)),
                    "altitude_m": float(getattr(bestnav, "altitude_m", math.nan)),
                    "lat_std": float(getattr(bestnav, "lat_std", math.inf)),
                    "lon_std": float(getattr(bestnav, "lon_std", math.inf)),
                    "hgt_std": float(getattr(bestnav, "hgt_std", math.inf)),
                    "diff_age_s": float(getattr(bestnav, "diff_age_s", math.inf)),
                    "svs_num": int(getattr(bestnav, "svs_num", 0)),
                    "soln_svs_num": int(getattr(bestnav, "soln_svs_num", 0)),
                },
                "heading": {
                    "sol_status": int(getattr(heading, "sol_status", -1)),
                    "heading_type": int(getattr(heading, "heading_type", 0)),
                    "base_line": float(getattr(heading, "base_line", 0.0)),
                    "heading_deg": float(getattr(heading, "heading_deg", 0.0)),
                    "heading_std": float(getattr(heading, "heading_std", math.inf)),
                    "pitch_deg": float(getattr(heading, "pitch_deg", 0.0)),
                    "pitch_std": float(getattr(heading, "pitch_std", math.inf)),
                    "svs_num": int(getattr(heading, "svs_num", 0)),
                    "soln_svs_num": int(getattr(heading, "soln_svs_num", 0)),
                },
            },
        )

    def _on_origin_ntrip(self, msg) -> None:
        self._rtk_origin_cache.update("ntrip", {"data": str(getattr(msg, "data", "") or "")})

    def origin_payload_snapshot(self) -> tuple[dict, dict, dict, float]:
        return self._rtk_origin_cache.snapshot()

    def _on_odometry(self, msg) -> None:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        stamp_seconds = 0.0
        if stamp is not None:
            stamp_seconds = float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) / 1e9
        now_seconds = self.get_clock().now().nanoseconds / 1e9
        offset = now_seconds - stamp_seconds if stamp_seconds > 0 else None
        child_frame = str(getattr(msg, "child_frame_id", "") or "")
        source = "slam" if child_frame == "body" else "localization"
        self.telemetry.on_sensor_message(
            "odometry",
            topic=self.ros_config.odometry_topic,
            frame_id=str(getattr(getattr(msg, "header", None), "frame_id", "") or ""),
            child_frame_id=child_frame,
            source=source,
            measurement_stamp=stamp_seconds or None,
            message_time_offset_seconds=round(offset, 6) if offset is not None else None,
            message_time_valid=bool(stamp_seconds > 0 and offset is not None and abs(offset) <= 5.0),
        )

    def set_localization_policy(self, source: str, phase: str) -> dict:
        normalized_source = str(source).strip().lower()
        source = normalized_source if normalized_source in {"ndt", "rtk", "ukf"} else "ndt"
        phase = "moving" if str(phase).lower() == "moving" else "stationary"
        msg = String()
        msg.data = f"{phase}:{source}"
        self._localization_policy_pub.publish(msg)
        return {"topic": "/localization/policy", "source": source, "phase": phase}

    def localization_decision(self) -> dict:
        return self.telemetry.localization_decision()

    def localization_diagnostics(self) -> dict:
        return self.telemetry.localization_diagnostics()

    def prepare_for_navigation(self, timeout_seconds: float = 12.0) -> bool:
        """Stand the robot and wait for the SDK bridge to confirm it is stable."""
        self._robot_standing_event.clear()
        self.teleop_action("stand_up")
        confirmed = self._robot_standing_event.wait(timeout=max(0.0, timeout_seconds))
        if confirmed:
            LOGGER.info("robot standing confirmed; navigation may start")
            return True
        LOGGER.error(
            "robot stand-up was not confirmed within %.1fs (last state=%s)",
            timeout_seconds,
            self._robot_motion_state,
        )
        # No Nav2 goal has been sent yet. Explicitly return the SDK bridge to
        # passive so its stand-up retry loop cannot continue after the task is
        # rejected.
        self.teleop_action("passive")
        return False

    def _on_localization(self, msg) -> None:
        self._latest_speed = float(msg.speed)
        self.telemetry.on_localization(msg)
        status = int(msg.status)
        with self._localization_sample_condition:
            self._localization_sample_sequence += 1
            self._localization_status_samples.append(
                (self._localization_sample_sequence, status)
            )
            self._localization_sample_condition.notify_all()
        if status != 3:
            self._localization_lost_count += 1
        else:
            self._localization_lost_count = 0
            self._localization_failure_notified = False
            latest = self.telemetry.latest_pose()
            absolute_stable = self._absolute_localization_stable()
            if latest and absolute_stable:
                self._last_trusted_pose = {
                    "x": float(latest.x),
                    "y": float(latest.y),
                    "z": float(latest.z),
                    "yaw": float(latest.yaw),
                    "sampled_at": latest.sampled_at,
                    "frame_id": "map",
                }
            stable_for = time.monotonic() - self.safety_state.localization_normal_since_monotonic
            if (
                self._trusted_pose_cb
                and absolute_stable
                and stable_for >= self.safety_config.localization_stable_seconds
                and time.monotonic() - self._last_trusted_pose_report_monotonic >= 5.0
            ):
                if latest:
                    self._trusted_pose_cb(latest)
                    self._last_trusted_pose_report_monotonic = time.monotonic()
            if (
                self._localization_recovery_armed
                and not self._localization_recovery_pending
                and self._localization_recovery_cb
            ):
                self._localization_recovery_pending = True
                threading.Thread(
                    target=self._notify_localization_recovery_when_stable,
                    daemon=True,
                    name="localization-recovery-handler",
                ).start()

        if (
            self._localization_lost_count >= max(1, self.safety_config.localization_loss_samples)
            and not self._localization_failure_notified
            and self._localization_failure_cb
        ):
            self._localization_failure_notified = True
            self._localization_recovery_armed = True
            threading.Thread(
                target=self._localization_failure_cb,
                args=("localization_lost",),
                daemon=True,
                name="localization-loss-handler",
            ).start()

    def set_localization_failure_callback(self, callback: Callable) -> None:
        self._localization_failure_cb = callback

    def set_localization_recovery_callback(self, callback: Callable) -> None:
        self._localization_recovery_cb = callback

    def set_trusted_pose_callback(self, callback: Callable) -> None:
        self._trusted_pose_cb = callback

    def set_mapping_divergence_callback(self, callback: Callable) -> None:
        self._mapping_divergence_cb = callback

    def _on_mapping_divergence_event(self, msg) -> None:
        if not self._mapping_divergence_cb:
            return
        try:
            event = json.loads(str(msg.data or "{}"))
        except (TypeError, ValueError):
            event = {"reason": str(msg.data or "invalid divergence event")}
        threading.Thread(
            target=self._mapping_divergence_cb,
            args=(event,),
            daemon=True,
            name="mapping-safe-hold-handler",
        ).start()

    def latest_trusted_pose(self) -> dict | None:
        return dict(self._last_trusted_pose) if self._last_trusted_pose else None

    def _notify_localization_recovery_when_stable(self) -> None:
        try:
            while True:
                if self.safety_state.localization_status != "normal":
                    return
                stable_for = time.monotonic() - self.safety_state.localization_normal_since_monotonic
                remaining = self.safety_config.localization_stable_seconds - stable_for
                if remaining <= 0.0 and self._absolute_localization_stable():
                    if self._localization_recovery_cb:
                        self._localization_recovery_cb()
                    self._localization_recovery_armed = False
                    return
                time.sleep(0.2 if remaining <= 0.0 else min(0.2, remaining))
        finally:
            self._localization_recovery_pending = False

    def _on_scan_matching_status(self, msg) -> None:
        LOGGER.debug(
            "scan matching status: converged=%s matching_error=%.3f inlier_fraction=%.3f",
            bool(getattr(msg, "has_converged", False)),
            float(getattr(msg, "matching_error", 0.0)),
            float(getattr(msg, "inlier_fraction", 0.0)),
        )
        self.telemetry.on_scan_matching_status(msg)
        # Outdoor RTK-primary navigation still publishes NDT as a shadow health
        # check. Open sky often has an empty/poor scan match even when GPS pose
        # is centimetre-grade. That must not pause the task as localization loss.
        # Indoor LIO-primary is different: status can stay 3 while the published
        # pose has left the map, which is when the dog starts walking randomly.
        if self._rtk_is_navigation_pose_source():
            self._ndt_failure_count = 0
            self._ndt_failure_notified = False
            return
        decision = self.telemetry.localization_decision()
        correction_policy = str(decision.get("correction_policy") or "ndt").lower()
        if (
            decision.get("active_source") == "lio_imu"
            and correction_policy in {"rtk", "ukf"}
            and decision.get("policy_source_ready") is True
            and decision.get("rtk_usable") is True
            and str(decision.get("rtk_quality") or "").lower() == "fixed"
        ):
            # FAST-LIO remains the continuous pose source. In RTK mode, or in
            # UKF mode when fixed RTK is available, NDT is only an unused
            # auxiliary candidate and its degradation must not pause motion.
            self._ndt_failure_count = 0
            self._ndt_failure_notified = False
            return
        score = float(getattr(msg, "matching_error", float("inf")))
        healthy = bool(getattr(msg, "has_converged", False)) and math.isfinite(score) and (
            score < self.safety_config.ndt_failure_score
        )
        if healthy:
            self._ndt_failure_count = 0
            self._ndt_failure_notified = False
            return
        self._ndt_failure_count += 1
        if (
            self._ndt_failure_count >= max(1, self.safety_config.ndt_failure_samples)
            and not self._ndt_failure_notified
            and self._localization_failure_cb
        ):
            self._ndt_failure_notified = True
            self._localization_recovery_armed = True
            threading.Thread(
                target=self._localization_failure_cb,
                args=("ndt_degraded",),
                daemon=True,
                name="ndt-failure-handler",
            ).start()

    def _rtk_is_navigation_pose_source(self) -> bool:
        decision = self.telemetry.localization_decision()
        return (
            decision.get("active_source") == "rtk_imu"
            and decision.get("rtk_usable") is True
        )

    def _lio_is_navigation_pose_source(self) -> bool:
        decision = self.telemetry.localization_decision()
        return decision.get("active_source") == "lio_imu"

    def _absolute_localization_stable(self) -> bool:
        decision = self.telemetry.localization_decision()
        policy_source_ready = decision.get("policy_source_ready")
        return bool(
            decision.get("active_source") in {"ndt_imu", "rtk_imu", "lio_imu"}
            and decision.get("absolute_stable")
            and (policy_source_ready is None or policy_source_ready is True)
        )

    def _on_cmd_vel_raw(self, msg) -> None:
        self._raw_forward_command = float(msg.linear.x)
        self._raw_lateral_command = float(msg.linear.y)
        self._raw_turn_command = float(msg.angular.z)

    def _on_cmd_vel(self, msg) -> None:
        self._actual_forward_command = float(msg.linear.x)
        self._actual_lateral_command = float(msg.linear.y)
        self._actual_turn_command = float(msg.angular.z)

    def _on_scan(self, scan) -> None:
        nearest = None
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or distance < scan.range_min or distance > 0.9:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            if x >= 0.18 and abs(y) <= 0.35:
                nearest = distance if nearest is None else min(nearest, distance)
        self._front_obstacle_distance_m = nearest

    def _subscribe_imu_cross_check(self) -> None:
        """Subscribe both gyro feeds, if the pieces for the comparison exist.

        Pure monitoring: nothing here feeds the localization or control path,
        so every missing piece degrades to "no comparison" instead of an error.
        """
        config = self._imu_cross_check.config
        if not config.enabled:
            return
        self.create_subscription(
            Imu, config.lidar_imu_topic, self._on_lidar_imu, qos_profile_sensor_data
        )
        if HighLevelRobotState is None:
            LOGGER.warning(
                "robots_dog_msgs HighLevelRobotState is unavailable; "
                "the 3588 IMU cross-check will only report the lidar side"
            )
            return
        self.create_subscription(
            HighLevelRobotState,
            config.robot_state_topic,
            self._on_high_level_robot_state,
            qos_profile_sensor_data,
        )

    def _on_lidar_imu(self, msg) -> None:
        # Also the evaluation trigger: this feed runs at 200 Hz and is always
        # present, so driving the comparison from here is what lets a missing
        # 3588 stream be reported as absent rather than simply going quiet.
        gyro = msg.angular_velocity
        now = time.monotonic()
        with self._imu_cross_check_lock:
            self._imu_cross_check.add_lidar_gyro(now, gyro.x, gyro.y, gyro.z)
            if not self._imu_cross_check.due(now):
                return
            report = self._imu_cross_check.evaluate(now)
        self._publish_imu_cross_check(report)

    def _on_high_level_robot_state(self, msg) -> None:
        gyro = msg.gyro
        with self._imu_cross_check_lock:
            self._imu_cross_check.add_body_gyro(time.monotonic(), gyro.x, gyro.y, gyro.z)

    def _publish_imu_cross_check(self, report: dict) -> None:
        try:
            self.telemetry.on_imu_cross_check(report)
        except Exception:
            LOGGER.exception("failed to record IMU cross-check telemetry")
        if self._imu_cross_check_cb is None:
            return
        try:
            # Every report, not just the bad ones: the consumer needs the
            # healthy ones to know when to re-arm its alert.
            self._imu_cross_check_cb(report)
        except Exception:
            LOGGER.exception("IMU cross-check callback failed")

    def set_imu_cross_check_callback(self, callback: Callable) -> None:
        self._imu_cross_check_cb = callback

    def _on_sensor_health(self, msg) -> None:
        try:
            sensors = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            LOGGER.warning("invalid /sensor_health payload")
            return
        for name, details in sensors.items():
            if isinstance(details, dict):
                if str(name) == "odometry":
                    self.telemetry.update_sensor_details(str(name), **details)
                else:
                    self.telemetry.on_sensor_message(str(name), **details)

    def obstacle_monitor_snapshot(self) -> dict:
        """Navigation demand and filtered front-scan state for task recovery."""
        return {
            "requested_forward_speed_mps": self._raw_forward_command,
            "actual_forward_speed_mps": self._actual_forward_command,
            "requested_planar_speed_mps": math.hypot(self._raw_forward_command, self._raw_lateral_command),
            "actual_planar_speed_mps": math.hypot(self._actual_forward_command, self._actual_lateral_command),
            "requested_turn_speed_rps": self._raw_turn_command,
            "actual_turn_speed_rps": self._actual_turn_command,
            "localized_speed_mps": self._latest_speed,
            "front_obstacle_distance_m": self._front_obstacle_distance_m,
        }

    _NAV_READY_SERVER_PROBE_SECONDS = 0.5

    @staticmethod
    def _action_server_wait_timeout(timeout_seconds: float, remaining: float) -> float:
        """Keep a real discovery window even when the caller asked for 0s.

        The periodic health reporter uses timeout 0 to mean "one probe".  Passing
        that 0 through to ``wait_for_server`` makes Zenoh miss a freshly restarted
        FollowWaypoints server, so the UI stays on Nav2 not ready.
        """
        if timeout_seconds <= 0:
            return RosAdapter._NAV_READY_SERVER_PROBE_SECONDS
        return min(RosAdapter._NAV_READY_SERVER_PROBE_SECONDS, max(0.0, remaining))

    def wait_until_ready(self, timeout_seconds: float = 10.0) -> bool:
        """Serialize Nav2 discovery probes made by background threads."""
        with self._nav_ready_probe_lock:
            return self._wait_until_ready_probe(timeout_seconds)

    def _wait_until_ready_probe(self, timeout_seconds: float) -> bool:
        """Wait for Nav2's action server *and* all required lifecycle nodes.

        ``wait_for_server`` alone is not sufficient after a map switch: ROS
        discovery can still expose the previous FollowWaypoints server while
        the replacement Nav2 stack is only configuring.  Such a goal is
        accepted by discovery then immediately rejected by the action server.
        """
        deadline = time.monotonic() + max(0.0, timeout_seconds)
        required_nodes = (
            "/controller_server",
            "/planner_server",
            "/bt_navigator",
            "/waypoint_follower",
        )
        previous = bool(self.safety_state.nav_ready)
        # A zero timeout is used by the periodic health reporter.  It must
        # still perform one probe with a short discovery window rather than
        # immediately reporting Nav2 as unavailable.
        while True:
            remaining = max(0.0, deadline - time.monotonic())
            server_timeout = self._action_server_wait_timeout(timeout_seconds, remaining)
            if not self._action_client.wait_for_server(timeout_sec=server_timeout):
                ready = False
            else:
                ready = all(self._lifecycle_node_is_active(name) for name in required_nodes)
            if ready:
                self.safety_state.nav_ready = True
                if not previous:
                    LOGGER.info("Nav2 ready: FollowWaypoints and lifecycle nodes are active")
                return True
            if timeout_seconds <= 0 or time.monotonic() >= deadline:
                break
            time.sleep(0.2)
        self.safety_state.nav_ready = False
        if previous:
            LOGGER.warning("Nav2 not ready: FollowWaypoints or a lifecycle node failed the ready probe")
        return False

    def _lifecycle_node_is_active(self, node_name: str) -> bool:
        client = self.create_client(GetState, f"{node_name}/get_state")
        try:
            if not client.wait_for_service(timeout_sec=0.25):
                return False
            future = client.call_async(GetState.Request())
            completed = threading.Event()
            future.add_done_callback(lambda _: completed.set())
            if not completed.wait(timeout=0.75) or future.result() is None:
                return False
            # lifecycle_msgs/State.PRIMARY_STATE_ACTIVE == 3.
            return int(future.result().current_state.id) == 3
        except Exception:
            return False
        finally:
            self.destroy_client(client)

    def send_waypoints(self, waypoints: list[dict], feedback_cb: Callable, result_cb: Callable) -> bool:
        if not self.wait_until_ready(timeout_seconds=30):
            return False
        poses = []
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(waypoint["x"])
            pose.pose.position.y = float(waypoint["y"])
            yaw = float(waypoint.get("yaw", 0.0))
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            poses.append(pose)
        self._feedback_cb = feedback_cb
        self._result_cb = result_cb
        self._sent_pose_count = len(poses)
        use_through_poses = len(poses) > 1
        if use_through_poses:
            if not self._through_poses_client.wait_for_server(timeout_sec=5.0):
                LOGGER.error("NavigateThroughPoses action server is unavailable")
                return False
            goal = NavigateThroughPoses.Goal()
            goal.poses = poses
            client = self._through_poses_client
            feedback_cb_ros = self._on_through_poses_feedback
            self._nav_cancel_action = self._through_poses_action
        else:
            goal = FollowWaypoints.Goal()
            goal.poses = poses
            client = self._action_client
            feedback_cb_ros = self._on_feedback
            self._nav_cancel_action = self.ros_config.follow_waypoints_action
        future = client.send_goal_async(goal, feedback_callback=feedback_cb_ros)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        completed.wait(timeout=5)
        if not future.done() or future.result() is None or not future.result().accepted:
            return False
        self._goal_handle = future.result()
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)
        return True

    def _on_feedback(self, feedback_message) -> None:
        if self._feedback_cb:
            self._feedback_cb(int(feedback_message.feedback.current_waypoint), None)

    def _on_through_poses_feedback(self, feedback_message) -> None:
        if not self._feedback_cb:
            return
        remaining = int(getattr(feedback_message.feedback, "number_of_poses_remaining", 0) or 0)
        sent = max(self._sent_pose_count, 1)
        current = min(max(sent - remaining, 0), sent - 1)
        distance = getattr(feedback_message.feedback, "distance_remaining", None)
        self._feedback_cb(current, None if distance is None else float(distance))

    def _on_result(self, future) -> None:
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            result = getattr(wrapped, "result", None)
            missed_waypoints = self._extract_missed_waypoints(result)
            # action_msgs/GoalStatus: SUCCEEDED=4, CANCELED=5, ABORTED=6
            mapped = "succeeded" if status == 4 else "cancelled" if status == 5 else "failed"
            if self._result_cb:
                self._result_cb(
                    mapped,
                    "" if mapped != "failed" else f"goal_status={status}",
                    {"goal_status": status, "missed_waypoints": missed_waypoints},
                )
        except Exception as exc:
            LOGGER.exception("navigation result callback failed")
            if self._result_cb:
                self._result_cb("failed", str(exc))
        finally:
            self._goal_handle = None

    def _extract_missed_waypoints(self, result) -> list[int]:
        raw = list(getattr(result, "missed_waypoints", []) or [])
        missed = []
        for item in raw:
            if isinstance(item, int):
                missed.append(item)
                continue
            for attr in ("waypoint_index", "index"):
                value = getattr(item, attr, None)
                if value is not None:
                    missed.append(int(value))
                    break
        return missed

    def latest_pose(self):
        return self.telemetry.latest_pose()

    def teleop_velocity(self, vx: float = 0.0, vy: float = 0.0, yaw_rate: float = 0.0) -> dict:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(yaw_rate)
        # Manual remote control is intentionally isolated from Nav2 /cmd_vel.
        # The remote-only bridge encodes this as the vendor remote joystick protocol.
        self._teleop_cmd_vel_pub.publish(msg)
        return {
            "topic": "/teleop_cmd_vel",
            "vx": msg.linear.x,
            "vy": msg.linear.y,
            "yaw_rate": msg.angular.z,
        }

    def teleop_action(self, action: str) -> dict:
        msg = String()
        msg.data = str(action)
        # The teleop bridge may have just been started for manual control.
        # Publish discrete actions more than once so a transient ROS discovery
        # race does not drop a one-shot command like stand_up.
        for _ in range(3):
            self._teleop_action_pub.publish(msg)
            time.sleep(0.08)
        return {"topic": "/teleop_action", "action": msg.data, "publish_count": 3}

    def remote_teleop_action(self, action: str) -> dict:
        msg = String()
        msg.data = str(action)
        # One-shot vendor motions must not be repeated: the remote bridge
        # interprets each SetCmd as a new action (especially GREET).
        publish_count = 1 if action in {"shake_hand", "two_leg_stand"} else 3
        for _ in range(publish_count):
            self._remote_teleop_action_pub.publish(msg)
            if publish_count > 1:
                time.sleep(0.08)
        return {"topic": "/remote_teleop_action", "action": msg.data, "publish_count": publish_count}

    def confirmed_remote_teleop_action(
        self,
        action: str,
        success_states: set[str],
        failure_states: set[str] | None = None,
        timeout_seconds: float = 4.0,
    ) -> dict:
        failure_states = failure_states or set()
        with self._robot_motion_condition:
            start_sequence = self._robot_motion_state_sequence
        result = self.remote_teleop_action(action)
        deadline = time.monotonic() + max(0.0, timeout_seconds)
        with self._robot_motion_condition:
            while self._robot_motion_state_sequence <= start_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._robot_motion_condition.wait(timeout=remaining)
            state = self._robot_motion_state
        if state in failure_states:
            raise ProtocolError("TELEOP_ACTION_FAILED", f"robot reported {state} while executing {action}")
        if state not in success_states:
            raise ProtocolError(
                "TELEOP_ACTION_TIMEOUT",
                f"robot did not confirm {action} within {timeout_seconds:.1f}s (last motion state={state})",
            )
        return {**result, "confirmed": True, "motion_state": state}

    def confirmed_teleop_action(
        self,
        action: str,
        success_states: set[str],
        failure_states: set[str] | None = None,
        timeout_seconds: float = 4.0,
    ) -> dict:
        failure_states = failure_states or set()
        with self._robot_motion_condition:
            start_sequence = self._robot_motion_state_sequence
        result = self.teleop_action(action)
        deadline = time.monotonic() + max(0.0, timeout_seconds)
        with self._robot_motion_condition:
            while self._robot_motion_state_sequence <= start_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._robot_motion_condition.wait(timeout=remaining)
            state = self._robot_motion_state
        if state in failure_states:
            raise ProtocolError(
                "TELEOP_ACTION_FAILED",
                f"robot reported {state} while executing {action}",
            )
        if state not in success_states:
            raise ProtocolError(
                "TELEOP_ACTION_TIMEOUT",
                f"robot did not confirm {action} within {timeout_seconds:.1f}s "
                f"(last motion state={state})",
            )
        return {**result, "confirmed": True, "motion_state": state}

    def release_to_remote_control(self, timeout_seconds: float = 3.0) -> dict:
        self._remote_control_event.clear()
        result = self.teleop_action("release_remote")
        confirmed = self._remote_control_event.wait(timeout=max(0.0, timeout_seconds))
        if not confirmed:
            raise ProtocolError(
                "REMOTE_CONTROL_RELEASE_TIMEOUT",
                f"robot did not confirm remote control within {timeout_seconds:.1f}s "
                f"(last motion state={self._robot_motion_state})",
            )
        return {**result, "confirmed": True, "motion_state": self._robot_motion_state}

    def set_initial_pose(self, pose: dict) -> dict:
        yaw = float(pose.get("yaw", 0.0))
        x = float(pose["x"])
        y = float(pose["y"])
        z = float(pose.get("z", 0.0))
        frame_id = str(pose.get("frame_id") or "map")
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        # This localization node accepts zero-stamped initial poses more
        # reliably than wall-clock stamped poses when odom TF is unavailable.
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = float(pose.get("covariance_x", 0.25))
        msg.pose.covariance[7] = float(pose.get("covariance_y", 0.25))
        msg.pose.covariance[35] = float(pose.get("covariance_yaw", 0.0685))
        subscriber_deadline = time.monotonic() + float(pose.get("subscriber_wait_seconds", 15.0))
        while self._initial_pose_pub.get_subscription_count() == 0 and time.monotonic() < subscriber_deadline:
            time.sleep(0.2)
        if self._initial_pose_pub.get_subscription_count() == 0:
            raise ProtocolError("LOCALIZATION_UNAVAILABLE", "/initialpose has no localization subscriber")
        for _ in range(5):
            self._initial_pose_pub.publish(msg)
            time.sleep(0.2)
        with self._localization_sample_condition:
            sample_sequence = self._localization_sample_sequence
        wait_seconds = float(pose.get("wait_seconds", 8.0))
        required_normal_samples = int(pose.get("required_normal_samples", 0))
        require_absolute = bool(pose.get("require_absolute", False))
        if require_absolute:
            deadline = time.monotonic() + wait_seconds
            latest = self.telemetry.latest_pose()
            while time.monotonic() < deadline:
                latest = self.telemetry.latest_pose()
                if (
                    latest
                    and latest.localization_status == "normal"
                    and self._absolute_localization_stable()
                ):
                    break
                time.sleep(0.2)
            accepted = bool(
                latest
                and latest.localization_status == "normal"
                and self._absolute_localization_stable()
            )
        elif required_normal_samples > 0:
            latest = self._wait_for_fresh_normal_samples(
                after_sequence=sample_sequence,
                required_samples=required_normal_samples,
                timeout_seconds=wait_seconds,
            )
            accepted = latest is not None
        else:
            deadline = time.monotonic() + wait_seconds
            latest = self.telemetry.latest_pose()
            while time.monotonic() < deadline:
                latest = self.telemetry.latest_pose()
                stable_for = time.monotonic() - self.safety_state.localization_normal_since_monotonic
                if (
                    latest
                    and latest.localization_status == "normal"
                    and stable_for >= self.safety_config.localization_stable_seconds
                ):
                    break
                time.sleep(0.2)
            stable_for = time.monotonic() - self.safety_state.localization_normal_since_monotonic
            accepted = bool(
                latest
                and latest.localization_status == "normal"
                and stable_for >= self.safety_config.localization_stable_seconds
            )
        if not accepted:
            status = latest.localization_status if latest else "unknown"
            raise ProtocolError(
                "INITIAL_POSE_NOT_ACCEPTED",
                f"localization_status={status}, wait_seconds={wait_seconds:.1f}",
            )
        return {
            "frame_id": frame_id,
            "x": x,
            "y": y,
            "yaw": yaw,
            "topic": "/initialpose",
            "localization_status": latest.localization_status,
            "localized_pose": {
                "x": latest.x,
                "y": latest.y,
                "yaw": latest.yaw,
                "source_status": latest.source_status,
            },
        }

    def set_initial_pose_from_rtk(self, wait_seconds: float = 30.0) -> dict:
        """Ask localization to convert a fresh fixed RTK pose into map coordinates."""
        if not self._rtk_initial_pose_client.wait_for_service(timeout_sec=3.0):
            raise ProtocolError(
                "RTK_INITIAL_POSE_UNAVAILABLE",
                "/localization/seed_from_rtk service is unavailable",
            )
        with self._localization_sample_condition:
            sample_sequence = self._localization_sample_sequence
        future = self._rtk_initial_pose_client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=5.0) or not future.done():
            raise ProtocolError("RTK_INITIAL_POSE_TIMEOUT", "RTK initial pose service timed out")
        response = future.result()
        if response is None or not response.success:
            raise ProtocolError(
                "RTK_POSE_UNAVAILABLE",
                response.message if response else "RTK initial pose service returned no response",
            )
        latest = self._wait_for_fresh_normal_samples(
            after_sequence=sample_sequence,
            required_samples=3,
            timeout_seconds=wait_seconds,
        )
        if latest is None:
            raise ProtocolError(
                "RTK_INITIAL_POSE_NOT_CONVERGED",
                "fixed RTK pose was accepted but local NDT validation did not converge",
            )
        return {
            "source": "rtk_fixed",
            "service": "/localization/seed_from_rtk",
            "message": response.message,
            "localization_status": latest.localization_status,
            "localized_pose": {
                "x": latest.x,
                "y": latest.y,
                "z": latest.z,
                "yaw": latest.yaw,
            },
        }

    def global_relocalize(self, wait_seconds: float = 90.0) -> dict:
        """Run map-wide position and 360-degree yaw search without a guessed pose."""
        if not self._global_relocalize_client.wait_for_service(timeout_sec=3.0):
            raise ProtocolError(
                "GLOBAL_RELOCALIZATION_UNAVAILABLE",
                "/localization/global_relocalize service is unavailable",
            )
        with self._localization_sample_condition:
            sample_sequence = self._localization_sample_sequence
        future = self._global_relocalize_client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=5.0) or not future.done():
            raise ProtocolError(
                "GLOBAL_RELOCALIZATION_TIMEOUT", "global relocalization service timed out"
            )
        response = future.result()
        if response is None or not response.success:
            raise ProtocolError(
                "GLOBAL_RELOCALIZATION_UNAVAILABLE",
                response.message if response else "global relocalization returned no response",
            )
        latest = self._wait_for_fresh_normal_samples(
            after_sequence=sample_sequence,
            required_samples=3,
            timeout_seconds=wait_seconds,
        )
        if latest is None:
            raise ProtocolError(
                "GLOBAL_RELOCALIZATION_NOT_VERIFIED",
                f"{response.message}; no verified 3-frame localization within {wait_seconds:.1f}s",
            )
        return {
            "mode": "global_position_yaw_search",
            "source": "scan_context",
            "service": "/localization/global_relocalize",
            "message": response.message,
            "localization_status": latest.localization_status,
            "localized_pose": {
                "x": latest.x,
                "y": latest.y,
                "z": latest.z,
                "yaw": latest.yaw,
            },
            "motion_commanded": False,
        }

    def _wait_for_fresh_normal_samples(
        self,
        after_sequence: int,
        required_samples: int,
        timeout_seconds: float,
    ):
        deadline = time.monotonic() + timeout_seconds
        with self._localization_sample_condition:
            while True:
                if self._fresh_normal_streak(
                    self._localization_status_samples,
                    after_sequence,
                ) >= required_samples:
                    return self.telemetry.latest_pose()
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._localization_sample_condition.wait(timeout=min(0.2, remaining))

    @staticmethod
    def _fresh_normal_streak(samples, after_sequence: int) -> int:
        streak = 0
        for sequence, status in reversed(samples):
            if sequence <= after_sequence or status != 3:
                break
            streak += 1
        return streak

    def active_relocalize(self, seed: dict) -> dict:
        """Try bounded stationary pose candidates without commanding motion."""
        base_x = float(seed["x"])
        base_y = float(seed["y"])
        base_z = float(seed.get("z", 0.0))
        base_yaw = float(seed["yaw"])
        max_attempts = max(1, min(12, int(seed.get("max_attempts", 12))))
        candidates = self._relocalization_candidates(base_x, base_y, base_z, base_yaw)
        attempts = []
        for index, candidate in enumerate(candidates[:max_attempts], start=1):
            try:
                result = self.set_initial_pose({
                    **candidate,
                    "frame_id": "map",
                    "wait_seconds": 10.0,
                    "required_normal_samples": 3,
                    "require_absolute": True,
                    "covariance_x": 1.0,
                    "covariance_y": 1.0,
                    "covariance_yaw": 0.274,
                })
                latest = self.telemetry.latest_pose()
                if self._trusted_pose_cb and latest:
                    self._trusted_pose_cb(latest)
                    self._last_trusted_pose_report_monotonic = time.monotonic()
                return {
                    "mode": "stationary_bounded_search",
                    "source": seed.get("source", "operator_seed"),
                    "attempts": attempts + [{"index": index, **candidate, "accepted": True}],
                    "localized_pose": result["localized_pose"],
                    "localization_status": result["localization_status"],
                    "motion_commanded": False,
                }
            except ProtocolError as exc:
                attempts.append({
                    "index": index,
                    **candidate,
                    "accepted": False,
                    "error_code": exc.code,
                })
        latest = self.telemetry.latest_pose()
        raise ProtocolError(
            "ACTIVE_RELOCALIZATION_FAILED",
            f"stationary search exhausted {len(attempts)} candidates; "
            f"localization_status={latest.localization_status if latest else 'unknown'}",
        )

    @staticmethod
    def _relocalization_candidates(x: float, y: float, z: float, yaw: float) -> list[dict]:
        def normalize(angle: float) -> float:
            return math.atan2(math.sin(angle), math.cos(angle))

        candidates = [
            {"x": x, "y": y, "z": z, "yaw": normalize(yaw + offset)}
            for offset in (
                0.0,
                math.pi / 4,
                -math.pi / 4,
                math.pi / 2,
                -math.pi / 2,
                3 * math.pi / 4,
                -3 * math.pi / 4,
                math.pi,
            )
        ]
        candidates.extend(
            {"x": x + dx, "y": y + dy, "z": z, "yaw": normalize(yaw)}
            for dx, dy in ((0.3, 0.0), (-0.3, 0.0), (0.0, 0.3), (0.0, -0.3))
        )
        return candidates

    def cancel_navigation(self, timeout_seconds: float = 5.0) -> bool:
        if self._goal_handle is not None:
            future = self._goal_handle.cancel_goal_async()
        else:
            # Edge may have restarted after it sent a goal. In that case the
            # local handle is gone while Nav2 continues executing the goal.
            # A default CancelGoal request cancels every goal on this action.
            client = self.create_client(
                CancelGoal, f"{self._nav_cancel_action}/_action/cancel_goal")
            if not client.wait_for_service(timeout_sec=min(timeout_seconds, 2.0)):
                self.destroy_client(client)
                LOGGER.error("%s cancel service is unavailable", self._nav_cancel_action)
                return False
            future = client.call_async(CancelGoal.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        completed.wait(timeout=timeout_seconds)
        response = future.result() if future.done() else None
        cancelled = bool(response and response.goals_canceling)
        if not cancelled:
            LOGGER.error("%s cancellation was not acknowledged", self._nav_cancel_action)
        return cancelled

    def stop_motion(self) -> None:
        """Publish an explicit zero command after a navigation goal is cancelled."""
        zero = Twist()
        for _ in range(5):
            self._cmd_vel_pub.publish(zero)
            time.sleep(0.05)

    def set_docking_profile(self, *, final_approach: bool) -> None:
        """Apply the slow, fine-control profile used only for the dock contact leg."""
        vx_max = 0.10 if final_approach else 0.5
        vx_min = -0.10 if final_approach else -0.12
        wz_max = 0.20 if final_approach else 0.5
        fine_control = Bool()
        fine_control.data = bool(final_approach)
        self._fine_control_pub.publish(fine_control)
        self._set_remote_parameters(
            "/controller_server",
            {
                "FollowPath.vx_max": vx_max,
                "FollowPath.vx_min": vx_min,
                "FollowPath.wz_max": wz_max,
            },
            code="DOCKING_PROFILE_FAILED",
        )
        LOGGER.info(
            "docking fine-control=%s vx=[%s,%s] wz_max=%s",
            final_approach, vx_min, vx_max, wz_max,
        )

    def set_waypoint_profile(
        self,
        *,
        avoid_obstacles: bool,
        require_yaw: bool,
        final_approach: bool = False,
        live: bool = False,
    ) -> None:
        yaw_message = Bool()
        yaw_message.data = bool(require_yaw)
        self._goal_yaw_required_pub.publish(yaw_message)
        outdoor = self._rtk_is_navigation_pose_source()
        # Temporary smoothness test: keep lidar local avoidance off even
        # when a waypoint asks for obstacles. Edge startup was turning it
        # back on before RTK was classified as the pose source.
        local_obstacles = False
        global_obstacles = bool(avoid_obstacles) and not outdoor
        # Last-point approach must be slow enough that the DiffDrive turning
        # radius (v / wz_max) fits inside the 0.35 m goal window. At 0.5 m/s
        # that radius is 1.0 m, so the dog orbits the final point instead of
        # stopping. PreferForward is off on the last point so overshoot can
        # reverse instead of looping.
        # Apply FollowPath first. A live last-point switch must not wait on
        # costmaps or retry controller_server for ~20s: that holds the task
        # lock while the dog keeps cruising past the click.
        params = follow_path_patrol_params(
            final_approach=final_approach,
            local_obstacles=local_obstacles,
        )
        follow_applied = False
        try:
            self._set_remote_parameters(
                "/controller_server",
                params,
                code="WAYPOINT_PROFILE_FAILED",
                attempts=2 if live else 8,
            )
            follow_applied = True
        except ProtocolError:
            LOGGER.warning("unable to apply FollowPath waypoint speed profile")
        if not live:
            for node_name, parameter_name, value in (
                ("/local_costmap/local_costmap", "obstacle_layer.enabled", local_obstacles),
                ("/global_costmap/global_costmap", "obstacle_layer.enabled", global_obstacles),
                ("/collision_monitor", "PolygonStop.enabled", local_obstacles),
                ("/collision_monitor", "PolygonSlow.enabled", local_obstacles),
            ):
                try:
                    self._set_remote_parameters(
                        node_name,
                        {parameter_name: value},
                        code="WAYPOINT_PROFILE_FAILED",
                        attempts=1,
                    )
                except ProtocolError:
                    if node_name == "/global_costmap/global_costmap":
                        LOGGER.info(
                            "global costmap has no obstacle_layer; lidar avoidance stays on the local costmap"
                        )
                    else:
                        LOGGER.warning(
                            "unable to set %s on %s; keeping the FollowPath profile",
                            parameter_name,
                            node_name,
                        )
                    continue
        LOGGER.info(
            "waypoint profile final_approach=%s live=%s follow_applied=%s vx=[%s,%s] wz_max=%s path_align=%s cost=%s",
            final_approach,
            live,
            follow_applied,
            params["FollowPath.vx_min"],
            params["FollowPath.vx_max"],
            params["FollowPath.wz_max"],
            params["FollowPath.PathAlignCritic.enabled"],
            params["FollowPath.CostCritic.enabled"],
        )

    def apply_outdoor_gps_profile(self) -> None:
        """Prefer a GPS line path; keep lidar for local slowdown/stop only."""
        if not self._rtk_is_navigation_pose_source():
            return
        try:
            self._set_remote_parameters(
                "/planner_server",
                {
                    "GridBased.allow_straight_line_fallback": True,
                    "GridBased.prefer_straight_line": True,
                    "GridBased.tolerance": 2.0,
                },
                code="WAYPOINT_PROFILE_FAILED",
            )
        except ProtocolError:
            LOGGER.warning("outdoor GPS planner fallback was not applied")

    def set_goal_precision(self, *, enabled: bool) -> None:
        """Select the tight pose tolerances used only for the dock contact point."""
        xy_tolerance = (
            self.safety_config.docking_goal_tolerance_m if enabled else 0.35
        )
        yaw_tolerance = (
            self.safety_config.docking_goal_yaw_tolerance_rad if enabled else 0.25
        )
        try:
            self._set_remote_parameters(
                "/controller_server",
                {
                    "general_goal_checker.xy_goal_tolerance": float(xy_tolerance),
                    "general_goal_checker.required_yaw_goal_tolerance": float(yaw_tolerance),
                },
                code="GOAL_PRECISION_PROFILE_FAILED",
                attempts=2 if not enabled else 8,
            )
        except ProtocolError:
            LOGGER.warning("unable to set goal precision enabled=%s", enabled)
            if enabled:
                raise

    def _set_remote_parameters(
        self,
        node_name: str,
        values: dict[str, bool | int | float],
        *,
        code: str,
        attempts: int = 8,
    ) -> None:
        """Set Nav2 parameters through its ROS service, without spawning ros2 CLI processes."""
        last_error = ""
        for _ in range(max(1, attempts)):
            client = self.create_client(SetParameters, f"{node_name}/set_parameters")
            try:
                if not client.wait_for_service(timeout_sec=0.75):
                    last_error = f"{node_name} parameter service is unavailable"
                else:
                    request = SetParameters.Request()
                    request.parameters = [self._parameter_message(name, value) for name, value in values.items()]
                    future = client.call_async(request)
                    completed = threading.Event()
                    future.add_done_callback(lambda _: completed.set())
                    if not completed.wait(timeout=1.5):
                        last_error = f"{node_name} parameter request timed out"
                    else:
                        response = future.result()
                        failures = [result.reason or "rejected" for result in response.results if not result.successful]
                        if not failures:
                            return
                        last_error = "; ".join(failures)
            except Exception as exc:
                last_error = str(exc)
            finally:
                self.destroy_client(client)
            time.sleep(0.25)
        raise ProtocolError(code, last_error or f"unable to set parameters on {node_name}")

    @staticmethod
    def _parameter_message(name: str, value: bool | int | float) -> ParameterMessage:
        parameter = ParameterMessage()
        parameter.name = name
        parameter.value = ParameterValue()
        if isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = value
        elif isinstance(value, int):
            parameter.value.type = ParameterType.PARAMETER_INTEGER
            parameter.value.integer_value = value
        else:
            parameter.value.type = ParameterType.PARAMETER_DOUBLE
            parameter.value.double_value = float(value)
        return parameter

    def is_robot_stopped(self) -> bool:
        deadline = time.monotonic() + self.safety_config.stop_confirmation_seconds + 3
        stable_since = None
        while time.monotonic() < deadline:
            # Localization.speed is derived from scan matching and is noisy
            # enough to report >1 m/s while the robot is stationary.  The
            # collision-monitor output is the actual motion command and is the
            # reliable source for pause confirmation.
            planar_speed = math.hypot(self._actual_forward_command, self._actual_lateral_command)
            stopped = (
                planar_speed <= self.safety_config.stop_speed_threshold_mps
                and abs(self._actual_turn_command) <= 0.05
            )
            if stopped:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= self.safety_config.stop_confirmation_seconds:
                    return True
            else:
                stable_since = None
            time.sleep(0.05)
        return False


class RosRuntime:
    def __init__(self, node: RosAdapter) -> None:
        self.node = node
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True, name="ros-executor")

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        self.thread.join(timeout=3)
