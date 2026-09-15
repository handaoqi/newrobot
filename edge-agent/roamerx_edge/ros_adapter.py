from __future__ import annotations

import copy
import logging
import json
import math
import os
import threading
import time
from collections import deque
from typing import Callable

from .callback_performance import CallbackPerformanceMonitor
from .config import (
    MappingConfig,
    RosCallbackOptimizationConfig,
    RosConfig,
    SafetyConfig,
)
from .imu_cross_check import ImuCrossCheck, ImuCrossCheckConfig
from .localization_recovery import planar_distance_m
from .navigation_controllers import (
    global_controller_plugin_id,
    local_controller_plugin_id,
    normalize_global_controller,
    normalize_local_controller,
)
from .navigation_speed import NavigationSpeedProfile, navigation_speed_profile
from .protocol import ProtocolError, now_iso
from .rtk_origin import RtkOriginPayloadCache
from .safety_policy import RuntimeSafetyState
from .telemetry_collector import TelemetryCollector

LOGGER = logging.getLogger(__name__)

# A live lidar pose at the same place whose heading is in the opposite
# hemisphere means the RTK dual-antenna yaw (or 180deg offset) is unusable.
# Weak or missing lidar stays advisory so outdoor RTK can still initialize.
RTK_LIDAR_HEADING_CONFLICT_DEG = 90.0
RTK_LIDAR_HEADING_CONFLICT_XY_M = 3.0


def follow_path_patrol_params(
    *,
    final_approach: bool,
    local_obstacles: bool,
    require_yaw: bool = False,
    outdoor: bool = False,
    speed_profile: NavigationSpeedProfile | None = None,
    reapproach: bool = False,
) -> dict[str, bool | float]:
    """MPPI settings for a patrol goal.

    Outdoor cruise uses PathAlign so FollowPath tracks the ThetaStar line
    instead of walking an arc under PreferForward. Outdoor final approach
    keeps PathAlign off so CostCritic can still go around the mark. Path
    orientations stay off: click noise would otherwise steer left/right.
    CostCritic is only useful when the local obstacle layer is painting.
    Final approach slows down so the DiffDrive turning radius fits the
    0.35 m window.
    """
    vx_max = 0.15 if final_approach else float(
        (speed_profile or navigation_speed_profile("micro")).vx_mps
    )
    # Last-metre, reapproach, and outdoor cruise are forward-only. Allowing
    # reverse on outdoor FollowPath lets CostCritic hunt grass returns instead
    # of tracking ThetaStar. Indoor cruise still keeps vx_min=-0.12 so a
    # blocked leg can reverse around a mark; Edge BackUp owns outdoor reverse.
    vx_min = 0.0 if reapproach or final_approach or outdoor else -0.12
    wz_max = 0.35 if final_approach else float(
        (speed_profile or navigation_speed_profile("micro")).wz_rps
    )
    return {
        "FollowPath.vx_max": vx_max,
        "FollowPath.vx_min": vx_min,
        # DiffDrive keeps vy at zero in its motion model. Preserve the shared
        # profile here for diagnostics and future Omni support without
        # pretending autonomous navigation can laterally translate today.
        "FollowPath.vy_max": 0.0 if final_approach else float(
            (speed_profile or navigation_speed_profile("micro")).vy_mps
        ),
        "FollowPath.wz_max": wz_max,
        "FollowPath.wz_std": 0.04,
        "FollowPath.gamma": 0.03,
        "FollowPath.time_steps": 40,
        "FollowPath.model_dt": 0.05,
        "FollowPath.batch_size": 1200,
        "FollowPath.GoalCritic.enabled": bool(final_approach),
        # A heading-constrained stop needs an explicit terminal-angle cost.
        # PathAlign targets the path tangent instead and made the robot orbit
        # short goals while the goal checker waited for the requested yaw.
        "FollowPath.GoalAngleCritic.enabled": bool(final_approach and require_yaw),
        "FollowPath.PreferForwardCritic.enabled": True,
        "FollowPath.CostCritic.enabled": bool(local_obstacles),
        # Outdoor scans contain more grass and long-range noise. Keep collision
        # rejection active, but use a lower gradient weight so MPPI makes one
        # deliberate detour instead of weaving along the RTK reference line.
        "FollowPath.CostCritic.cost_weight": 8.0 if outdoor else 18.0,
        # Indoor: tangent pull on the last metre and after a local detour.
        # Outdoor cruise: same pull so the dog follows the planned line.
        # Outdoor final click: leave it off; PathAlign used to override
        # CostCritic and drive straight into the mark.
        "FollowPath.PathAlignCritic.enabled": bool(
            not require_yaw
            and (
                (not outdoor and (final_approach or local_obstacles))
                or (outdoor and not final_approach)
            )
        ),
        "FollowPath.PathAlignCritic.cost_weight": 4.0 if local_obstacles else 12.0,
        "FollowPath.PathAlignCritic.offset_from_furthest": 4,
        "FollowPath.PathAlignCritic.use_path_orientations": False,
        "FollowPath.PathFollowCritic.enabled": True,
        "FollowPath.PathFollowCritic.cost_weight": 14.0,
        "FollowPath.PathAngleCritic.enabled": True,
        "FollowPath.PathAngleCritic.cost_weight": 3.0,
        "FollowPath.PathAngleCritic.max_angle_to_furthest": 0.30,
    }

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import BackUp, DriveOnHeading, FollowWaypoints, NavigateThroughPoses
    from nav_msgs.msg import Odometry, Path
    from nav2_msgs.msg import SpeedLimit
    from action_msgs.srv import CancelGoal
    from lifecycle_msgs.srv import GetState
    from rclpy.action import ActionClient
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan, NavSatFix
    from robots_dog_msgs.msg import Localization, UniRtkPvh
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from rcl_interfaces.msg import Parameter as ParameterMessage, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    # Keep module consumers importable in test/diagnostic environments without
    # ROS. EdgeAgentApplication only calls rclpy when ROS_AVAILABLE is true.
    rclpy = None  # type: ignore[assignment]
    Node = object
    GetParameters = None  # type: ignore

    # Keep policy/profile helpers unit-testable when ROS Python messages are
    # not installed. These are intentionally minimal stand-ins; the real
    # message classes above are always used on the robot.
    class String:  # type: ignore[no-redef]
        def __init__(self):
            self.data = ""

    class Bool:  # type: ignore[no-redef]
        def __init__(self):
            self.data = False

    class SpeedLimit:  # type: ignore[no-redef]
        def __init__(self):
            self.speed_limit = -1.0
            self.percentage = False

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

try:
    if ROS_AVAILABLE:
        from robots_dog_msgs.srv import ControlLocalizationCorrection, NavigationRecoveryLease
    else:
        ControlLocalizationCorrection = None
        NavigationRecoveryLease = None
except ImportError:
    ControlLocalizationCorrection = None
    NavigationRecoveryLease = None

try:
    if ROS_AVAILABLE:
        from robots_dog_msgs.srv import NavigationSelfHealing, SetLocalizationFusionProfile
    else:
        NavigationSelfHealing = None
        SetLocalizationFusionProfile = None
except ImportError:
    NavigationSelfHealing = None
    SetLocalizationFusionProfile = None


class RosAdapter(Node):
    def __init__(
        self,
        ros_config: RosConfig,
        safety_config: SafetyConfig,
        telemetry: TelemetryCollector,
        safety_state: RuntimeSafetyState,
        mapping_config: MappingConfig | None = None,
        imu_cross_check_config: ImuCrossCheckConfig | None = None,
        structured_logs=None,
        callback_optimization_config: RosCallbackOptimizationConfig | None = None,
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
        self._trusted_pose_frozen = False
        self._attempt_progress_cb: Callable | None = None
        self._recovery_lease_acquire_cb: Callable | None = None
        self._recovery_lease_release_cb: Callable | None = None
        self._recovery_snapshot_cb: Callable | None = None
        self._self_healing_cb: Callable | None = None
        self._bt_recovery_leases: dict[int, object] = {}
        self.structured_logs = structured_logs
        self._callback_optimization = (
            callback_optimization_config or RosCallbackOptimizationConfig()
        )
        self._callback_performance = CallbackPerformanceMonitor()
        self._control_callback_group = MutuallyExclusiveCallbackGroup()
        self._telemetry_callback_group = MutuallyExclusiveCallbackGroup()
        # These three subscriptions are the stop-confirmation authority and
        # the source of every Edge pose/status frame.  They must not queue
        # behind a long scan-match, recovery-service, or scan-processing
        # callback in the mutually-exclusive control group.  A reentrant,
        # dedicated group keeps their tiny state assignments responsive while
        # the other callback groups perform bounded work.
        self._safety_callback_group = ReentrantCallbackGroup()
        # Action result callbacks enter TaskExecutor and may wait for its state
        # lock while an obstacle-recovery worker is making synchronous ROS
        # requests.  Keeping Nav2 action and service responses in reentrant,
        # dedicated groups prevents one blocked result callback from starving
        # cancel, lifecycle, or parameter futures on this node.
        self._nav_action_callback_group = ReentrantCallbackGroup()
        self._nav_service_callback_group = ReentrantCallbackGroup()
        # Service clients belong to the node for its full lifetime. Reusing
        # them avoids destroying handles that may still be present in the
        # executor wait set after a timeout.
        self._service_clients: dict[tuple[object, str], object] = {}
        self._service_clients_lock = threading.Lock()
        self._ros_executor_alive_provider: Callable[[], bool] | None = None
        self._latest_odometry = None
        self._odom_linear_x = 0.0
        self._odom_linear_y = 0.0
        self._odom_angular_z = 0.0
        self._odom_received_monotonic = 0.0
        self._standstill_started_monotonic = None
        self._lio_started_monotonic = time.monotonic()
        self._odometry_sequence = 0
        self._odometry_consumed_sequence = 0
        self._latest_scan = None
        self._latest_scan_received_monotonic = 0.0
        self._scan_sequence = 0
        self._scan_consumed_sequence = 0
        self._scan_geometry_key = None
        self._scan_geometry: list[tuple[int, float, float]] = []
        self._latest_lidar_imu = None
        self._lidar_imu_sequence = 0
        self._lidar_imu_consumed_sequence = 0
        self._latest_body_imu = None
        self._body_imu_sequence = 0
        self._body_imu_consumed_sequence = 0
        self._last_scan_matching_detail_monotonic = 0.0
        self._last_scan_matching_signature = None
        self._log_context_provider: Callable | None = None
        self._localization_sample_condition = threading.Condition()
        self._localization_sample_sequence = 0
        self._localization_decision_sequence = 0
        self._localization_status_samples = deque(maxlen=100)
        # A localization operation owns a monotonically increasing generation.
        # Starting an operator request invalidates an older automatic search so
        # the old worker can no longer overwrite the newly selected pose while
        # it is being verified.
        self._localization_operation_lock = threading.Lock()
        self._localization_operation_generation = 0
        self._operator_localization_depth = 0
        self._scan_match_condition = threading.Condition()
        self._scan_match_sequence = 0
        self._scan_match_records: dict[tuple[int, int], dict] = {}
        self._scan_match_record_order = deque(maxlen=200)
        self._localization_lost_count = 0
        self._localization_failure_notified = False
        self._lio_motion_anomaly_notified = False
        self._lio_absolute_disagreement_notified = False
        self._ndt_failure_count = 0
        self._ndt_failure_notified = False
        self._localization_recovery_pending = False
        self._localization_recovery_armed = False
        self._latest_speed = 0.0
        self._latest_localization_status = None
        self._raw_forward_command = 0.0
        self._actual_forward_command = 0.0
        self._raw_lateral_command = 0.0
        self._actual_lateral_command = 0.0
        self._raw_turn_command = 0.0
        self._actual_turn_command = 0.0
        self._raw_velocity_updated_monotonic = 0.0
        self._actual_velocity_updated_monotonic = 0.0
        self._front_obstacle_distance_m = None
        self._rear_clearance_m = None
        self._left_clearance_m = None
        self._right_clearance_m = None
        self._collision_monitor_state: dict = {}
        self._collision_monitor_state_received_monotonic = 0.0
        self._obstacle_recovery_goal_lock = threading.Lock()
        self._obstacle_recovery_goal_handle = None
        self._global_plan_points: list[dict] = []
        self._global_plan_updated_monotonic = 0.0
        self._global_plan_stale_seconds = 30.0
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
        self._active_local_controller: str | None = None
        self._active_global_controller: str | None = None
        # Skip identical Nav2 parameter writes across consecutive waypoints.
        self._remote_param_cache: dict[str, dict[str, bool | int | float]] = {}
        # Nodes whose set_parameters service recently timed out stay skipped so
        # each waypoint does not burn ~2s × N failed costmap/monitor calls.
        self._remote_param_unavailable_until: dict[str, float] = {}
        self._waypoint_profile_signature: tuple | None = None
        self._outdoor_planner_profile: bool | None = None
        self._boundary_zone_speed_limit: float | None = None
        self._boundary_base_velocity = {"vx_max": 0.30, "vx_min": -0.12, "vy_max": 0.5}
        self._rtk_origin_cache = RtkOriginPayloadCache(
            stale_after_seconds=(mapping_config.heading_max_age_seconds if mapping_config else 1.5)
        )
        # Durable diagnostics for relocalization searches.  This is deliberately
        # outside the ROS graph so a node restart does not erase the candidate
        # state needed to explain a failed cold start.
        self._relocalization_state_file = None
        if mapping_config and mapping_config.map_dir:
            self._relocalization_state_file = os.path.join(
                os.path.expanduser(mapping_config.map_dir), "relocalization_search_state.json"
            )
        self.create_subscription(
            Localization,
            ros_config.localization_topic,
            self._on_localization,
            10,
            callback_group=self._safety_callback_group,
        )
        self.create_subscription(
            Odometry,
            ros_config.odometry_topic,
            self._on_odometry,
            20,
            callback_group=self._safety_callback_group,
        )
        self.create_subscription(
            Twist, ros_config.cmd_vel_raw_topic, self._on_cmd_vel_raw, 10,
            callback_group=self._safety_callback_group,
        )
        self.create_subscription(
            Twist, ros_config.cmd_vel_topic, self._on_cmd_vel, 10,
            callback_group=self._safety_callback_group,
        )
        self.create_subscription(
            LaserScan, ros_config.scan_topic, self._on_scan, qos_profile_sensor_data,
            callback_group=self._control_callback_group,
        )
        self.create_subscription(String, "/sensor_health", self._on_sensor_health, 2)
        self.create_subscription(String, "/localization/decision", self._on_localization_decision, 10)
        self.create_subscription(String, "/planner/performance", self._on_planner_performance, 10)
        self.create_subscription(String, "/mppi/performance", self._on_mppi_performance, 10)
        self.create_subscription(String, "/collision_monitor/state", self._on_collision_state, 10)
        self.create_subscription(
            PoseStamped,
            "/localization/scan_match_pose",
            self._on_scan_match_pose,
            qos_profile_sensor_data,
        )
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
                callback_group=self._control_callback_group,
            )
        elif ros_config.scan_matching_status_topic:
            LOGGER.warning(
                "localization ScanMatchingStatus message is unavailable; quality telemetry subscription is disabled"
            )
        self._action_client = ActionClient(
            self,
            FollowWaypoints,
            ros_config.follow_waypoints_action,
            callback_group=self._nav_action_callback_group,
        )
        through_poses_action = getattr(
            ros_config, "navigate_through_poses_action", "/navigate_through_poses"
        )
        self._through_poses_client = ActionClient(
            self,
            NavigateThroughPoses,
            through_poses_action,
            callback_group=self._nav_action_callback_group,
        )
        self._through_poses_action = through_poses_action
        self._backup_client = ActionClient(
            self,
            BackUp,
            "/backup",
            callback_group=self._nav_action_callback_group,
        )
        self._drive_on_heading_client = ActionClient(
            self,
            DriveOnHeading,
            "/drive_on_heading",
            callback_group=self._nav_action_callback_group,
        )
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 8)
        self._rtk_initial_pose_client = self.create_client(Trigger, "/localization/seed_from_rtk")
        self._global_relocalize_client = self.create_client(
            Trigger, "/localization/global_relocalize"
        )
        self._localization_correction_client = (
            self.create_client(
                ControlLocalizationCorrection, "/localization/control_correction"
            )
            if ControlLocalizationCorrection is not None
            else None
        )
        self._localization_fusion_profile_client = (
            self.create_client(
                SetLocalizationFusionProfile,
                "/localization/set_fusion_profile",
                callback_group=self._nav_service_callback_group,
            )
            if SetLocalizationFusionProfile is not None
            else None
        )
        self._recovery_lease_service = (
            self.create_service(
                NavigationRecoveryLease,
                "/navigation/recovery/lease",
                self._on_recovery_lease,
                callback_group=self._control_callback_group,
            )
            if NavigationRecoveryLease is not None
            else None
        )
        self._self_healing_service = (
            self.create_service(
                NavigationSelfHealing,
                "/navigation/self_healing",
                self._on_navigation_self_healing,
                callback_group=self._control_callback_group,
            )
            if NavigationSelfHealing is not None
            else None
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._arrival_adjust_cmd_vel_pub = self.create_publisher(
            Twist, ros_config.cmd_vel_raw_topic, 10
        )
        # Manual assist is deliberately routed through the Nav2 velocity
        # optimizer and collision monitor, never directly to the vendor bridge.
        self._manual_assist_cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_assist", 10)
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
        self._controller_selector_pub = self.create_publisher(String, "/controller_selector", goal_yaw_qos)
        self._planner_selector_pub = self.create_publisher(String, "/planner_selector", goal_yaw_qos)
        self._smoother_selector_pub = self.create_publisher(String, "/smoother_selector", goal_yaw_qos)
        self._navigation_speed_limit_pub = self.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.create_subscription(String, "/robot_motion_state", self._on_robot_motion_state, 10)
        self.create_subscription(Path, "/plan", self._on_global_plan, 10)
        self._install_callback_timers()

    @staticmethod
    def _timer_period(rate_hz: float) -> float:
        return 1.0 / max(0.1, float(rate_hz))

    def _install_callback_timers(self) -> None:
        config = self._callback_optimization
        if config.enabled:
            self.create_timer(
                self._timer_period(config.odometry_telemetry_rate_hz),
                self._consume_latest_odometry,
                callback_group=self._telemetry_callback_group,
            )
            self.create_timer(
                self._timer_period(config.scan_processing_rate_hz),
                self._consume_latest_scan,
                callback_group=self._control_callback_group,
            )
            if self._imu_cross_check.config.enabled:
                self.create_timer(
                    self._timer_period(config.imu_sample_rate_hz),
                    self._consume_latest_imu,
                    callback_group=self._telemetry_callback_group,
                )
        if config.metrics_enabled:
            self.create_timer(
                max(1.0, float(config.metrics_interval_seconds)),
                self._log_callback_performance,
                callback_group=self._telemetry_callback_group,
            )

    def _latest_queue_depth(self) -> int:
        return sum((
            getattr(self, "_odometry_sequence", 0) > getattr(self, "_odometry_consumed_sequence", 0),
            getattr(self, "_scan_sequence", 0) > getattr(self, "_scan_consumed_sequence", 0),
            getattr(self, "_lidar_imu_sequence", 0) > getattr(self, "_lidar_imu_consumed_sequence", 0),
            getattr(self, "_body_imu_sequence", 0) > getattr(self, "_body_imu_consumed_sequence", 0),
        ))

    def _callback_optimization_enabled(self) -> bool:
        return bool(getattr(getattr(self, "_callback_optimization", None), "enabled", False))

    def _callback_metrics_enabled(self) -> bool:
        return bool(
            getattr(getattr(self, "_callback_optimization", None), "metrics_enabled", False)
        )

    def record_callback_performance(self, name: str, duration_seconds: float) -> None:
        if self._callback_metrics_enabled():
            self._callback_performance.record(
                name,
                duration_seconds,
                queue_depth=self._latest_queue_depth(),
            )

    def _log_callback_performance(self) -> None:
        summary = self._callback_performance.snapshot_and_reset()
        LOGGER.info("edge_callback_perf %s", json.dumps(summary, sort_keys=True))

    def _on_global_plan(self, msg) -> None:
        points = []
        for pose_stamped in getattr(msg, "poses", []) or []:
            position = pose_stamped.pose.position
            points.append({"x": float(position.x), "y": float(position.y)})
        self._global_plan_points = points
        self._global_plan_updated_monotonic = time.monotonic()

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
        decision_condition = getattr(self, "_localization_sample_condition", None)
        if decision_condition is not None:
            with decision_condition:
                self._localization_decision_sequence = int(
                    getattr(self, "_localization_decision_sequence", 0)
                ) + 1
                decision_condition.notify_all()
        anomaly = bool(payload.get("lio_motion_anomaly"))
        disagreement = bool(payload.get("lio_large_absolute_disagreement"))
        if not anomaly and not disagreement:
            self._lio_motion_anomaly_notified = False
            self._lio_absolute_disagreement_notified = False
            return

        already_notified = (
            getattr(self, "_lio_motion_anomaly_notified", False) if anomaly
            else getattr(self, "_lio_absolute_disagreement_notified", False)
        )
        if (
            already_notified
            or not self._localization_failure_cb
        ):
            return
        if anomaly:
            self._lio_motion_anomaly_notified = True
        else:
            self._lio_absolute_disagreement_notified = True
        self._localization_failure_notified = True
        self._localization_recovery_armed = True
        threading.Thread(
            target=self._localization_failure_cb,
            args=("lio_motion_anomaly" if anomaly else "lio_absolute_disagreement",),
            daemon=True,
            name="lio-localization-fault-handler",
        ).start()

    def set_log_context_provider(self, provider: Callable | None) -> None:
        self._log_context_provider = provider

    def _emit_ros_diagnostic(self, level: str, event_code: str, message: str, payload: dict) -> None:
        if not self.structured_logs:
            return
        context = self._log_context_provider() if callable(self._log_context_provider) else {}
        context = context if isinstance(context, dict) else {}
        module = "planner" if event_code.startswith(("planner.", "mppi.")) else "avoidance"
        self.structured_logs.emit(level, module, event_code, message, data=payload, **context)

    def _diagnostic_payload(self, msg) -> dict | None:
        try:
            payload = json.loads(str(getattr(msg, "data", "") or "{}"))
        except (TypeError, ValueError, json.JSONDecodeError):
            LOGGER.warning("invalid ROS diagnostic payload")
            return None
        return payload if isinstance(payload, dict) else None

    def _on_planner_performance(self, msg) -> None:
        payload = self._diagnostic_payload(msg)
        if payload is not None:
            level = "INFO" if payload.get("success", True) else "WARNING"
            self._emit_ros_diagnostic(level, "planner.performance.sample", "规划器性能数据", payload)

    def _on_mppi_performance(self, msg) -> None:
        payload = self._diagnostic_payload(msg)
        if payload is not None:
            self._emit_ros_diagnostic("DEBUG", "mppi.performance.sample", "MPPI性能数据", payload)

    def _on_collision_state(self, msg) -> None:
        payload = self._diagnostic_payload(msg)
        if payload is None:
            return
        state = str(payload.get("state") or "CLEAR").upper()
        self._collision_monitor_state = dict(payload)
        self._collision_monitor_state["state"] = state
        self._collision_monitor_state_received_monotonic = time.monotonic()
        level = "INFO" if state == "CLEAR" else "WARNING"
        self._emit_ros_diagnostic(level, "avoidance.collision_monitor.state", f"避障状态变为 {state}", payload)

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

    @staticmethod
    def _stamp_key(header) -> tuple[int, int]:
        stamp = getattr(header, "stamp", None)
        return (
            int(getattr(stamp, "sec", 0)),
            int(getattr(stamp, "nanosec", 0)),
        )

    def _scan_match_record(self, key: tuple[int, int]) -> dict:
        record = self._scan_match_records.get(key)
        if record is None:
            if len(self._scan_match_record_order) == self._scan_match_record_order.maxlen:
                oldest = self._scan_match_record_order.popleft()
                self._scan_match_records.pop(oldest, None)
            record = {"stamp": {"sec": key[0], "nanosec": key[1]}}
            self._scan_match_records[key] = record
            self._scan_match_record_order.append(key)
        return record

    def _on_scan_match_pose(self, msg) -> None:
        orientation = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        with self._scan_match_condition:
            record = self._scan_match_record(self._stamp_key(getattr(msg, "header", None)))
            record["matched_pose"] = {
                "x": float(msg.pose.position.x),
                "y": float(msg.pose.position.y),
                "z": float(msg.pose.position.z),
                "yaw": float(yaw),
                "frame_id": str(msg.header.frame_id or "map"),
            }
            self._scan_match_condition.notify_all()

    def begin_operator_localization(self) -> None:
        """Prevent automatic recovery from preempting an operator pose request."""
        with self._localization_operation_lock:
            self._operator_localization_depth += 1

    def end_operator_localization(self) -> None:
        with self._localization_operation_lock:
            self._operator_localization_depth = max(
                0,
                self._operator_localization_depth - 1,
            )
            if self._operator_localization_depth == 0:
                # If the operator attempt ended without reaching normal, the
                # next non-normal sample may hand control back to automatic
                # recovery instead of leaving its previous notification latch
                # stuck forever.
                self._localization_failure_notified = False

    def operator_localization_active(self) -> bool:
        with self._localization_operation_lock:
            return self._operator_localization_depth > 0

    def _start_localization_operation(self, source: str, *, automatic: bool = False) -> int:
        with self._localization_operation_lock:
            if automatic and self._operator_localization_depth > 0:
                raise ProtocolError(
                    "RELOCALIZATION_SUPERSEDED",
                    "automatic localization recovery was suppressed by an operator request",
                )
            self._localization_operation_generation += 1
            generation = self._localization_operation_generation
        LOGGER.info("localization operation %d started by %s", generation, source)
        return generation

    def _assert_localization_operation(self, generation: int) -> None:
        with self._localization_operation_lock:
            current = self._localization_operation_generation
        if generation != current:
            raise ProtocolError(
                "RELOCALIZATION_SUPERSEDED",
                f"localization operation {generation} was superseded by {current}",
            )

    def cancel_localization_operations(self) -> None:
        """Invalidate in-flight localization workers owned by a finished task."""
        with self._localization_operation_lock:
            self._localization_operation_generation += 1
            self._localization_failure_notified = False
        LOGGER.info(
            "invalidated in-flight localization operations generation=%s",
            self._localization_operation_generation,
        )

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
        started = time.perf_counter()
        dropped = 0
        # Parking confirmation is safety-critical and must observe the 10 Hz
        # FAST-LIO twist directly. Keep this tiny state update in the
        # dedicated safety callback instead of waiting for the throttled
        # telemetry consumer below.
        self._update_standstill_odometry(msg)
        optimization_enabled = self._callback_optimization_enabled()
        if optimization_enabled:
            dropped = int(self._odometry_sequence > self._odometry_consumed_sequence)
            self._odometry_sequence += 1
            self._latest_odometry = msg
        else:
            self._process_odometry(msg)
        if self._callback_metrics_enabled():
            self._callback_performance.record(
                "odometry_callback",
                time.perf_counter() - started,
                processed=not optimization_enabled,
                dropped=dropped,
                queue_depth=self._latest_queue_depth(),
            )

    def _update_standstill_odometry(self, msg) -> None:
        twist = getattr(getattr(msg, "twist", None), "twist", None)
        linear = getattr(twist, "linear", None)
        angular = getattr(twist, "angular", None)
        try:
            values = (
                float(getattr(linear, "x")),
                float(getattr(linear, "y")),
                float(getattr(angular, "z")),
            )
        except (AttributeError, TypeError, ValueError):
            self._standstill_started_monotonic = None
            return
        if not all(math.isfinite(value) for value in values):
            self._standstill_started_monotonic = None
            return
        self._odom_linear_x, self._odom_linear_y, self._odom_angular_z = values
        now = time.monotonic()
        self._odom_received_monotonic = now
        # Advance/reset the single continuous-stillness window on the LIO
        # callback itself. is_robot_stopped() only waits for this result.
        self._is_stopped_from_velocity(now)

    def _consume_latest_odometry(self) -> None:
        if self._odometry_sequence <= self._odometry_consumed_sequence:
            return
        msg = self._latest_odometry
        self._odometry_consumed_sequence = self._odometry_sequence
        started = time.perf_counter()
        self._process_odometry(msg)
        if self._callback_metrics_enabled():
            self._callback_performance.record(
                "odometry_processing",
                time.perf_counter() - started,
                queue_depth=self._latest_queue_depth(),
            )

    def _process_odometry(self, msg) -> None:
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

    def set_localization_policy(
        self,
        source: str,
        phase: str,
        anchor_preference: str = "balanced",
        rtk_primary_allowed: bool = False,
        online_anchor_correction_allowed: bool = False,
    ) -> dict:
        normalized_source = str(source).strip().lower()
        source = normalized_source if normalized_source in {"ndt", "rtk", "ukf"} else "ndt"
        phase = "moving" if str(phase).lower() == "moving" else "stationary"
        anchor_preference = str(anchor_preference or "balanced").strip().lower()
        if anchor_preference not in {"ndt", "rtk", "balanced"}:
            anchor_preference = "balanced"
        rtk_primary_allowed = bool(rtk_primary_allowed) and source == "rtk"
        online_anchor_correction_allowed = (
            bool(online_anchor_correction_allowed) and phase == "moving"
        )
        msg = String()
        # Keep the original first two fields intact for older localization
        # nodes; newer nodes consume the anchor preference and explicit RTK
        # primary opt-in after the second colon.
        msg.data = (
            f"{phase}:{source}:{anchor_preference}:{int(rtk_primary_allowed)}:"
            f"{int(online_anchor_correction_allowed)}"
        )
        self._localization_policy_pub.publish(msg)
        return {
            "topic": "/localization/policy",
            "source": source,
            "phase": phase,
            "anchor_preference": anchor_preference,
            "rtk_primary_allowed": rtk_primary_allowed,
            "online_anchor_correction_allowed": online_anchor_correction_allowed,
        }

    def localization_decision(self) -> dict:
        return self.telemetry.localization_decision()

    def localization_diagnostics(self) -> dict:
        return self.telemetry.localization_diagnostics()

    def control_localization_correction(
        self,
        transaction_id: str,
        mode: str,
        command: str = "start",
        timeout_seconds: float = 2.0,
    ) -> dict:
        client = self._localization_correction_client
        if client is None:
            return {
                "accepted": False,
                "status": "unavailable",
                "message": "ControlLocalizationCorrection interface is unavailable",
            }
        if not client.wait_for_service(timeout_sec=max(0.0, timeout_seconds)):
            return {
                "accepted": False,
                "status": "unavailable",
                "message": "/localization/control_correction is unavailable",
            }
        request = ControlLocalizationCorrection.Request()
        request.transaction_id = str(transaction_id)
        request.command = (
            request.COMMAND_CANCEL if str(command).lower() == "cancel" else request.COMMAND_START
        )
        normalized_mode = str(mode).lower()
        request.mode = {
            "rtk": request.MODE_RTK,
            "ukf": request.MODE_UKF,
        }.get(normalized_mode, request.MODE_NDT)
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout=max(0.0, timeout_seconds)):
            return {"accepted": False, "status": "timeout", "message": "service timeout"}
        if future.exception() is not None:
            return {
                "accepted": False,
                "status": "failed",
                "message": str(future.exception()),
            }
        response = future.result()
        return {
            "accepted": bool(response.accepted),
            "transaction_id": str(response.transaction_id),
            "status": str(response.status),
            "message": str(response.message),
        }

    def set_localization_fusion_profile(
        self,
        profile: str,
        *,
        reason: str = "",
        duration_seconds: float = 180.0,
        expected_generation: int = 0,
        timeout_seconds: float = 2.0,
    ) -> dict:
        client = self._localization_fusion_profile_client
        if client is None:
            return {
                "accepted": False,
                "profile_name": "unavailable",
                "message": "SetLocalizationFusionProfile interface is unavailable",
            }
        if not client.wait_for_service(timeout_sec=max(0.0, timeout_seconds)):
            return {
                "accepted": False,
                "profile_name": "unavailable",
                "message": "/localization/set_fusion_profile is unavailable",
            }
        request = SetLocalizationFusionProfile.Request()
        normalized = str(profile or "nominal").strip().lower()
        request.profile = {
            "lio_hold": request.PROFILE_LIO_HOLD,
            "balanced": request.PROFILE_BALANCED,
        }.get(normalized, request.PROFILE_NOMINAL)
        request.reason = str(reason)
        request.duration_seconds = (
            0.0 if request.profile == request.PROFILE_NOMINAL
            else max(1.0, min(180.0, float(duration_seconds)))
        )
        request.expected_generation = max(0, int(expected_generation))
        future = client.call_async(request)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout=max(0.0, timeout_seconds)):
            return {"accepted": False, "profile_name": normalized, "message": "service timeout"}
        if future.exception() is not None:
            return {
                "accepted": False,
                "profile_name": normalized,
                "message": str(future.exception()),
            }
        response = future.result()
        return {
            "accepted": bool(response.accepted),
            "applied_profile": int(response.applied_profile),
            "generation": int(response.generation),
            "profile_name": str(response.profile_name),
            "message": str(response.message),
        }

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
        self._latest_localization_status = int(msg.status)
        self.telemetry.on_localization(msg)
        status = int(msg.status)
        with self._localization_sample_condition:
            self._localization_sample_sequence += 1
            self._localization_status_samples.append(
                (self._localization_sample_sequence, status)
            )
            self._localization_sample_condition.notify_all()
        if status != 3:
            if (
                self._rtk_is_good_for_navigation()
                or self._rtk_fixed_solution_available()
                or self._rtk_usable_for_recovery()
            ):
                self._localization_lost_count = 0
            else:
                self._localization_lost_count += 1
        else:
            self._localization_lost_count = 0
            self._localization_failure_notified = False
            latest = self.telemetry.latest_pose()
            absolute_stable = self._absolute_localization_stable()
            trusted_pose_open = not bool(getattr(self, "_trusted_pose_frozen", False))
            if latest and absolute_stable and trusted_pose_open:
                self._update_last_trusted_pose(latest)
            stable_for = time.monotonic() - self.safety_state.localization_normal_since_monotonic
            if (
                self._trusted_pose_cb
                and absolute_stable
                and trusted_pose_open
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
            and not self.operator_localization_active()
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

    def set_attempt_progress_callback(self, callback: Callable | None) -> None:
        self._attempt_progress_cb = callback

    def set_recovery_lease_callbacks(
        self,
        acquire_callback: Callable,
        release_callback: Callable,
        snapshot_callback: Callable,
    ) -> None:
        """Expose the Edge recovery arbiter to BT actions over ROS."""
        self._recovery_lease_acquire_cb = acquire_callback
        self._recovery_lease_release_cb = release_callback
        self._recovery_snapshot_cb = snapshot_callback

    def set_self_healing_callback(self, callback: Callable) -> None:
        """Expose the Edge-owned diagnosis/decision coordinator to BT nodes."""
        self._self_healing_cb = callback

    def self_healing_evidence(self) -> dict:
        decision = self._localization_decision()
        obstacle = self.obstacle_monitor_snapshot()
        return {
            "localization_status": self._latest_localization_status,
            "localization_decision": decision,
            "obstacle": obstacle,
            "sensor_stale": bool(
                decision.get("lio_healthy") is False
                and decision.get("ndt_healthy") is False
                and decision.get("rtk_usable") is False
            ),
        }

    @staticmethod
    def _fill_self_healing_response(response, result: dict | None):
        result = result or {}
        response.accepted = bool(result.get("accepted", False))
        response.episode_id = str(result.get("episode_id") or "")
        response.action_id = str(result.get("action_id") or "")
        response.fault_label = str(result.get("fault_label") or "")
        response.scene_mode = str(result.get("scene_mode") or "indoor")
        response.level = int(result.get("level", 0))
        response.action_type = str(result.get("action_type") or "")
        response.forbid_spin = bool(result.get("forbid_spin", False))
        response.wait_for_rtk = bool(result.get("wait_for_rtk", False))
        response.use_lio_hold = bool(result.get("use_lio_hold", False))
        response.search_laser = bool(result.get("search_laser", False))
        response.recovered = bool(result.get("recovered", False))
        response.reason = str(result.get("reason") or "")
        return response

    def _on_navigation_self_healing(self, request, response):
        if self._self_healing_cb is None:
            return self._fill_self_healing_response(
                response, {"accepted": False, "reason": "self_healing_coordinator_unavailable"}
            )
        try:
            result = self._self_healing_cb({
                "operation": int(request.operation),
                "episode_id": str(request.episode_id),
                "action_id": str(request.action_id),
                "fault_label": str(request.fault_label),
                "level": int(request.level),
                "action_type": str(request.action_type),
                "success": bool(request.success),
                "detail": str(request.detail),
                "evidence": self.self_healing_evidence(),
            })
        except Exception as exc:
            LOGGER.exception("self-healing service callback failed")
            result = {"accepted": False, "reason": str(exc)}
        return self._fill_self_healing_response(response, result)

    @staticmethod
    def _fill_recovery_lease_response(response, *, granted: bool, lease=None, snapshot=None):
        snapshot = snapshot or {}
        budget = snapshot.get("budget", {})
        response.granted = bool(granted)
        response.generation = int(getattr(lease, "generation", snapshot.get("recovery_generation", 0)))
        response.owner = str(getattr(lease, "owner", snapshot.get("owner", "NONE")))
        response.reason = str(getattr(lease, "reason", snapshot.get("reason", "")))
        response.attempts = int(budget.get("attempts", 0))
        response.max_attempts = int(budget.get("max_attempts", 0))
        response.distance_m = float(budget.get("distance_m", 0.0))
        response.max_distance_m = float(budget.get("max_distance_m", 0.0))
        response.exhausted = bool(budget.get("exhausted", False))
        return response

    def _on_recovery_lease(self, request, response):
        """Bridge BT recovery ownership to the TaskExecutor arbiter."""
        snapshot = self._recovery_snapshot_cb() if self._recovery_snapshot_cb else {}
        if request.operation == NavigationRecoveryLease.Request.ACQUIRE:
            if not self._recovery_lease_acquire_cb:
                return self._fill_recovery_lease_response(response, granted=False, snapshot=snapshot)
            lease = self._recovery_lease_acquire_cb(
                str(request.owner), str(request.reason), distance_m=float(request.distance_m)
            )
            if lease is not None:
                self._bt_recovery_leases[int(lease.generation)] = lease
                snapshot = self._recovery_snapshot_cb() if self._recovery_snapshot_cb else snapshot
                return self._fill_recovery_lease_response(
                    response, granted=True, lease=lease, snapshot=snapshot
                )
            snapshot = self._recovery_snapshot_cb() if self._recovery_snapshot_cb else snapshot
            return self._fill_recovery_lease_response(response, granted=False, snapshot=snapshot)

        if request.operation == NavigationRecoveryLease.Request.RELEASE:
            lease = self._bt_recovery_leases.pop(int(request.generation), None)
            released = bool(lease and self._recovery_lease_release_cb and self._recovery_lease_release_cb(lease))
            snapshot = self._recovery_snapshot_cb() if self._recovery_snapshot_cb else snapshot
            return self._fill_recovery_lease_response(
                response, granted=released, lease=lease if released else None, snapshot=snapshot
            )

        return self._fill_recovery_lease_response(response, granted=False, snapshot=snapshot)

    def reclaim_recovery_lease(self, generation: int) -> bool:
        """Drop an expired BT lease after TaskExecutor confirmed zero motion.

        The BT release RPC is intentionally asynchronous. If a navigator goal
        is cancelled while that RPC is lost, retain neither the adapter's old
        generation mapping nor the Edge movement ownership.
        """
        lease = self._bt_recovery_leases.pop(int(generation), None)
        if lease is None or not self._recovery_lease_release_cb:
            return False
        return bool(self._recovery_lease_release_cb(lease))

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
        pose = getattr(self, "_last_trusted_pose", None)
        return dict(pose) if pose else None

    def invalidate_last_trusted_pose(self) -> None:
        """Drop a prior task's in-memory seed and freeze writes during startup."""
        self._last_trusted_pose = None
        self._last_trusted_pose_report_monotonic = 0.0
        self._trusted_pose_frozen = True

    def accept_startup_trusted_pose(self) -> None:
        """Persist only the fresh, verified pose produced by this task's startup."""
        decision = self._localization_decision()
        latest = self.telemetry.latest_pose()
        if not (
            latest
            and self._fast_lio_handoff_ready(decision)
        ):
            raise ProtocolError(
                "LIO_HANDOFF_TIMEOUT",
                "startup absolute pose was accepted but FAST-LIO handoff is not ready",
                details={"localization_decision": decision},
            )
        self._trusted_pose_frozen = False
        self._update_last_trusted_pose(latest)
        if self._trusted_pose_cb:
            self._trusted_pose_cb(latest)
            self._last_trusted_pose_report_monotonic = time.monotonic()

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
        started = time.perf_counter()
        LOGGER.debug(
            "scan matching status: converged=%s matching_error=%.3f inlier_fraction=%.3f",
            bool(getattr(msg, "has_converged", False)),
            float(getattr(msg, "matching_error", 0.0)),
            float(getattr(msg, "inlier_fraction", 0.0)),
        )
        score = float(getattr(msg, "matching_error", float("inf")))
        converged = bool(getattr(msg, "has_converged", False))
        healthy = converged and math.isfinite(score) and (
            score < self.safety_config.ndt_failure_score
        )
        signature = (converged, healthy)
        now_monotonic = time.monotonic()
        optimization = getattr(
            self, "_callback_optimization", RosCallbackOptimizationConfig()
        )
        include_predictions = bool(
            not healthy
            or signature != getattr(self, "_last_scan_matching_signature", None)
            or now_monotonic - getattr(self, "_last_scan_matching_detail_monotonic", 0.0)
            >= max(
                1.0,
                float(
                    optimization.scan_matching_full_detail_interval_seconds
                ),
            )
        )
        self.telemetry.on_scan_matching_status(
            msg,
            include_predictions=include_predictions,
        )
        self._last_scan_matching_signature = signature
        if include_predictions:
            self._last_scan_matching_detail_monotonic = now_monotonic
        scan_condition = getattr(self, "_scan_match_condition", None)
        if scan_condition is not None:
            with scan_condition:
                self._scan_match_sequence += 1
                record = self._scan_match_record(self._stamp_key(getattr(msg, "header", None)))
                record.update({
                    "sequence": self._scan_match_sequence,
                    "has_converged": bool(getattr(msg, "has_converged", False)),
                    "matching_error": float(getattr(msg, "matching_error", float("inf"))),
                    "inlier_fraction": float(getattr(msg, "inlier_fraction", 0.0)),
                })
                scan_condition.notify_all()
        # Outdoor RTK-primary navigation still publishes NDT as a shadow health
        # check. Open sky often has an empty/poor scan match even when GPS pose
        # is centimetre-grade. That must not pause the task as localization loss.
        # Indoor LIO-primary is different: status can stay 3 while the published
        # pose has left the map, which is when the dog starts walking randomly.
        if self._rtk_is_navigation_pose_source() or self._rtk_fixed_solution_available():
            self._ndt_failure_count = 0
            self._ndt_failure_notified = False
            self._record_scan_matching_performance(started)
            return
        decision = self.telemetry.localization_decision()
        if (
            decision.get("active_source") == "lio_imu"
            and decision.get("lio_healthy") is True
        ):
            # NDT is a consistency/anchor observer. Its loss alone does not
            # invalidate healthy FAST-LIO motion; a stopped waypoint whose
            # policy requires NDT remains parked by the correction transaction.
            self._ndt_failure_count = 0
            self._ndt_failure_notified = False
            return
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
            self._record_scan_matching_performance(started)
            return
        if healthy:
            self._ndt_failure_count = 0
            self._ndt_failure_notified = False
            self._record_scan_matching_performance(started)
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
        self._record_scan_matching_performance(started)

    def _record_scan_matching_performance(self, started: float) -> None:
        if self._callback_metrics_enabled():
            self._callback_performance.record(
                "scan_matching_callback",
                time.perf_counter() - started,
                queue_depth=self._latest_queue_depth(),
            )

    def _localization_decision(self) -> dict:
        getter = getattr(self.telemetry, "localization_decision", None)
        decision = getter() if callable(getter) else {}
        return decision if isinstance(decision, dict) else {}

    def _rtk_is_good_for_navigation(self) -> bool:
        decision = self._localization_decision()
        if decision.get("rtk_good_for_navigation") is True:
            return True
        return (
            decision.get("rtk_usable") is True
            and str(decision.get("rtk_quality") or "").lower() == "fixed"
            and decision.get("rtk_heading_usable") is True
        )

    def _rtk_position_good_for_navigation(self) -> bool:
        decision = self._localization_decision()
        if decision.get("rtk_position_good_for_navigation") is True:
            return True
        return self._rtk_fixed_solution_available()

    def _rtk_fixed_solution_available(self) -> bool:
        decision = self._localization_decision()
        return (
            decision.get("rtk_usable") is True
            and str(decision.get("rtk_quality") or "").lower() == "fixed"
        )

    def _rtk_usable_for_recovery(self) -> bool:
        decision = self._localization_decision()
        return (
            decision.get("rtk_usable") is True
            and decision.get("rtk_heading_usable") is True
        )

    def _trusted_seed_max_drift_m(self) -> float:
        return float(getattr(self.safety_config, "localization_trusted_seed_max_drift_m", 15.0))

    def _update_last_trusted_pose(self, latest) -> None:
        candidate = {
            "x": float(latest.x),
            "y": float(latest.y),
            "z": float(latest.z),
            "yaw": float(latest.yaw),
            "sampled_at": latest.sampled_at,
            "frame_id": "map",
        }
        previous = getattr(self, "_last_trusted_pose", None)
        if previous and previous.get("x") is not None and previous.get("y") is not None:
            drift = planar_distance_m(previous, candidate)
            if drift > self._trusted_seed_max_drift_m():
                LOGGER.warning(
                    "skip in-memory last_trusted update: jump %.1fm exceeds %.1fm",
                    drift,
                    self._trusted_seed_max_drift_m(),
                )
                return
        self._last_trusted_pose = candidate

    def _rtk_is_navigation_pose_source(self) -> bool:
        if self._rtk_is_good_for_navigation() or self._rtk_fixed_solution_available():
            return True
        decision = self._localization_decision()
        return (
            decision.get("active_source") == "rtk_imu"
            and decision.get("rtk_usable") is True
        )

    def _lio_is_navigation_pose_source(self) -> bool:
        decision = self.telemetry.localization_decision()
        return decision.get("active_source") == "lio_imu"

    def _absolute_localization_stable(self) -> bool:
        decision = self._localization_decision()
        source = decision.get("active_source")
        policy_source_ready = decision.get("policy_source_ready")
        if source == "rtk_imu" and self._rtk_is_good_for_navigation():
            return bool(decision.get("absolute_stable"))
        return bool(
            source in {"ndt_imu", "rtk_imu", "lio_imu"}
            and decision.get("absolute_stable")
            and (policy_source_ready is None or policy_source_ready is True)
        )

    def _fast_lio_handoff_ready(self, decision: dict | None = None) -> bool:
        """Whether FAST-LIO + IMU is the settled continuous pose source.

        RTK and NDT are stationary absolute-correction inputs. They may
        verify a candidate, but task startup and any successful initial-pose
        command must wait until the corrected anchor has handed ownership back
        to FAST-LIO + IMU. This prevents Nav2 from starting on an NDT/RTK
        transient frame.
        """
        decision = decision if isinstance(decision, dict) else self._localization_decision()
        # ``policy_source_ready`` describes whether the configured absolute
        # correction source (RTK/NDT/UKF) is currently eligible.  It is not a
        # prerequisite for FAST-LIO to resume as the continuous pose source;
        # indoor NDT maps commonly report it false after the one-shot
        # correction has completed.
        return bool(
            decision.get("active_source") == "lio_imu"
            and decision.get("lio_healthy") is True
            and decision.get("lio_anchored") is True
            and decision.get("absolute_stable") is True
        )

    def _on_cmd_vel_raw(self, msg) -> None:
        self._raw_forward_command = float(msg.linear.x)
        self._raw_lateral_command = float(msg.linear.y)
        self._raw_turn_command = float(msg.angular.z)
        self._raw_velocity_updated_monotonic = time.monotonic()
        if self._velocity_command_requests_motion(
            self._raw_forward_command,
            self._raw_lateral_command,
            self._raw_turn_command,
        ):
            self._standstill_started_monotonic = None

    def _on_cmd_vel(self, msg) -> None:
        self._actual_forward_command = float(msg.linear.x)
        self._actual_lateral_command = float(msg.linear.y)
        self._actual_turn_command = float(msg.angular.z)
        self._actual_velocity_updated_monotonic = time.monotonic()
        if self._velocity_command_requests_motion(
            self._actual_forward_command,
            self._actual_lateral_command,
            self._actual_turn_command,
        ):
            self._standstill_started_monotonic = None

    def _on_scan(self, scan) -> None:
        started = time.perf_counter()
        dropped = 0
        optimization_enabled = self._callback_optimization_enabled()
        self._latest_scan_received_monotonic = time.monotonic()
        self._latest_scan = scan
        if optimization_enabled:
            dropped = int(
                getattr(self, "_scan_sequence", 0)
                > getattr(self, "_scan_consumed_sequence", 0)
            )
            self._scan_sequence = getattr(self, "_scan_sequence", 0) + 1
        else:
            self._process_scan(scan)
        if self._callback_metrics_enabled():
            self._callback_performance.record(
                "scan_callback",
                time.perf_counter() - started,
                processed=not optimization_enabled,
                dropped=dropped,
                queue_depth=self._latest_queue_depth(),
            )

    def _consume_latest_scan(self) -> None:
        if self._scan_sequence <= self._scan_consumed_sequence:
            return
        scan = self._latest_scan
        self._scan_consumed_sequence = self._scan_sequence
        started = time.perf_counter()
        self._process_scan(scan)
        if self._callback_metrics_enabled():
            self._callback_performance.record(
                "scan_processing",
                time.perf_counter() - started,
                queue_depth=self._latest_queue_depth(),
            )

    def _scan_geometry_for(self, scan) -> list[tuple[int, float, float]]:
        key = (
            len(scan.ranges),
            round(float(scan.angle_min), 12),
            round(float(scan.angle_increment), 12),
        )
        if key != getattr(self, "_scan_geometry_key", None):
            geometry = []
            for index in range(len(scan.ranges)):
                angle = scan.angle_min + index * scan.angle_increment
                cosine = math.cos(angle)
                geometry.append((index, cosine, math.sin(angle)))
            self._scan_geometry_key = key
            self._scan_geometry = geometry
        return getattr(self, "_scan_geometry", [])

    def _process_scan(self, scan) -> None:
        nearest = None
        measured_range = float(getattr(scan, "range_max", 0.0) or 0.0)
        verified_open_range = measured_range if math.isfinite(measured_range) and measured_range > 0 else None
        # A finite LaserScan range_max is verified free-space evidence. Keep
        # None reserved for unavailable directional data so recovery can fail
        # closed without pretending that missing data means infinity.
        rear = verified_open_range
        left = verified_open_range
        right = verified_open_range
        for index, cosine, sine in self._scan_geometry_for(scan):
            distance = scan.ranges[index]
            if not math.isfinite(distance) or distance < scan.range_min:
                continue
            x = distance * cosine
            y = distance * sine
            if distance <= 0.9 and x >= 0.18 and abs(y) <= 0.35:
                nearest = distance if nearest is None else min(nearest, distance)
            if -2.5 <= x <= -0.18 and abs(y) <= 0.35:
                rear = distance if rear is None else min(rear, distance)
            if -0.35 <= x <= 2.5 and 0.18 <= y <= 2.0:
                left = distance if left is None else min(left, distance)
            if -0.35 <= x <= 2.5 and -2.0 <= y <= -0.18:
                right = distance if right is None else min(right, distance)
        self._front_obstacle_distance_m = nearest
        self._rear_clearance_m = rear
        self._left_clearance_m = left
        self._right_clearance_m = right

    def _subscribe_imu_cross_check(self) -> None:
        """Subscribe both gyro feeds, if the pieces for the comparison exist.

        Pure monitoring: nothing here feeds the localization or control path,
        so every missing piece degrades to "no comparison" instead of an error.
        """
        config = self._imu_cross_check.config
        if not config.enabled:
            return
        self.create_subscription(
            Imu, config.lidar_imu_topic, self._on_lidar_imu, qos_profile_sensor_data,
            callback_group=self._telemetry_callback_group,
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
            callback_group=self._telemetry_callback_group,
        )

    def _on_lidar_imu(self, msg) -> None:
        # Also the evaluation trigger: this feed runs at 200 Hz and is always
        # present, so driving the comparison from here is what lets a missing
        # 3588 stream be reported as absent rather than simply going quiet.
        started = time.perf_counter()
        dropped = 0
        optimization_enabled = self._callback_optimization_enabled()
        if optimization_enabled:
            dropped = int(self._lidar_imu_sequence > self._lidar_imu_consumed_sequence)
            self._lidar_imu_sequence += 1
            self._latest_lidar_imu = msg
        else:
            self._process_lidar_imu(msg)
        if self._callback_metrics_enabled():
            self._callback_performance.record(
                "imu_callback",
                time.perf_counter() - started,
                processed=not optimization_enabled,
                dropped=dropped,
                queue_depth=self._latest_queue_depth(),
            )

    def _on_high_level_robot_state(self, msg) -> None:
        started = time.perf_counter()
        dropped = 0
        optimization_enabled = self._callback_optimization_enabled()
        if optimization_enabled:
            dropped = int(self._body_imu_sequence > self._body_imu_consumed_sequence)
            self._body_imu_sequence += 1
            self._latest_body_imu = msg
        else:
            self._process_body_imu(msg)
        if self._callback_metrics_enabled():
            self._callback_performance.record(
                "imu_callback",
                time.perf_counter() - started,
                processed=not optimization_enabled,
                dropped=dropped,
                queue_depth=self._latest_queue_depth(),
            )

    def _consume_latest_imu(self) -> None:
        started = time.perf_counter()
        if self._body_imu_sequence > self._body_imu_consumed_sequence:
            self._body_imu_consumed_sequence = self._body_imu_sequence
            self._process_body_imu(self._latest_body_imu)
        if self._lidar_imu_sequence > self._lidar_imu_consumed_sequence:
            self._lidar_imu_consumed_sequence = self._lidar_imu_sequence
            self._process_lidar_imu(self._latest_lidar_imu)
        if self._callback_metrics_enabled():
            self._callback_performance.record(
                "imu_processing",
                time.perf_counter() - started,
                queue_depth=self._latest_queue_depth(),
            )

    def _process_lidar_imu(self, msg) -> None:
        gyro = msg.angular_velocity
        now = time.monotonic()
        with self._imu_cross_check_lock:
            self._imu_cross_check.add_lidar_gyro(now, gyro.x, gyro.y, gyro.z)
            if not self._imu_cross_check.due(now):
                return
            report = self._imu_cross_check.evaluate(now)
        self._publish_imu_cross_check(report)

    def _process_body_imu(self, msg) -> None:
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
        now_monotonic = time.monotonic()
        plan_age = (
            max(0.0, now_monotonic - self._global_plan_updated_monotonic)
            if self._global_plan_updated_monotonic
            else None
        )
        requested_velocity_age = (
            max(0.0, now_monotonic - self._raw_velocity_updated_monotonic)
            if self._raw_velocity_updated_monotonic
            else None
        )
        actual_velocity_age = (
            max(0.0, now_monotonic - self._actual_velocity_updated_monotonic)
            if self._actual_velocity_updated_monotonic
            else None
        )
        scan_age = (
            max(0.0, now_monotonic - getattr(self, "_latest_scan_received_monotonic", 0.0))
            if getattr(self, "_latest_scan_received_monotonic", 0.0)
            else None
        )
        scan_max_age = float(getattr(
            getattr(self, "safety_config", None), "arrival_adjust_scan_max_age_seconds", 0.5
        ))
        plan_fresh = bool(
            plan_age is not None
            and plan_age <= self._global_plan_stale_seconds
            and self._global_plan_points
        )
        collision_state_age = (
            max(
                0.0,
                now_monotonic
                - float(getattr(self, "_collision_monitor_state_received_monotonic", 0.0)),
            )
            if getattr(self, "_collision_monitor_state_received_monotonic", 0.0)
            else None
        )
        collision_state = dict(getattr(self, "_collision_monitor_state", {}) or {})
        collision_state["sample_age_seconds"] = (
            round(collision_state_age, 3) if collision_state_age is not None else None
        )
        return {
            "requested_forward_speed_mps": self._raw_forward_command,
            "actual_forward_speed_mps": self._actual_forward_command,
            "requested_planar_speed_mps": math.hypot(self._raw_forward_command, self._raw_lateral_command),
            "actual_planar_speed_mps": math.hypot(self._actual_forward_command, self._actual_lateral_command),
            "requested_turn_speed_rps": self._raw_turn_command,
            "actual_turn_speed_rps": self._actual_turn_command,
            "requested_velocity_sample_age_seconds": (
                round(requested_velocity_age, 3) if requested_velocity_age is not None else None
            ),
            "actual_velocity_sample_age_seconds": (
                round(actual_velocity_age, 3) if actual_velocity_age is not None else None
            ),
            "localized_speed_mps": self._latest_speed,
            "front_obstacle_distance_m": self._front_obstacle_distance_m,
            "rear_clearance_m": getattr(self, "_rear_clearance_m", None),
            "left_clearance_m": self._left_clearance_m,
            "right_clearance_m": self._right_clearance_m,
            "rear_obstacle_distance_m": getattr(self, "_rear_clearance_m", None),
            "left_obstacle_distance_m": self._left_clearance_m,
            "right_obstacle_distance_m": self._right_clearance_m,
            "collision_monitor": collision_state,
            "scan_sample_age_seconds": round(scan_age, 3) if scan_age is not None else None,
            "stale": scan_age is None or scan_age > scan_max_age,
            "collision_limited": bool(
                math.hypot(self._raw_forward_command, self._raw_lateral_command) > 0.03
                and math.hypot(self._actual_forward_command, self._actual_lateral_command)
                < 0.5 * math.hypot(self._raw_forward_command, self._raw_lateral_command)
            ),
            "localization_normal": getattr(
                getattr(self, "safety_state", None), "localization_status", ""
            ) == "normal",
            "global_plan": {
                "updated": plan_fresh,
                "sample_age_seconds": round(plan_age, 3) if plan_age is not None else None,
                "points": list(self._global_plan_points) if plan_fresh else [],
            },
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

    @staticmethod
    def _wait_for_future(future, timeout_seconds: float) -> bool:
        if future is None:
            return False
        if future.done():
            return True
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if future.done():
            return True
        return completed.wait(timeout=timeout_seconds)

    def _destroy_client_if_idle(self, client, future=None) -> None:
        """Destroy a one-shot client only after the executor is done with it.

        rclpy keeps the client handle in the wait set while a request is in
        flight. Destroying it there raises InvalidHandle inside ros-executor
        and stops all pose/status callbacks.
        """
        if client is None:
            return
        if future is not None and not future.done():
            LOGGER.warning(
                "leaving ROS service client alive until its in-flight request finishes"
            )
            return
        try:
            self.destroy_client(client)
        except Exception:
            LOGGER.debug("destroy_client failed", exc_info=True)

    def _persistent_service_client(self, service_type, service_name: str):
        """Return one node-lifetime client for a service endpoint."""
        key = (service_type, str(service_name))
        lock = getattr(self, "_service_clients_lock", None)
        if lock is None:
            self._service_clients_lock = threading.Lock()
            lock = self._service_clients_lock
        with lock:
            clients = getattr(self, "_service_clients", None)
            if clients is None:
                self._service_clients = {}
                clients = self._service_clients
            client = clients.get(key)
            if client is None:
                client = self.create_client(
                    service_type,
                    service_name,
                    callback_group=self._nav_service_callback_group,
                )
                clients[key] = client
            return client

    def _lifecycle_node_is_active(self, node_name: str) -> bool:
        client = self._persistent_service_client(
            GetState,
            f"{node_name}/get_state",
        )
        try:
            if not client.wait_for_service(timeout_sec=0.25):
                return False
            future = client.call_async(GetState.Request())
            if not self._wait_for_future(future, 0.75) or future.result() is None:
                return False
            # lifecycle_msgs/State.PRIMARY_STATE_ACTIVE == 3.
            return int(future.result().current_state.id) == 3
        except Exception:
            return False

    # Patrol goal dispatch: prefer fail-fast + Edge retry over a 30s stand.
    _NAV_SEND_READY_TIMEOUT_SECONDS = 5.0
    # Remember unavailable parameter services briefly across waypoints.
    _REMOTE_PARAM_UNAVAILABLE_COOLDOWN_SECONDS = 120.0

    def send_waypoints(self, waypoints: list[dict], feedback_cb: Callable, result_cb: Callable) -> bool:
        if len(waypoints) != 1:
            LOGGER.error(
                "rejecting non-single waypoint navigation request (count=%d); "
                "Edge navigation semantics require one action per waypoint",
                len(waypoints),
            )
            return False
        if not self.wait_until_ready(timeout_seconds=self._NAV_SEND_READY_TIMEOUT_SECONDS):
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
        self._sent_pose_count = 1
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
        self._result_cb = result_cb
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut, cb=result_cb: self._on_result(fut, cb))
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

    def _on_result(self, future, result_cb: Callable | None = None) -> None:
        callback = result_cb if result_cb is not None else self._result_cb
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            result = getattr(wrapped, "result", None)
            missed_waypoints = self._extract_missed_waypoints(result)
            # action_msgs/GoalStatus: SUCCEEDED=4, CANCELED=5, ABORTED=6
            mapped = "succeeded" if status == 4 else "cancelled" if status == 5 else "failed"
            if callback:
                callback(
                    mapped,
                    "" if mapped != "failed" else f"goal_status={status}",
                    {"goal_status": status, "missed_waypoints": missed_waypoints},
                )
        except Exception as exc:
            LOGGER.exception("navigation result callback failed")
            if callback:
                callback("failed", str(exc))
        finally:
            if result_cb is None or result_cb is self._result_cb:
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

    def wait_for_pose_update(
        self, after_sampled_at: str | None, timeout_seconds: float = 1.0
    ):
        """Wait for a new final localization pose for arrival confirmation."""
        waiter = getattr(self.telemetry, "wait_for_pose_update", None)
        if callable(waiter):
            return waiter(after_sampled_at, timeout_seconds=timeout_seconds)
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

    def manual_assist_velocity(
        self, vx: float = 0.0, vy: float = 0.0, yaw_rate: float = 0.0
    ) -> dict:
        """Publish bounded operator intent into the safe Nav2 control pipeline."""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(yaw_rate)
        self._manual_assist_cmd_vel_pub.publish(msg)
        return {
            "topic": "/cmd_vel_assist",
            "vx": msg.linear.x,
            "vy": msg.linear.y,
            "yaw_rate": msg.angular.z,
        }

    def search_laser_feature(
        self,
        *,
        max_distance_m: float = 0.15,
        timeout_seconds: float = 4.0,
        allow_rotation: bool = False,
    ) -> bool:
        """Perform one collision-monitored, bounded feature-search motion.

        Commands enter through /cmd_vel_assist, so Collision Monitor remains
        the final authority. Outdoor callers keep ``allow_rotation`` false.
        """
        speed = 0.05
        yaw_rate = 0.10 if allow_rotation else 0.0
        motion_seconds = min(
            max(0.2, max_distance_m / speed),
            max(0.2, float(timeout_seconds)),
        )
        deadline = time.monotonic() + motion_seconds
        try:
            while time.monotonic() < deadline:
                decision = self._localization_decision()
                score = decision.get("ndt_score")
                try:
                    ndt_health = decision.get("ndt_healthy")
                    ndt_good = ndt_health is True or (
                        not isinstance(ndt_health, bool)
                        and score is not None
                        and math.isfinite(float(score))
                        and float(score) < self.safety_config.ndt_failure_score
                    )
                except (TypeError, ValueError):
                    ndt_good = False
                if ndt_good:
                    return True
                observation = self.obstacle_monitor_snapshot()
                if observation.get("stale") is True:
                    LOGGER.warning("laser feature search stopped: scan is stale")
                    return False
                front = observation.get("front_obstacle_distance_m")
                if front is not None and float(front) < 0.8:
                    LOGGER.warning("laser feature search stopped: front clearance %.2fm", float(front))
                    return False
                self.manual_assist_velocity(vx=speed, vy=0.0, yaw_rate=yaw_rate)
                time.sleep(0.1)
            return False
        finally:
            self.manual_assist_velocity(vx=0.0, vy=0.0, yaw_rate=0.0)
            self.stop_motion()

    def arrival_adjust_velocity(
        self, vx: float = 0.0, vy: float = 0.0, yaw_rate: float = 0.0
    ) -> dict:
        """Publish an autonomous fine-adjustment through collision monitor."""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(yaw_rate)
        self._arrival_adjust_cmd_vel_pub.publish(msg)
        return {
            "topic": self.ros_config.cmd_vel_raw_topic,
            "vx": msg.linear.x,
            "vy": msg.linear.y,
            "yaw_rate": msg.angular.z,
        }

    def directional_clearance(
        self,
        vx: float,
        vy: float,
        travel_distance_m: float,
        *,
        max_scan_age_seconds: float = 0.5,
    ) -> dict:
        """Check the 360-degree scan in the commanded translation corridor."""
        scan = self._latest_scan
        received = float(getattr(self, "_latest_scan_received_monotonic", 0.0) or 0.0)
        age = max(0.0, time.monotonic() - received) if received else None
        if scan is None or age is None or age > max(0.0, float(max_scan_age_seconds)):
            return {"clear": False, "reason": "scan_stale", "scan_age_seconds": age}
        speed = math.hypot(float(vx), float(vy))
        if speed <= 1e-6:
            return {"clear": True, "reason": "rotation_only", "scan_age_seconds": age}
        ux, uy = float(vx) / speed, float(vy) / speed
        # Base footprint is about 0.63 x 0.36 m. Project the rectangle into
        # the commanded direction and add a conservative swept-corridor margin.
        longitudinal_extent = abs(ux) * 0.315 + abs(uy) * 0.18
        lateral_extent = abs(-uy) * 0.315 + abs(ux) * 0.18
        corridor_end = longitudinal_extent + max(0.0, float(travel_distance_m)) + 0.15
        corridor_half_width = lateral_extent + 0.10
        nearest = None
        for index, cosine, sine in self._scan_geometry_for(scan):
            distance = scan.ranges[index]
            if not math.isfinite(distance) or distance < scan.range_min:
                continue
            x = float(distance) * cosine
            y = float(distance) * sine
            along = x * ux + y * uy
            lateral = abs(-x * uy + y * ux)
            # Ignore the immediate sensor/footprint return, but cover forward,
            # reverse and lateral motion with the same directional projection.
            if 0.18 <= along <= corridor_end and lateral <= corridor_half_width:
                nearest = along if nearest is None else min(nearest, along)
        return {
            "clear": nearest is None,
            "reason": "clear" if nearest is None else "obstacle",
            "scan_age_seconds": age,
            "nearest_along_m": nearest,
            "required_clearance_m": corridor_end,
        }

    def _execute_behavior_action(self, client, goal, *, name: str, timeout_seconds: float) -> dict:
        """Run one BehaviorServer action while the executor continues spinning."""
        timeout = max(0.1, float(timeout_seconds))
        if not client.wait_for_server(timeout_sec=min(timeout, 1.0)):
            return {"action": name, "status": "unavailable", "success": False}
        sent = client.send_goal_async(goal)
        if not self._wait_for_future(sent, min(timeout, 2.0)):
            return {"action": name, "status": "goal_timeout", "success": False}
        handle = sent.result() if sent.done() else None
        if handle is None or not handle.accepted:
            return {"action": name, "status": "rejected", "success": False}
        lock = getattr(self, "_obstacle_recovery_goal_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._obstacle_recovery_goal_lock = lock
        with lock:
            self._obstacle_recovery_goal_handle = handle
        try:
            result_future = handle.get_result_async()
            if not self._wait_for_future(result_future, timeout):
                cancel_future = handle.cancel_goal_async()
                cancel_confirmed = self._wait_for_future(cancel_future, min(2.0, timeout))
                self.stop_motion()
                return {
                    "action": name,
                    "status": "timeout",
                    "success": False,
                    "cancel_confirmed": cancel_confirmed,
                }
            wrapped = result_future.result()
            status = int(getattr(wrapped, "status", 0) or 0)
            result = getattr(wrapped, "result", None)
            error_code = int(getattr(result, "error_code", 0) or 0)
            return {
                "action": name,
                "status": "succeeded" if status == 4 else "failed",
                "success": status == 4,
                "goal_status": status,
                "error_code": error_code,
            }
        finally:
            with lock:
                if getattr(self, "_obstacle_recovery_goal_handle", None) is handle:
                    self._obstacle_recovery_goal_handle = None

    @staticmethod
    def _set_action_duration(duration, seconds: float) -> None:
        nanoseconds = int(max(0.0, float(seconds)) * 1_000_000_000)
        duration.sec = nanoseconds // 1_000_000_000
        duration.nanosec = nanoseconds % 1_000_000_000

    def execute_obstacle_recovery(
        self,
        *,
        reverse_distance_m: float,
        lateral_distance_m: float,
        lateral_direction: int,
        speed_mps: float,
        timeout_seconds: float,
    ) -> dict:
        """Execute the bounded backup + lateral BehaviorServer sequence."""
        backup = BackUp.Goal()
        backup.target.x = abs(float(reverse_distance_m))
        backup.target.y = 0.0
        backup.target.z = 0.0
        backup.speed = abs(float(speed_mps))
        self._set_action_duration(backup.time_allowance, timeout_seconds)
        actions = [
            self._execute_behavior_action(
                self._backup_client,
                backup,
                name="backup",
                timeout_seconds=timeout_seconds,
            )
        ]
        if not actions[-1]["success"]:
            return {"success": False, "actions": actions}
        lateral = DriveOnHeading.Goal()
        lateral.target.x = 0.0
        lateral.target.y = math.copysign(
            abs(float(lateral_distance_m)),
            1 if int(lateral_direction) >= 0 else -1,
        )
        lateral.target.z = 0.0
        lateral.speed = math.copysign(abs(float(speed_mps)), lateral.target.y)
        self._set_action_duration(lateral.time_allowance, timeout_seconds)
        actions.append(
            self._execute_behavior_action(
                self._drive_on_heading_client,
                lateral,
                name="drive_on_heading",
                timeout_seconds=timeout_seconds,
            )
        )
        return {"success": bool(actions[-1]["success"]), "actions": actions}

    def cancel_obstacle_recovery(self) -> bool:
        lock = getattr(self, "_obstacle_recovery_goal_lock", None)
        if lock is None:
            return False
        with lock:
            handle = getattr(self, "_obstacle_recovery_goal_handle", None)
        if handle is None:
            return False
        try:
            future = handle.cancel_goal_async()
            confirmed = self._wait_for_future(future, 2.0)
            self.stop_motion()
            return bool(confirmed)
        except Exception:
            LOGGER.warning("failed to cancel obstacle recovery action", exc_info=True)
            return False

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

    def _scan_match_sequence_snapshot(self) -> int:
        with self._scan_match_condition:
            return self._scan_match_sequence

    def _best_ndt_candidate(self, after_sequence: int, submitted_pose: dict) -> dict | None:
        with self._scan_match_condition:
            records = [
                {
                    **record,
                    "matched_pose": (
                        dict(record["matched_pose"])
                        if record.get("matched_pose") else None
                    ),
                }
                for record in self._scan_match_records.values()
                if int(record.get("sequence", 0)) > after_sequence
            ]
        if not records:
            return None
        score_limit = float(self.safety_config.ndt_failure_score)
        min_inlier_fraction = 0.50
        usable = [
            record for record in records
            if record.get("has_converged") is True
            and self._finite_or_none(record.get("matching_error")) is not None
            and float(record["matching_error"]) < score_limit
            and (self._finite_or_none(record.get("inlier_fraction")) or 0.0) >= min_inlier_fraction
            and record.get("matched_pose")
        ]
        decision = self.telemetry.localization_decision()
        initialization = dict(decision.get("initialization") or {})
        verified = bool(initialization.get("verified"))
        stable_frames = int(initialization.get("stable_frames") or 0)
        required_frames = max(1, int(initialization.get("required_stable_frames") or 3))

        if usable:
            # When C++ has verified a stable streak, choose the strongest
            # result from that streak rather than an earlier isolated low
            # score that may belong to a different local minimum.
            ranked_pool = sorted(usable, key=lambda item: int(item.get("sequence", 0)))
            if verified or stable_frames >= required_frames:
                ranked_pool = ranked_pool[-required_frames:]
            best = min(
                ranked_pool,
                key=lambda item: (
                    float(item.get("matching_error", float("inf"))),
                    -float(item.get("inlier_fraction", 0.0)),
                ),
            )
        else:
            # A rejected observation is still operational evidence.  Keep the
            # strongest raw sample so every attempted seed reports score,
            # inlier ratio and the exact gate that rejected it.
            best = min(
                records,
                key=lambda item: (
                    0 if item.get("has_converged") is True else 1,
                    self._finite_or_none(item.get("matching_error"))
                    if self._finite_or_none(item.get("matching_error")) is not None
                    else float("inf"),
                    -(self._finite_or_none(item.get("inlier_fraction")) or 0.0),
                    -int(item.get("sequence", 0)),
                ),
            )

        matched_pose = dict(best["matched_pose"]) if best.get("matched_pose") else None
        position_correction_m = None
        yaw_correction_rad = None
        if matched_pose:
            position_correction_m = math.hypot(
                float(matched_pose["x"]) - float(submitted_pose["x"]),
                float(matched_pose["y"]) - float(submitted_pose["y"]),
            )
            yaw_correction_rad = math.atan2(
                math.sin(float(matched_pose["yaw"]) - float(submitted_pose.get("yaw", 0.0))),
                math.cos(float(matched_pose["yaw"]) - float(submitted_pose.get("yaw", 0.0))),
            )
        within_seed_gate = bool(
            position_correction_m is not None
            and yaw_correction_rad is not None
            and position_correction_m <= 1.50
            and abs(yaw_correction_rad) <= math.radians(30.0)
        )
        matching_error = self._finite_or_none(best.get("matching_error"))
        inlier_fraction = self._finite_or_none(best.get("inlier_fraction"))
        quality_failures = []
        if best.get("has_converged") is not True:
            quality_failures.append("ndt_not_converged")
        if matching_error is None:
            quality_failures.append("ndt_score_unavailable")
        elif matching_error >= score_limit:
            quality_failures.append("ndt_score_above_threshold")
        if inlier_fraction is None:
            quality_failures.append("ndt_inlier_fraction_unavailable")
        elif inlier_fraction < min_inlier_fraction:
            quality_failures.append("ndt_inlier_fraction_below_threshold")
        if not matched_pose:
            quality_failures.append("ndt_matched_pose_unavailable")
        if matched_pose and not within_seed_gate:
            if position_correction_m is not None and position_correction_m > 1.50:
                quality_failures.append("seed_position_correction_exceeded")
            if yaw_correction_rad is not None and abs(yaw_correction_rad) > math.radians(30.0):
                quality_failures.append("seed_yaw_correction_exceeded")
        stable_enough = verified or stable_frames >= required_frames
        if not stable_enough:
            quality_failures.append("ndt_stable_frames_insufficient")
        eligible = not quality_failures
        geometric_rmse = self._finite_or_none(best.get("geometric_rmse"))
        return {
            "matched_pose": matched_pose,
            "has_converged": best.get("has_converged") is True,
            "matching_error": matching_error,
            "inlier_fraction": inlier_fraction,
            "geometric_rmse": geometric_rmse,
            "healthy_samples": len(usable),
            "sample_count": len(records),
            "stable_frames": stable_frames,
            "required_stable_frames": required_frames,
            "verified": verified,
            "position_correction_m": position_correction_m,
            "yaw_correction_deg": (
                math.degrees(yaw_correction_rad) if yaw_correction_rad is not None else None
            ),
            "within_seed_gate": within_seed_gate,
            "ndt_score_threshold": score_limit,
            "min_inlier_fraction": min_inlier_fraction,
            "quality_failures": quality_failures,
            "reject_reason": quality_failures[0] if quality_failures else None,
            "eligible": eligible,
        }

    @staticmethod
    def _finite_or_none(value) -> float | None:
        if value is None or value == "":
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    @staticmethod
    def _candidate_rank(candidate: dict | None) -> tuple:
        if not candidate or not candidate.get("eligible"):
            return (
                1,
                0,
                0.0,
                float("inf"),
                float("inf"),
                float("inf"),
                float("inf"),
            )
        rmse = RosAdapter._finite_or_none(candidate.get("geometric_rmse"))
        if rmse is None:
            rmse = float("inf")
        return (
            0,
            -int(candidate.get("stable_frames") or 0),
            -float(candidate.get("inlier_fraction") or 0.0),
            float(candidate.get("matching_error", float("inf"))),
            rmse,
            float(candidate.get("position_correction_m") or float("inf")),
            abs(float(candidate.get("yaw_correction_deg") or 0.0)),
        )

    def _candidate_is_optimal(self, candidate: dict | None) -> bool:
        if not candidate or not candidate.get("eligible"):
            return False
        try:
            score = float(candidate.get("matching_error"))
        except (TypeError, ValueError):
            return False
        required = max(1, int(candidate.get("required_stable_frames") or 3))
        return (
            math.isfinite(score)
            and score < float(getattr(
                getattr(self, "safety_config", None),
                "localization_optimal_ndt_score",
                0.01,
            ))
            and int(candidate.get("stable_frames") or 0) >= required
        )

    @staticmethod
    def _skip_waiting_attempts(attempts: list[dict], reason: str) -> None:
        finished_at = now_iso()
        for item in attempts:
            if item.get("status") != "waiting":
                continue
            item["status"] = "skipped"
            item["reject_reason"] = reason
            item["eligible"] = False
            item["accepted"] = False
            item["updated_at"] = finished_at
            item["finished_at"] = finished_at

    @staticmethod
    def _evaluated_localization_attempt_count(attempts: list[dict]) -> int:
        """Count candidates that actually completed NDT evaluation.

        ``waiting`` and ``skipped`` candidates were never evaluated. A
        qualified or committing winner has completed NDT evaluation even while
        its later FAST-LIO handoff is still pending.
        """
        return sum(
            1
            for item in attempts
            if item.get("status") in {
                "qualified", "committing", "accepted", "rejected", "failed",
            }
        )

    def set_initial_pose(self, pose: dict) -> dict:
        generation = self._start_localization_operation("operator_initial_pose")
        try:
            return self._set_initial_pose_once(pose, generation)
        except ProtocolError as exc:
            if exc.code != "INITIAL_POSE_NOT_ACCEPTED":
                raise
            candidate = dict((exc.details or {}).get("best_ndt_candidate") or {})
            if not candidate.get("eligible"):
                raise
            best_pose = dict(candidate["matched_pose"])
            LOGGER.warning(
                "initial pose NDT verified but handoff was not accepted; committing best match "
                "x=%.3f y=%.3f yaw=%.3f score=%.3f inlier=%.3f",
                best_pose["x"], best_pose["y"], best_pose["yaw"],
                candidate["matching_error"], candidate["inlier_fraction"],
            )
            try:
                result = self._set_initial_pose_once({
                    **pose,
                    **best_pose,
                    "frame_id": "map",
                    "wait_seconds": max(12.0, float(pose.get("wait_seconds", 8.0))),
                    "require_absolute": True,
                }, generation)
                result["best_ndt_candidate"] = candidate
                result["best_ndt_committed"] = True
                return result
            except ProtocolError as handoff_exc:
                if handoff_exc.code == "RELOCALIZATION_SUPERSEDED":
                    raise
                details = dict(handoff_exc.details or {})
                details.update({
                    "best_ndt_candidate": candidate,
                    "best_ndt_committed": True,
                    "handoff_pending": True,
                })
                raise ProtocolError(
                    "RELOCALIZATION_HANDOFF_FAILED",
                    "best NDT pose was committed but FAST-LIO did not become stable",
                    details=details,
                ) from handoff_exc

    def initial_pose_subscriber_ready(self) -> bool:
        """Return whether the localization node can currently receive a seed."""
        return self._initial_pose_pub.get_subscription_count() > 0

    def wait_for_initial_pose_subscriber(
        self,
        timeout_seconds: float = 0.0,
        generation: int | None = None,
    ) -> bool:
        """Wait for the real /initialpose consumer, not a stale ROS graph node."""
        deadline = time.monotonic() + max(0.0, float(timeout_seconds))
        while True:
            if generation is not None:
                self._assert_localization_operation(generation)
            if self.initial_pose_subscriber_ready():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.2)

    def _set_initial_pose_once(self, pose: dict, generation: int) -> dict:
        self._assert_localization_operation(generation)
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
        if not self.wait_for_initial_pose_subscriber(
            timeout_seconds=float(pose.get("subscriber_wait_seconds", 15.0)),
            generation=generation,
        ):
            raise ProtocolError("LOCALIZATION_UNAVAILABLE", "/initialpose has no localization subscriber")
        with self._localization_sample_condition:
            sample_sequence = self._localization_sample_sequence
        scan_match_sequence = self._scan_match_sequence_snapshot()
        # The publisher is reliable and a live subscriber was confirmed above.
        # Publishing once also lets the localization node deliberately re-run
        # an unchanged pose while lost without five duplicate resets.
        self._assert_localization_operation(generation)
        self._initial_pose_pub.publish(msg)
        wait_seconds = float(pose.get("wait_seconds", 8.0))
        required_normal_samples = int(pose.get("required_normal_samples", 0))
        require_absolute = bool(pose.get("require_absolute", False))
        if require_absolute and required_normal_samples > 0:
            latest = self._wait_for_fresh_normal_samples(
                after_sequence=sample_sequence,
                required_samples=required_normal_samples,
                timeout_seconds=wait_seconds,
                generation=generation,
            )
            accepted = bool(latest is not None and self._fast_lio_handoff_ready())
        elif require_absolute:
            deadline = time.monotonic() + wait_seconds
            latest = self.telemetry.latest_pose()
            while time.monotonic() < deadline:
                self._assert_localization_operation(generation)
                latest = self.telemetry.latest_pose()
                if (
                    latest
                    and latest.localization_status == "normal"
                    and self._fast_lio_handoff_ready()
                ):
                    break
                time.sleep(0.2)
            accepted = bool(
                latest
                and latest.localization_status == "normal"
                and self._fast_lio_handoff_ready()
            )
        elif required_normal_samples > 0:
            latest = self._wait_for_fresh_normal_samples(
                after_sequence=sample_sequence,
                required_samples=required_normal_samples,
                timeout_seconds=wait_seconds,
                generation=generation,
            )
            accepted = latest is not None
        else:
            deadline = time.monotonic() + wait_seconds
            latest = self.telemetry.latest_pose()
            while time.monotonic() < deadline:
                self._assert_localization_operation(generation)
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
            best_candidate = self._best_ndt_candidate(scan_match_sequence, {
                "x": x, "y": y, "z": z, "yaw": yaw,
            })
            handoff_decision = self._localization_decision()
            handoff_diagnostics = self._lio_handoff_diagnostics(
                handoff_decision, accepted=False
            )
            diagnostic = ""
            if best_candidate:
                score = best_candidate.get("matching_error")
                inlier = best_candidate.get("inlier_fraction")
                reason = best_candidate.get("reject_reason") or "quality_gate"
                score_text = f"{float(score):.3f}" if score is not None else "unavailable"
                inlier_text = f"{float(inlier):.3f}" if inlier is not None else "unavailable"
                diagnostic = (
                    f", ndt_score={score_text}, inlier_fraction={inlier_text}, "
                    f"reject_reason={reason}"
                )
            raise ProtocolError(
                "INITIAL_POSE_NOT_ACCEPTED",
                f"localization_status={status}, wait_seconds={wait_seconds:.1f}{diagnostic}",
                details={
                    "submitted_pose": {"x": x, "y": y, "z": z, "yaw": yaw},
                    "best_ndt_candidate": best_candidate,
                    "localization_status": status,
                    "ndt_observation_available": best_candidate is not None,
                    "reject_reason": (
                        best_candidate.get("reject_reason")
                        if best_candidate else "ndt_sample_unavailable"
                    ),
                    "handoff_diagnostics": handoff_diagnostics,
                    "localization_decision": handoff_decision,
                },
            )
        return {
            "frame_id": frame_id,
            "x": x,
            "y": y,
            "yaw": yaw,
            "topic": "/initialpose",
            "localization_status": latest.localization_status,
            "best_ndt_candidate": self._best_ndt_candidate(scan_match_sequence, {
                "x": x, "y": y, "z": z, "yaw": yaw,
            }),
            "localized_pose": {
                "x": latest.x,
                "y": latest.y,
                "yaw": latest.yaw,
                "source_status": latest.source_status,
            },
        }

    def set_initial_pose_from_rtk(self, wait_seconds: float = 30.0) -> dict:
        """Ask localization to convert a fresh fixed RTK pose into map coordinates."""
        generation = self._start_localization_operation("operator_rtk_initial_pose")
        return self._set_initial_pose_from_rtk_once(wait_seconds, generation)

    def _rtk_verification_observation(self, decision: dict) -> dict:
        """Capture the concrete RTK values used by the fixed-solution gate."""
        diagnostics = {}
        diagnostics_getter = getattr(getattr(self, "telemetry", None), "localization_diagnostics", None)
        if callable(diagnostics_getter):
            try:
                diagnostics = diagnostics_getter() or {}
            except Exception:
                LOGGER.exception("unable to capture RTK diagnostics for localization attempt")
        if not isinstance(diagnostics, dict):
            diagnostics = {}
        raw_rtk = diagnostics.get("raw_rtk") if isinstance(
            diagnostics.get("raw_rtk"), dict
        ) else {}
        time_diagnostics = diagnostics.get("time_diagnostics") if isinstance(
            diagnostics.get("time_diagnostics"), dict
        ) else {}
        rtk_time = time_diagnostics.get("rtk") if isinstance(
            time_diagnostics.get("rtk"), dict
        ) else {}
        heading = raw_rtk.get("heading") if isinstance(raw_rtk.get("heading"), dict) else {}
        drift = decision.get("rtk_drift") if isinstance(
            decision.get("rtk_drift"), dict
        ) else {}

        def finite(value):
            return self._finite_or_none(value)

        try:
            sample_stamp_ns = int(drift.get("sample_stamp_ns") or 0)
        except (TypeError, ValueError):
            sample_stamp_ns = 0
        return {
            "sample_stamp_ns": sample_stamp_ns,
            "quality": str(decision.get("rtk_quality") or raw_rtk.get("quality") or "unknown"),
            "usable": decision.get("rtk_usable") is True,
            "heading_usable": decision.get("rtk_heading_usable") is True,
            "good_for_navigation": decision.get("rtk_good_for_navigation") is True,
            "blocked_reason": str(decision.get("rtk_blocked_reason") or ""),
            "map_x": finite(decision.get("rtk_x")),
            "map_y": finite(decision.get("rtk_y")),
            "map_yaw": finite(decision.get("rtk_yaw")),
            "latitude": finite(raw_rtk.get("latitude")),
            "longitude": finite(raw_rtk.get("longitude")),
            "altitude": finite(raw_rtk.get("altitude")),
            "position_age_s": finite(
                rtk_time.get("sample_age_seconds", raw_rtk.get("sample_age_seconds"))
            ),
            "horizontal_std_m": finite(raw_rtk.get("horizontal_std_m")),
            "fix_status": raw_rtk.get("fix_status"),
            "solution_status": raw_rtk.get("solution_status"),
            "position_type": raw_rtk.get("position_type"),
            "solution_satellites": raw_rtk.get("solution_satellites"),
            "heading_status": heading.get("status"),
            "heading_type": heading.get("type"),
            "heading_deg": finite(heading.get("heading_deg")),
            "heading_std_deg": finite(heading.get("heading_std_deg")),
            "heading_baseline_m": finite(heading.get("baseline_m")),
            "heading_age_s": finite(
                decision.get("rtk_heading_age_s", heading.get("sample_age_seconds"))
            ),
        }

    def _new_rtk_verification(
        self,
        decision: dict,
        *,
        started_at: str | None = None,
    ) -> dict:
        required = max(1, int(getattr(
            self.safety_config, "localization_rtk_required_samples", 3
        )))
        max_span = float(getattr(
            self.safety_config, "localization_rtk_max_drift_m", 0.30
        ))
        return {
            "started_at": started_at or now_iso(),
            "updated_at": now_iso(),
            "status": "verifying",
            "verified": False,
            "conclusion_code": "awaiting_fresh_rtk_samples",
            "conclusion": "waiting for fresh fixed RTK position-and-heading samples",
            "source": "rtk_self_stability",
            "sample_count": 0,
            "stable_frames": 0,
            "required_stable_frames": required,
            "span_m": None,
            "threshold_xy_m": max_span,
            "last_sample": self._rtk_verification_observation(decision),
            "sample_history": [],
        }

    def _report_rtk_verification(
        self,
        verification: dict,
        *,
        state: str = "running",
        stage_status: str = "verifying",
        error_code: str = "",
        error_message: str = "",
    ) -> None:
        updated_at = now_iso()
        verification.setdefault("started_at", updated_at)
        verification["updated_at"] = updated_at
        if stage_status in {"accepted", "rejected", "failed"}:
            verification.setdefault("finished_at", updated_at)
        stage = {
            "stage": "rtk_fixed",
            "status": stage_status,
            "started_at": verification["started_at"],
            "updated_at": updated_at,
            "rtk_verification": verification,
        }
        if verification.get("finished_at"):
            stage["finished_at"] = verification["finished_at"]
        if error_code:
            stage["error_code"] = error_code
        if error_message:
            stage["error_message"] = error_message
        self._report_localization_attempts({
            "state": state,
            "mode": "rtk_fixed_initialization",
            "selected_stage": "rtk_fixed",
            "strategy": ["rtk_fixed"],
            "stages": [stage],
            "attempts": [],
            "candidate_count": 0,
            "evaluated_candidate_count": 0,
            "motion_commanded": False,
            "rtk_verification": verification,
            "rtk_stability": verification,
        })

    def _wait_for_verified_fixed_rtk(
        self,
        *,
        timeout_seconds: float,
        generation: int,
        on_update: Callable[[dict], None] | None = None,
        started_at: str | None = None,
    ) -> dict:
        """Verify RTK from consecutive RTK samples, never from LIO agreement."""
        deadline = time.monotonic() + max(0.0, float(timeout_seconds))
        required = max(1, int(getattr(
            self.safety_config, "localization_rtk_required_samples", 3
        )))
        max_span = float(getattr(
            self.safety_config, "localization_rtk_max_drift_m", 0.30
        ))
        samples: list[tuple[float, float]] = []
        last_stamp = 0
        sample_count = 0
        sample_history = []
        last_decision = self._localization_decision()
        latest_result = self._new_rtk_verification(
            last_decision,
            started_at=started_at,
        )
        while time.monotonic() < deadline:
            self._assert_localization_operation(generation)
            decision = self._localization_decision()
            last_decision = decision
            drift = decision.get("rtk_drift") if isinstance(
                decision.get("rtk_drift"), dict
            ) else {}
            try:
                stamp_ns = int(drift.get("sample_stamp_ns") or 0)
            except (TypeError, ValueError):
                stamp_ns = 0
            x_value = self._finite_or_none(decision.get("rtk_x"))
            y_value = self._finite_or_none(decision.get("rtk_y"))
            yaw_value = self._finite_or_none(decision.get("rtk_yaw"))
            x = x_value if x_value is not None else float("nan")
            y = y_value if y_value is not None else float("nan")
            yaw = yaw_value if yaw_value is not None else float("nan")
            if stamp_ns > last_stamp:
                last_stamp = stamp_ns
                sample_count += 1
                quality = str(decision.get("rtk_quality") or "").lower()
                checks = {
                    "position_usable": {
                        "actual": decision.get("rtk_usable"),
                        "expected": True,
                        "passed": decision.get("rtk_usable") is True,
                    },
                    "fixed_quality": {
                        "actual": quality or "unknown",
                        "expected": "fixed",
                        "passed": quality == "fixed",
                    },
                    "heading_usable": {
                        "actual": decision.get("rtk_heading_usable"),
                        "expected": True,
                        "passed": decision.get("rtk_heading_usable") is True,
                    },
                    "map_position_finite": {
                        "actual": [x, y] if math.isfinite(x) and math.isfinite(y) else None,
                        "expected": "finite map x/y",
                        "passed": math.isfinite(x) and math.isfinite(y),
                    },
                    "map_heading_finite": {
                        "actual": yaw if math.isfinite(yaw) else None,
                        "expected": "finite map yaw",
                        "passed": math.isfinite(yaw),
                    },
                }
                reject_reasons = [
                    code for code, check in checks.items() if check["passed"] is not True
                ]
                accepted = not reject_reasons
                if not accepted:
                    samples.clear()
                else:
                    samples.append((x, y))
                    samples = samples[-required:]
                span = max(
                    (math.hypot(ax - bx, ay - by)
                     for index, (ax, ay) in enumerate(samples)
                     for bx, by in samples[index + 1:]),
                    default=0.0,
                )
                verified = len(samples) >= required and span <= max_span
                if accepted and len(samples) < required:
                    conclusion_code = "fixed_rtk_samples_pending"
                    conclusion = f"fixed RTK sample streak {len(samples)}/{required}"
                elif accepted and span > max_span:
                    conclusion_code = "rtk_self_span_above_threshold"
                    conclusion = (
                        f"RTK position span {span:.3f} m exceeds {max_span:.3f} m"
                    )
                    reject_reasons.append("rtk_self_span_above_threshold")
                elif verified:
                    conclusion_code = "fixed_rtk_verified"
                    conclusion = (
                        f"fixed RTK verified with {required} fresh samples; "
                        f"position span {span:.3f} m <= {max_span:.3f} m"
                    )
                else:
                    conclusion_code = reject_reasons[0]
                    conclusion = f"RTK sample rejected by {', '.join(reject_reasons)}"
                observation = {
                    **self._rtk_verification_observation(decision),
                    "accepted": accepted and span <= max_span,
                    "reject_reasons": reject_reasons,
                }
                sample_history.append(observation)
                sample_history = sample_history[-12:]
                latest_result = {
                    "started_at": latest_result.get("started_at") or started_at or now_iso(),
                    "updated_at": now_iso(),
                    "sample_stamp_ns": stamp_ns,
                    "source": "rtk_self_stability",
                    "span_m": span,
                    "threshold_xy_m": max_span,
                    "stable_frames": len(samples),
                    "required_stable_frames": required,
                    "verified": verified,
                    "status": "accepted" if verified else "verifying",
                    "conclusion_code": conclusion_code,
                    "conclusion": conclusion,
                    "sample_count": sample_count,
                    "checks": checks,
                    "last_sample": observation,
                    "sample_history": list(sample_history),
                }
                terminal_rejection = (
                    not accepted
                    or (len(samples) >= required and span > max_span)
                )
                if terminal_rejection:
                    rejected_at = now_iso()
                    latest_result.update({
                        "status": "rejected",
                        "verified": False,
                        "updated_at": rejected_at,
                        "finished_at": rejected_at,
                    })
                if callable(on_update):
                    on_update(latest_result)
                if verified:
                    return latest_result
                if terminal_rejection:
                    return latest_result
            time.sleep(0.1)
        latest_result = {
            **latest_result,
            "updated_at": now_iso(),
            "finished_at": now_iso(),
            "status": "rejected",
            "verified": False,
            "timed_out": True,
        }
        if sample_count == 0:
            latest_result.update({
                "conclusion_code": "no_fresh_rtk_samples",
                "conclusion": "no fresh RTK sample arrived before the verification timeout",
                "last_sample": self._rtk_verification_observation(last_decision),
            })
        elif latest_result.get("conclusion_code") == "fixed_rtk_samples_pending":
            latest_result["conclusion"] = (
                f"only {latest_result.get('stable_frames', 0)}/{required} consecutive "
                "fixed RTK samples arrived before timeout"
            )
        if callable(on_update):
            on_update(latest_result)
        return latest_result

    def _rtk_pose_from_verification(self, verification: dict) -> dict | None:
        sample = verification.get("last_sample") if isinstance(verification, dict) else None
        sample = sample if isinstance(sample, dict) else {}
        x = self._finite_or_none(sample.get("map_x"))
        y = self._finite_or_none(sample.get("map_y"))
        yaw = self._finite_or_none(sample.get("map_yaw"))
        if x is None or y is None or yaw is None:
            return None
        return {
            "x": x,
            "y": y,
            "z": 0.0,
            "yaw": yaw,
            "candidate_label": "RTK固定解定位点",
        }

    def _rtk_still_fixed_for_commit(self) -> bool:
        decision = self._localization_decision()
        return bool(
            decision.get("rtk_usable") is True
            and str(decision.get("rtk_quality") or "").lower() == "fixed"
            and decision.get("rtk_heading_usable") is True
            and self._finite_or_none(decision.get("rtk_x")) is not None
            and self._finite_or_none(decision.get("rtk_y")) is not None
            and self._finite_or_none(decision.get("rtk_yaw")) is not None
        )

    def _lio_handoff_diagnostics(self, decision: dict, *, accepted: bool) -> dict:
        nested = decision.get("localization") if isinstance(
            decision.get("localization"), dict
        ) else {}
        sample_age = self._finite_or_none(
            decision.get("sample_age_seconds", nested.get("sample_age_seconds"))
        )
        provider = getattr(self, "_ros_executor_alive_provider", None)
        try:
            executor_alive = bool(provider()) if callable(provider) else None
        except Exception:
            executor_alive = False
        reason = ""
        if not accepted:
            if executor_alive is False:
                reason = "ros_executor_not_alive"
            elif decision.get("active_source") != "lio_imu":
                reason = "active_source_not_lio_imu"
            elif decision.get("lio_healthy") is not True:
                reason = "lio_not_healthy"
            elif decision.get("lio_anchored") is not True:
                reason = "lio_anchor_not_ready"
            elif decision.get("absolute_stable") is not True:
                reason = "absolute_pose_not_stable"
            elif decision.get("handoff_state") != "ready":
                reason = "handoff_state_not_ready"
            else:
                reason = "fresh_lio_frame_not_confirmed"
        return {
            "ros_executor_alive": executor_alive,
            "localization_frame_age_seconds": sample_age,
            "anchor_generation": decision.get("handoff_anchor_generation"),
            "active_source": decision.get("active_source"),
            "handoff_state": decision.get("handoff_state"),
            "lio_healthy": decision.get("lio_healthy"),
            "lio_anchored": decision.get("lio_anchored"),
            "absolute_stable": decision.get("absolute_stable"),
            "handoff_failure_reason": reason,
        }

    def _wait_for_lio_handoff(
        self,
        *,
        after_generation: int,
        timeout_seconds: float,
        generation: int,
    ):
        """Wait for the localization node to accept a post-seed FAST-LIO frame."""
        deadline = time.monotonic() + max(0.0, float(timeout_seconds))
        last_decision = {}
        while time.monotonic() < deadline:
            self._assert_localization_operation(generation)
            decision = self._localization_decision()
            last_decision = decision
            try:
                handoff_generation = int(decision.get("handoff_anchor_generation") or 0)
            except (TypeError, ValueError):
                handoff_generation = 0
            ready = (
                handoff_generation > after_generation
                and decision.get("handoff_state") == "ready"
                and decision.get("active_source") == "lio_imu"
                and decision.get("lio_healthy") is True
                and decision.get("lio_anchored") is True
                and decision.get("absolute_stable") is True
            )
            latest = self.telemetry.latest_pose()
            if ready and latest and self._localization_status_is_normal(
                getattr(latest, "localization_status", None)
            ):
                return latest, decision
            time.sleep(0.1)
        return None, last_decision

    @staticmethod
    def _rtk_heading_conflict_with_lidar(rtk_seed: dict | None, lidar_pose) -> dict | None:
        """Return conflict details when lidar heading flips relative to RTK.

        Only vetoes when lidar is currently normal and already at the RTK XY.
        Lost lidar, or a large XY disagreement, is not treated as a heading
        fault so a cold start can still use a verified fixed RTK seed.
        """
        if not rtk_seed or lidar_pose is None:
            return None
        if not RosAdapter._localization_status_is_normal(
            getattr(lidar_pose, "localization_status", None)
        ):
            return None
        lidar_x = RosAdapter._finite_or_none(getattr(lidar_pose, "x", None))
        lidar_y = RosAdapter._finite_or_none(getattr(lidar_pose, "y", None))
        lidar_yaw = RosAdapter._finite_or_none(getattr(lidar_pose, "yaw", None))
        rtk_x = RosAdapter._finite_or_none(rtk_seed.get("x"))
        rtk_y = RosAdapter._finite_or_none(rtk_seed.get("y"))
        rtk_yaw = RosAdapter._finite_or_none(rtk_seed.get("yaw"))
        if None in (lidar_x, lidar_y, lidar_yaw, rtk_x, rtk_y, rtk_yaw):
            return None
        xy_delta_m = math.hypot(rtk_x - lidar_x, rtk_y - lidar_y)
        if xy_delta_m > RTK_LIDAR_HEADING_CONFLICT_XY_M:
            return None
        yaw_delta_deg = math.degrees(
            math.atan2(
                math.sin(rtk_yaw - lidar_yaw),
                math.cos(rtk_yaw - lidar_yaw),
            )
        )
        if abs(yaw_delta_deg) <= RTK_LIDAR_HEADING_CONFLICT_DEG:
            return None
        return {
            "lidar_x": lidar_x,
            "lidar_y": lidar_y,
            "lidar_yaw": lidar_yaw,
            "rtk_x": rtk_x,
            "rtk_y": rtk_y,
            "rtk_yaw": rtk_yaw,
            "xy_delta_m": xy_delta_m,
            "yaw_delta_deg": yaw_delta_deg,
        }

    @staticmethod
    def _localization_status_is_normal(status) -> bool:
        """Accept both ROS numeric status=3 and telemetry's readable value.

        ROS callbacks carry the numeric localization status, while the
        telemetry snapshot deliberately exposes ``normal`` / ``lost`` for
        operators. RTK handoff can be evaluated from either representation;
        parsing the latter with ``int()`` used to turn a successful handoff
        into ``INITIALIZATION_FAILED``.
        """
        if isinstance(status, str):
            normalized = status.strip().lower()
            if normalized == "normal":
                return True
            try:
                return int(normalized) == 3
            except ValueError:
                return False
        try:
            return int(status) == 3
        except (TypeError, ValueError):
            return False

    def _set_initial_pose_from_rtk_once(
        self,
        wait_seconds: float,
        generation: int,
    ) -> dict:
        self._assert_localization_operation(generation)
        previous_decision = self._localization_decision()
        verification = self._new_rtk_verification(previous_decision)
        self._report_rtk_verification(verification)
        if not self._rtk_initial_pose_client.wait_for_service(timeout_sec=3.0):
            self._report_rtk_verification(
                verification,
                state="failed",
                stage_status="failed",
                error_code="RTK_INITIAL_POSE_UNAVAILABLE",
                error_message="/localization/seed_from_rtk service is unavailable",
            )
            raise ProtocolError(
                "RTK_INITIAL_POSE_UNAVAILABLE",
                "/localization/seed_from_rtk service is unavailable",
                details={"rtk_verification": verification},
            )
        deadline = time.monotonic() + max(1.0, float(wait_seconds))
        last_progress_at = 0.0

        def report_progress(current: dict) -> None:
            nonlocal last_progress_at
            now_monotonic = time.monotonic()
            terminal = current.get("status") in {"accepted", "rejected"}
            if terminal or not last_progress_at or now_monotonic - last_progress_at >= 0.5:
                self._report_rtk_verification(current)
                last_progress_at = now_monotonic

        stability = self._wait_for_verified_fixed_rtk(
            timeout_seconds=max(0.0, deadline - time.monotonic()),
            generation=generation,
            on_update=report_progress,
            started_at=verification.get("started_at"),
        )
        if not stability or not stability.get("verified"):
            verification = stability or verification
            self._report_rtk_verification(
                verification,
                state="failed",
                stage_status="rejected",
                error_code="RTK_FIXED_NOT_STABLE",
                error_message=verification.get("conclusion") or "fixed RTK verification failed",
            )
            raise ProtocolError(
                "RTK_FIXED_NOT_STABLE",
                "fixed RTK did not provide 3 fresh position-and-heading samples within the 0.30 m self-stability gate",
                details={
                    "rtk_stability": verification,
                    "rtk_verification": verification,
                },
            )
        verification = stability

        # RTK has provided the map pose. A healthy live lidar heading at the
        # same XY can veto a flipped RTK yaw before that seed is published.
        # Weak or missing lidar stays advisory: the NDT cross-check below is
        # still evidence only, so outdoor RTK can initialize without a match.
        rtk_seed = self._rtk_pose_from_verification(verification)
        if rtk_seed is None:
            self._report_rtk_verification(
                verification,
                state="failed",
                stage_status="failed",
                error_code="RTK_POSE_UNAVAILABLE",
                error_message="verified RTK did not provide finite map x/y/yaw",
            )
            raise ProtocolError(
                "RTK_POSE_UNAVAILABLE",
                "verified RTK did not provide finite map x/y/yaw",
                details={
                    "rtk_stability": verification,
                    "rtk_verification": verification,
                },
            )

        lidar_pose = None
        latest_getter = getattr(self.telemetry, "latest_pose", None) if self.telemetry else None
        if callable(latest_getter):
            lidar_pose = latest_getter()
        heading_conflict = self._rtk_heading_conflict_with_lidar(rtk_seed, lidar_pose)
        if heading_conflict:
            conclusion = (
                "live lidar heading disagrees with fixed RTK by "
                f"{abs(heading_conflict['yaw_delta_deg']):.1f}deg at the same place; "
                "not committing the RTK heading"
            )
            self._report_rtk_verification(
                verification,
                state="failed",
                stage_status="rejected",
                error_code="RTK_HEADING_CONFLICTS_WITH_LIDAR",
                error_message=conclusion,
            )
            raise ProtocolError(
                "RTK_HEADING_CONFLICTS_WITH_LIDAR",
                conclusion,
                details={
                    "rtk_stability": verification,
                    "rtk_verification": verification,
                    "heading_conflict": heading_conflict,
                },
            )

        attempt_started = self._begin_localization_attempt(
            1,
            rtk_seed,
            extra={"stage": "rtk_fixed", "source": "rtk_ndt_crosscheck"},
        )
        attempts = [attempt_started]
        stage = {
            "stage": "rtk_fixed",
            "status": "verifying",
            "started_at": verification.get("started_at") or now_iso(),
            "updated_at": now_iso(),
            "rtk_verification": verification,
            "attempts": attempts,
        }
        transaction = {
            "state": "running",
            "mode": "rtk_fixed_initialization",
            "selected_stage": "rtk_fixed",
            "strategy": ["rtk_fixed"],
            "stages": [stage],
            "attempts": attempts,
            "candidate_count": 1,
            "evaluated_candidate_count": 0,
            "active_candidate_number": 1,
            "active_candidate_stage": "rtk_fixed",
            "motion_commanded": False,
            "rtk_verification": verification,
            "rtk_stability": verification,
        }
        self._report_localization_attempts(transaction)

        handoff_settle = float(getattr(
            self.safety_config, "localization_handoff_settle_seconds", 8.0
        ))
        probe_wait = min(
            5.0,
            max(1.0, deadline - time.monotonic() - handoff_settle),
        )
        trusted_was_frozen = bool(getattr(self, "_trusted_pose_frozen", False))
        self._trusted_pose_frozen = True
        try:
            attempt = self._probe_localization_seed(
                rtk_seed,
                generation,
                wait_seconds=probe_wait,
                index=1,
                extra={"stage": "rtk_fixed", "source": "rtk_ndt_crosscheck"},
                require_ndt_observation=True,
            )
        finally:
            self._trusted_pose_frozen = trusted_was_frozen
        attempt = self._finish_localization_attempt(attempt, attempt_started)
        attempts[0] = attempt
        ndt_candidate = dict(attempt.get("ndt_candidate") or {})
        ndt_summary = {
            "matching_error": attempt.get("matching_error"),
            "inlier_fraction": attempt.get("inlier_fraction"),
            "has_converged": attempt.get("has_converged"),
            "stable_frames": attempt.get("stable_frames"),
            "required_stable_frames": attempt.get("required_stable_frames"),
            "reject_reason": attempt.get("reject_reason"),
        }
        stage.update({"updated_at": now_iso(), "attempts": attempts})
        transaction.update({
            "attempts": attempts,
            "evaluated_candidate_count": 1,
            "active_candidate_number": None,
            "active_candidate_stage": None,
            "best_candidate_index": 1,
            "best_candidate_stage": "rtk_fixed",
            "best_candidate_seed_pose": self._pose_payload(rtk_seed),
            "best_candidate_label": rtk_seed["candidate_label"],
            "best_candidate_ndt": ndt_summary,
            "best_ndt_candidate": ndt_candidate or None,
            "best_match_pose": attempt.get("matched_pose") or self._pose_payload(rtk_seed),
            "ndt_crosscheck_passed": bool(attempt.get("eligible")),
        })
        self._report_localization_attempts(transaction)

        if not self._rtk_still_fixed_for_commit():
            failed_at = now_iso()
            verification.update({
                "status": "rejected",
                "verified": False,
                "finished_at": failed_at,
                "updated_at": failed_at,
                "conclusion_code": "rtk_lost_before_commit",
                "conclusion": "RTK fixed quality was lost before the verified result could be committed",
            })
            stage.update({
                "status": "rejected",
                "updated_at": failed_at,
                "finished_at": failed_at,
                "error_code": "RTK_FIXED_NOT_STABLE",
                "error_message": verification["conclusion"],
                "rtk_verification": verification,
            })
            transaction.update({
                "state": "failed",
                "rtk_verification": verification,
                "rtk_stability": verification,
            })
            self._report_localization_attempts(transaction)
            raise ProtocolError(
                "RTK_FIXED_NOT_STABLE",
                verification["conclusion"],
                details={
                    "rtk_stability": verification,
                    "rtk_verification": verification,
                    "localization_attempts": copy.deepcopy(transaction),
                },
            )

        # Capture the generation after the transient NDT probe. Only a newer
        # generation can prove that /seed_from_rtk performed the final commit.
        pre_commit_decision = self._localization_decision()
        try:
            previous_handoff_generation = int(
                pre_commit_decision.get("handoff_anchor_generation") or 0
            )
        except (TypeError, ValueError):
            previous_handoff_generation = 0
        future = self._rtk_initial_pose_client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=5.0) or not future.done():
            failed_at = now_iso()
            stage.update({
                "status": "failed",
                "updated_at": failed_at,
                "finished_at": failed_at,
                "error_code": "RTK_INITIAL_POSE_TIMEOUT",
                "error_message": "RTK initial pose service timed out after RTK verification passed",
            })
            transaction["state"] = "failed"
            self._report_localization_attempts(transaction)
            raise ProtocolError(
                "RTK_INITIAL_POSE_TIMEOUT",
                "RTK initial pose service timed out",
                details={
                    "rtk_verification": verification,
                    "localization_attempts": copy.deepcopy(transaction),
                },
            )
        response = future.result()
        self._assert_localization_operation(generation)
        if response is None or not response.success:
            error_message = (
                response.message if response else "RTK initial pose service returned no response"
            )
            failed_at = now_iso()
            stage.update({
                "status": "failed",
                "updated_at": failed_at,
                "finished_at": failed_at,
                "error_code": "RTK_POSE_UNAVAILABLE",
                "error_message": error_message,
            })
            transaction["state"] = "failed"
            self._report_localization_attempts(transaction)
            raise ProtocolError(
                "RTK_POSE_UNAVAILABLE",
                error_message,
                details={
                    "rtk_verification": verification,
                    "localization_attempts": copy.deepcopy(transaction),
                },
            )
        handoff_timeout = min(
            max(0.0, deadline - time.monotonic()),
            float(getattr(self.safety_config, "localization_handoff_settle_seconds", 8.0)),
        )
        latest, handoff_decision = self._wait_for_lio_handoff(
            after_generation=previous_handoff_generation,
            timeout_seconds=handoff_timeout,
            generation=generation,
        )
        if latest is None:
            handoff = {
                "status": "failed",
                "conclusion_code": "lio_handoff_timeout",
                "conclusion": "RTK seed passed, but no fresh stable FAST-LIO handoff was verified",
                **self._lio_handoff_diagnostics(handoff_decision, accepted=False),
            }
            failed_at = now_iso()
            verification = {
                **verification,
                "handoff": handoff,
                "status": "failed",
                "updated_at": failed_at,
                "finished_at": failed_at,
            }
            stage.update({
                "status": "failed",
                "updated_at": failed_at,
                "finished_at": failed_at,
                "error_code": "LIO_HANDOFF_TIMEOUT",
                "error_message": handoff["conclusion"],
                "rtk_verification": verification,
            })
            transaction.update({
                "state": "failed",
                "handoff_pending": True,
                "rtk_verification": verification,
                "rtk_stability": verification,
                "handoff_diagnostics": handoff,
            })
            self._report_localization_attempts(transaction)
            raise ProtocolError(
                "LIO_HANDOFF_TIMEOUT",
                "fixed RTK pose was accepted but FAST-LIO did not become the fresh continuous source",
                details={
                    "rtk_stability": verification,
                    "rtk_verification": verification,
                    "localization_decision": handoff_decision,
                    "handoff_diagnostics": handoff,
                    "localization_attempts": copy.deepcopy(transaction),
                },
            )
        accepted_at = now_iso()
        verification = {
            **verification,
            "status": "accepted",
            "updated_at": accepted_at,
            "finished_at": accepted_at,
            "handoff": {
                "status": "accepted",
                "conclusion_code": "lio_imu_handoff_verified",
                "conclusion": "RTK absolute seed accepted; fresh FAST-LIO + IMU is the continuous pose source",
                **self._lio_handoff_diagnostics(handoff_decision, accepted=True),
            },
        }
        localized_pose = {
            "x": latest.x,
            "y": latest.y,
            "z": latest.z,
            "yaw": latest.yaw,
        }
        attempt["rtk_fixed_committed"] = True
        stage.update({
            "status": "accepted",
            "updated_at": accepted_at,
            "finished_at": accepted_at,
            "rtk_verification": verification,
            "attempts": attempts,
        })
        best_match_pose = attempt.get("matched_pose") or self._pose_payload(rtk_seed)
        transaction.update({
            "state": "accepted",
            "stop_reason": "rtk_fixed_ndt_crosscheck_then_lio_handoff",
            "early_stopped": True,
            "rtk_fixed_committed": True,
            "localized_pose": localized_pose,
            "best_match_pose": best_match_pose,
            "best_ndt_candidate": ndt_candidate or None,
            "rtk_stability": stability,
            "rtk_verification": verification,
            "handoff_diagnostics": verification["handoff"],
        })
        result = {
            "source": "rtk_fixed",
            "service": "/localization/seed_from_rtk",
            "message": response.message,
            "localization_status": latest.localization_status,
            "localized_pose": localized_pose,
            "best_match_pose": best_match_pose,
            "best_ndt_candidate": ndt_candidate or None,
            "best_candidate_index": 1,
            "best_candidate_stage": "rtk_fixed",
            "best_candidate_seed_pose": self._pose_payload(rtk_seed),
            "best_candidate_label": rtk_seed["candidate_label"],
            "best_candidate_ndt": ndt_summary,
            "ndt_crosscheck_passed": bool(attempt.get("eligible")),
            "rtk_fixed_committed": True,
            "rtk_stability": stability,
            "rtk_verification": verification,
            "handoff_diagnostics": verification["handoff"],
            "localization_attempts": transaction,
        }
        self._report_localization_attempts(transaction)
        return result

    def global_relocalize(self, wait_seconds: float = 90.0, *, automatic: bool = False) -> dict:
        """Run map-wide position and 360-degree yaw search without a guessed pose."""
        generation = (
            self._start_localization_operation("global_relocalize", automatic=True)
            if automatic
            else self._start_localization_operation("global_relocalize")
        )
        return self._global_relocalize_once(wait_seconds, generation)

    def _global_relocalize_once(self, wait_seconds: float, generation: int) -> dict:
        """Run the keyframe/global matcher inside an existing localization operation."""
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
        self._assert_localization_operation(generation)
        if response is None or not response.success:
            raise ProtocolError(
                "GLOBAL_RELOCALIZATION_UNAVAILABLE",
                response.message if response else "global relocalization returned no response",
            )
        latest = self._wait_for_fresh_normal_samples(
            after_sequence=sample_sequence,
            required_samples=3,
            timeout_seconds=wait_seconds,
            generation=generation,
        )
        if latest is None:
            raise ProtocolError(
                "GLOBAL_RELOCALIZATION_NOT_VERIFIED",
                f"{response.message}; no verified 3-frame localization within {wait_seconds:.1f}s",
            )
        return {
            "mode": "global_position_yaw_search",
            "source": "scan_context_fastgicp_icp",
            "stage": "keyframe_global_match",
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

    def quick_then_global_relocalize(
        self,
        *,
        origin: dict | None,
        manual_seed: dict | None = None,
        scene_scope: str = "indoor",
        coordinate_mode: str = "local_only",
        wait_seconds: float = 120.0,
        automatic: bool = False,
    ) -> dict:
        """Try cheap stationary seeds first and arm Scan Context only if none work."""
        generation = self._start_localization_operation(
            "quick_then_global", automatic=automatic
        )
        overall_deadline = time.monotonic() + max(35.0, float(wait_seconds))
        quick_budget = max(5.0, float(getattr(
            self.safety_config, "localization_quick_search_seconds", 30.0
        )))
        quick_deadline = min(overall_deadline, time.monotonic() + quick_budget)
        stages = []
        rtk_verification = None

        seeds = []
        if manual_seed is not None:
            for candidate in self._relocalization_candidates(
                float(manual_seed["x"]),
                float(manual_seed["y"]),
                float(manual_seed.get("z", 0.0)),
                float(manual_seed.get("yaw", 0.0)),
            )[:12]:
                seeds.append(("operator_seed", candidate))
        else:
            if origin and all(origin.get(field) is not None for field in ("x", "y", "yaw")):
                for candidate in self._relocalization_candidates(
                    float(origin["x"]),
                    float(origin["y"]),
                    float(origin.get("z", 0.0)),
                    float(origin.get("yaw", 0.0)),
                )[:12]:
                    seeds.append(("mapping_origin", candidate))
            trusted = self.latest_trusted_pose()
            if trusted and all(trusted.get(field) is not None for field in ("x", "y", "yaw")):
                seeds.append(("last_trusted", dict(trusted)))

        deduplicated = []
        seen = set()
        for source, seed in seeds:
            key = (
                round(float(seed["x"]), 3),
                round(float(seed["y"]), 3),
                round(float(seed.get("yaw", 0.0)), 3),
            )
            if key in seen:
                continue
            seen.add(key)
            deduplicated.append((source, seed))
        seeds = deduplicated
        attempts = [
            {**self._waiting_attempt(index, seed), "stage": source, "source": source}
            for index, (source, seed) in enumerate(seeds, start=1)
        ]
        quick_started_at = now_iso()
        quick_stage = {
            "stage": "quick_initialization",
            "status": "searching",
            "started_at": quick_started_at,
            "updated_at": quick_started_at,
        }
        session = {
            "state": "running",
            "mode": "quick_then_global",
            "selected_stage": "quick_initialization",
            "strategy": ["mapping_origin_local", "last_trusted", "keyframe_global_match"],
            "stages": stages + [quick_stage],
            "attempts": attempts,
            "candidate_count": len(attempts),
            "evaluated_candidate_count": 0,
            "early_stopped": False,
            "motion_commanded": False,
        }
        if isinstance(rtk_verification, dict):
            session["rtk_verification"] = rtk_verification
            session["rtk_stability"] = rtk_verification
        self._trusted_pose_frozen = True
        try:
            self._report_localization_attempts(session)
            early_stopped = False
            for index, (source, seed) in enumerate(seeds, start=1):
                self._assert_localization_operation(generation)
                remaining = quick_deadline - time.monotonic()
                if remaining < 1.0:
                    break
                started_attempt = self._begin_localization_attempt(
                    index,
                    seed,
                    extra={"stage": source, "source": source},
                )
                attempts[index - 1] = started_attempt
                session.update({
                    "attempts": attempts,
                    "active_candidate_number": index,
                    "active_candidate_stage": source,
                    "evaluated_candidate_count": index - 1,
                    "live_pose": started_attempt.get("live_pose"),
                })
                # Publish before the blocking NDT/localization wait.  Without
                # this snapshot the UI receives only the final result and
                # keeps the candidate shown as waiting while it is being tried.
                self._report_localization_attempts(session)
                completed_attempt = self._probe_localization_seed(
                    seed,
                    generation,
                    wait_seconds=min(2.0, remaining),
                    index=index,
                    extra={"stage": source, "source": source},
                )
                attempts[index - 1] = self._finish_localization_attempt(
                    completed_attempt, started_attempt
                )
                session.update({
                    "attempts": attempts,
                    "evaluated_candidate_count": index,
                    "active_candidate_number": None,
                    "active_candidate_stage": None,
                    "live_pose": self._current_live_pose(),
                })
                self._report_localization_attempts(session)
                if self._candidate_is_optimal(attempts[index - 1].get("ndt_candidate")):
                    early_stopped = True
                    self._skip_waiting_attempts(attempts, "optimal_threshold_reached")
                    session.update({
                        "early_stopped": True,
                        "stop_reason": "ndt_optimal_score",
                    })
                    break
            if not early_stopped:
                self._skip_waiting_attempts(attempts, "quick_search_budget_exhausted")
            ranked = self._select_ranked_attempt(attempts)
            if ranked is not None:
                quick_finished_at = now_iso()
                quick_stage.update({
                    "status": "accepted",
                    "updated_at": quick_finished_at,
                    "finished_at": quick_finished_at,
                })
                self._mark_out_ranked_attempts(attempts, ranked)
                self._trusted_pose_frozen = False
                candidate = dict(ranked.get("ndt_candidate") or {})
                committed = self._commit_best_relocalization_candidate(
                    generation,
                    {
                        "source": ranked.get("stage") or "quick_initialization",
                        "selected_candidate_index": ranked.get("index"),
                    },
                    candidate,
                    attempts,
                )
                payload = {
                    **committed,
                    "mode": "quick_then_global",
                    "selected_stage": ranked.get("stage") or "quick_initialization",
                    "stages": session["stages"],
                    "attempts": attempts,
                    "candidate_count": len(attempts),
                    "evaluated_candidate_count": sum(
                        1 for item in attempts if item.get("status") != "skipped"
                    ),
                    "early_stopped": early_stopped,
                    "stop_reason": "ndt_optimal_score" if early_stopped else "quick_search_best",
                    "global_search_started": False,
                }
                self._report_localization_attempts({**payload, "state": "accepted"})
                return payload
        finally:
            self._trusted_pose_frozen = False

        quick_finished_at = now_iso()
        quick_stage.update({
            "status": "failed",
            "updated_at": quick_finished_at,
            "finished_at": quick_finished_at,
        })
        session["best_ndt_candidate"] = self._best_diagnostic_candidate(attempts)
        global_started_at = now_iso()
        session["stages"].append({
            "stage": "keyframe_global_match",
            "status": "searching",
            "started_at": global_started_at,
            "updated_at": global_started_at,
        })
        session.update({
            "state": "global_searching",
            "selected_stage": "keyframe_global_match",
            "stop_reason": "quick_search_no_eligible_candidate",
            "global_search_started": True,
            "attempts": attempts,
        })
        self._report_localization_attempts(session)
        try:
            global_result = self._global_relocalize_once(
                max(30.0, overall_deadline - time.monotonic()), generation
            )
        except ProtocolError as exc:
            failed_at = now_iso()
            session["stages"][-1].update({
                "status": "failed",
                "updated_at": failed_at,
                "finished_at": failed_at,
                "error_code": exc.code,
                "error_message": exc.message,
            })
            session.update({
                "state": "failed",
                "evaluated_candidate_count": sum(
                    1 for item in attempts if item.get("status") != "skipped"
                ),
                "live_pose": self._current_live_pose(),
                "global_relocalization_error": {
                    "error_code": exc.code,
                    "error_message": exc.message,
                    "details": dict(exc.details or {}),
                },
            })
            self._report_localization_attempts(session)
            raise ProtocolError(
                "QUICK_THEN_GLOBAL_RELOCALIZATION_FAILED",
                f"quick NDT candidates and global relocalization failed: {exc.message}",
                details=session,
            ) from exc
        global_finished_at = now_iso()
        session["stages"][-1].update({
            "status": "accepted",
            "updated_at": global_finished_at,
            "finished_at": global_finished_at,
        })
        payload = {
            **global_result,
            "mode": "quick_then_global",
            "selected_stage": "keyframe_global_match",
            "stages": session["stages"],
            "attempts": attempts,
            "candidate_count": len(attempts),
            "evaluated_candidate_count": sum(
                1 for item in attempts if item.get("status") != "skipped"
            ),
            "early_stopped": False,
            "stop_reason": "global_match_verified",
            "global_search_started": True,
            "best_match_pose": global_result.get("localized_pose"),
        }
        self._report_localization_attempts({**payload, "state": "accepted"})
        return payload

    def progressive_relocalize(
        self,
        *,
        origin: dict | None,
        waypoints: list[dict],
        wait_seconds: float = 180.0,
    ) -> dict:
        """Always reinitialize through origin, route points, then global matching.

        Each stage evaluates every seed against the quality gate first, then
        commits only the ranked best match.  Scan Context keyframe retrieval,
        FastGICP/ICP geometry verification, and full-map ICP remain the final
        fallback when explicit seeds fail.
        """
        generation = self._start_localization_operation("progressive_operator_initialization")
        deadline = time.monotonic() + max(30.0, float(wait_seconds))
        stages = []
        strategy = ["mapping_origin_bounded", "route_waypoints", "keyframe_global_match"]
        origin_seed = None
        seeds = []
        all_attempts = []
        if origin and all(origin.get(field) is not None for field in ("x", "y", "yaw")):
            origin_seed = dict(origin)
            origin_stage = {
                "stage": "mapping_origin_bounded",
                "status": "searching",
                "started_at": now_iso(),
            }
            origin_stage["updated_at"] = origin_stage["started_at"]
            stages.append(origin_stage)
        else:
            unavailable_at = now_iso()
            stages.append({
                "stage": "mapping_origin_bounded",
                "status": "unavailable",
                "started_at": unavailable_at,
                "updated_at": unavailable_at,
                "finished_at": unavailable_at,
                "error_code": (origin or {}).get("unavailable_error_code", "MAPPING_START_POSE_MISSING"),
                "error_message": (origin or {}).get(
                    "unavailable_error_message", "map package has no usable mapping start pose"
                ),
            })
        for index, raw in enumerate(waypoints):
            if not isinstance(raw, dict) or not all(
                raw.get(field) is not None for field in ("x", "y", "yaw")
            ):
                continue
            seeds.append(("route_waypoint", index, dict(raw)))

        session = {
            "state": "running",
            "mode": "progressive_stationary_search",
            "strategy": strategy,
            "selected_stage": "mapping_origin_bounded" if origin_seed is not None else None,
            "candidate_count": len(seeds) + (20 if origin_seed else 0),
            "evaluated_candidate_count": 0,
            "stages": stages,
            "attempts": all_attempts,
            "best_match_pose": None,
            "best_ndt_candidate": None,
            "live_pose": self._current_live_pose(),
        }
        self._report_localization_attempts(session)

        # The mapping origin receives the complete stationary bounded search:
        # exact pose, eight yaw hypotheses, then 0.3/0.6/1.0 m XY rings.  Keep
        # route waypoints exact to avoid multiplying a large route by twenty,
        # and reserve sixty seconds for map-wide keyframe recovery.
        if origin_seed is not None:
            origin_budget = min(
                120.0,
                max(20.0, deadline - time.monotonic() - 60.0 - 3.0 * len(seeds)),
            )
            try:
                result = self._active_relocalize_once({
                    **origin_seed,
                    "source": "mapping_origin",
                    "stage": "mapping_origin_bounded",
                    "stage_started_at": origin_stage["started_at"],
                    "max_attempts": 20,
                    "wait_seconds": origin_budget,
                    "candidate_wait_seconds": 5.0,
                }, generation, persist_state=False)
                origin_stage.update({
                    "status": "accepted",
                    "updated_at": now_iso(),
                    "finished_at": now_iso(),
                    "attempts": result.get("attempts", []),
                    "best_ndt_candidate": result.get("best_ndt_candidate"),
                    "best_match_pose": result.get("best_match_pose"),
                })
                payload = {
                    **result,
                    "mode": "progressive_stationary_search",
                    "strategy": strategy,
                    "selected_stage": "mapping_origin_bounded",
                    "selected_waypoint_index": None,
                    "stages": stages,
                }
                self._report_localization_attempts({**payload, "state": "accepted"})
                return payload
            except ProtocolError as exc:
                if exc.code == "RELOCALIZATION_SUPERSEDED":
                    raise
                details = dict(exc.details or {})
                origin_attempts = list(details.get("attempts") or [])
                all_attempts.extend(origin_attempts)
                evaluated_count = self._evaluated_localization_attempt_count(origin_attempts)
                skipped_count = sum(
                    1 for item in origin_attempts if item.get("status") == "skipped"
                )
                best_candidate = dict(details.get("best_ndt_candidate") or {})
                winner_index = details.get("best_candidate_index")
                if exc.code == "RELOCALIZATION_HANDOFF_FAILED" and best_candidate.get("eligible"):
                    winner_label = f" #{winner_index}" if winner_index is not None else ""
                    origin_message = (
                        f"建图原点候选{winner_label} 已通过 NDT 质量门限，但提交后的 "
                        f"FAST-LIO 接管失败（{exc.message}），已转入路线航点候选"
                    )
                    origin_status = "failed"
                else:
                    skipped_detail = f"，另有 {skipped_count} 个因搜索预算结束跳过" if skipped_count else ""
                    origin_message = (
                        f"已评估 {evaluated_count} 个建图原点及周边候选{skipped_detail}，"
                        "均未通过 NDT 质量门限，已转入路线航点候选"
                    ) if origin_attempts else exc.message
                    origin_status = "rejected"
                origin_stage.update({
                    "status": origin_status,
                    "updated_at": now_iso(),
                    "finished_at": now_iso(),
                    "error_code": exc.code,
                    "error_message": origin_message,
                    "attempts": origin_attempts,
                    "best_ndt_candidate": best_candidate or None,
                    "best_candidate_index": winner_index,
                    "best_candidate_stage": details.get("best_candidate_stage"),
                    "best_candidate_seed_pose": details.get("best_candidate_seed_pose"),
                    "timed_out": details.get("timed_out", False),
                })
                session.update(
                    attempts=all_attempts,
                    stages=stages,
                    state="running",
                    evaluated_candidate_count=self._evaluated_localization_attempt_count(all_attempts),
                    # Keep a failed handoff visible while the next stage is
                    # searched.  Otherwise the next progress packet erases
                    # exactly which origin candidate was submitted and its
                    # NDT evidence from the operator-facing result.
                    best_ndt_candidate=best_candidate or session.get("best_ndt_candidate"),
                    best_match_pose=details.get("best_match_pose") or session.get("best_match_pose"),
                    best_ndt_committed=bool(details.get("best_ndt_committed", False)),
                    best_candidate_index=winner_index,
                    best_candidate_stage=details.get("best_candidate_stage"),
                    best_candidate_seed_pose=details.get("best_candidate_seed_pose"),
                    best_candidate_label=details.get("best_candidate_label"),
                    best_candidate_ndt=details.get("best_candidate_ndt"),
                    handoff_pending=bool(details.get("handoff_pending", False)),
                )
                self._report_localization_attempts(session)

        local_budget = max(0.0, deadline - time.monotonic() - 60.0)
        per_seed_wait = min(8.0, max(3.0, local_budget / max(1, len(seeds))))
        waypoint_attempts = []
        route_stage = None
        if seeds:
            route_stage = {
                "stage": "route_waypoints",
                "status": "searching",
                "started_at": now_iso(),
            }
            route_stage["updated_at"] = route_stage["started_at"]
            stages.append(route_stage)
        else:
            skipped_at = now_iso()
            stages.append({
                "stage": "route_waypoints",
                "status": "skipped",
                "started_at": skipped_at,
                "updated_at": skipped_at,
                "finished_at": skipped_at,
                "error_code": "NO_ROUTE_WAYPOINTS",
            })
        session.update(
            stages=stages,
            state="running",
            selected_stage="route_waypoints" if route_stage is not None else None,
        )
        self._report_localization_attempts(session)
        self._trusted_pose_frozen = True
        try:
            for stage_name, waypoint_index, seed in seeds:
                self._assert_localization_operation(generation)
                remaining_local = deadline - time.monotonic() - 60.0
                if remaining_local < 1.0:
                    skipped_at = now_iso()
                    skipped = {
                        "index": len(all_attempts) + 1,
                        "stage": stage_name,
                        "waypoint_index": waypoint_index,
                        "status": "failed",
                        "started_at": skipped_at,
                        "updated_at": skipped_at,
                        "finished_at": skipped_at,
                        "reject_reason": "LOCAL_SEARCH_BUDGET_EXHAUSTED",
                        "error_code": "LOCAL_SEARCH_BUDGET_EXHAUSTED",
                        "seed_pose": {
                            key: seed.get(key) for key in ("x", "y", "z", "yaw") if seed.get(key) is not None
                        },
                    }
                    all_attempts.append(skipped)
                    waypoint_attempts.append(skipped)
                    continue
                started_attempt = self._begin_localization_attempt(
                    len(all_attempts) + 1,
                    seed,
                    extra={"stage": stage_name, "waypoint_index": waypoint_index},
                )
                all_attempts.append(started_attempt)
                waypoint_attempts.append(started_attempt)
                session.update(
                    attempts=all_attempts,
                    active_candidate_number=started_attempt["index"],
                    active_candidate_stage=stage_name,
                    evaluated_candidate_count=max(0, len(all_attempts) - 1),
                    live_pose=started_attempt.get("live_pose"),
                )
                self._report_localization_attempts(session)
                attempt = self._probe_localization_seed(
                    seed,
                    generation,
                    wait_seconds=min(per_seed_wait, remaining_local),
                    index=started_attempt["index"],
                    extra={"stage": stage_name, "waypoint_index": waypoint_index},
                )
                attempt = self._finish_localization_attempt(attempt, started_attempt)
                all_attempts[-1] = attempt
                waypoint_attempts[-1] = attempt
                session.update(
                    attempts=all_attempts,
                    active_candidate_number=None,
                    active_candidate_stage=None,
                    evaluated_candidate_count=len(all_attempts),
                    live_pose=self._current_live_pose(),
                )
                self._report_localization_attempts(session)
            ranked = self._select_ranked_attempt(waypoint_attempts)
            if ranked is not None:
                self._mark_out_ranked_attempts(all_attempts, ranked)
                candidate = dict(ranked.get("ndt_candidate") or {})
                route_stage.update({
                    "status": "accepted",
                    "updated_at": now_iso(),
                    "finished_at": now_iso(),
                    "waypoint_index": ranked.get("waypoint_index"),
                    "attempts": waypoint_attempts,
                    "best_ndt_candidate": candidate,
                })
                self._trusted_pose_frozen = False
                commit_started_at = now_iso()
                committed = self._commit_best_relocalization_candidate(
                    generation,
                    {
                        "source": ranked.get("stage") or "route_waypoint",
                        "waypoint_index": ranked.get("waypoint_index"),
                        "selected_candidate_index": ranked.get("index"),
                    },
                    candidate,
                    all_attempts,
                )
                commit_finished_at = now_iso()
                payload = {
                    **committed,
                    "mode": "progressive_stationary_search",
                    "strategy": strategy,
                    "selected_stage": ranked.get("stage") or "route_waypoint",
                    "selected_waypoint_index": ranked.get("waypoint_index"),
                    "stages": stages,
                    "attempts": all_attempts,
                    "best_match_pose": (committed.get("best_ndt_candidate") or {}).get("matched_pose"),
                    "best_candidate_commit_started_at": commit_started_at,
                    "best_candidate_commit_finished_at": commit_finished_at,
                }
                self._report_localization_attempts({**payload, "state": "accepted"})
                return payload
        finally:
            self._trusted_pose_frozen = False

        if route_stage is not None:
            route_finished_at = now_iso()
            route_stage.update({
                "status": "rejected",
                "updated_at": route_finished_at,
                "finished_at": route_finished_at,
            })

        session["best_ndt_candidate"] = self._best_diagnostic_candidate(all_attempts)

        global_stage = {
            "stage": "keyframe_global_match",
            "status": "searching",
            "started_at": now_iso(),
        }
        global_stage["updated_at"] = global_stage["started_at"]
        stages.append(global_stage)
        session.update(state="global_searching", stages=stages, attempts=all_attempts)
        self._report_localization_attempts(session)
        try:
            global_result = self._global_relocalize_once(
                max(30.0, deadline - time.monotonic()), generation
            )
        except ProtocolError as exc:
            global_stage.update({
                "status": "failed",
                "updated_at": now_iso(),
                "finished_at": now_iso(),
                "error_code": exc.code,
                "error_message": exc.message,
            })
            details = {
                "mode": "progressive_stationary_search",
                "strategy": strategy,
                "stages": stages,
                "attempts": all_attempts,
                "best_ndt_candidate": self._best_diagnostic_candidate(all_attempts),
                "candidate_count": len(all_attempts),
                "evaluated_candidate_count": sum(
                    1 for item in all_attempts if item.get("status") != "skipped"
                ),
                "motion_commanded": False,
                "live_pose": self._current_live_pose(),
                "global_relocalization_error": {
                    "error_code": exc.code,
                    "error_message": exc.message,
                    "details": dict(exc.details or {}),
                },
            }
            self._report_localization_attempts({**details, "state": "failed"})
            raise ProtocolError(
                "PROGRESSIVE_RELOCALIZATION_FAILED",
                "origin, all route waypoints, and keyframe/global matching failed",
                details=details,
            ) from exc
        global_finished_at = now_iso()
        global_stage.update({
            "status": "accepted",
            "updated_at": global_finished_at,
            "finished_at": global_finished_at,
            "message": global_result.get("message"),
        })
        payload = {
            **global_result,
            "mode": "progressive_stationary_search",
            "strategy": strategy,
            "selected_stage": "keyframe_global_match",
            "stages": stages,
            "attempts": all_attempts,
            "best_match_pose": global_result.get("localized_pose"),
        }
        self._report_localization_attempts({**payload, "state": "accepted"})
        return payload

    def _wait_for_fresh_normal_samples(
        self,
        after_sequence: int,
        required_samples: int,
        timeout_seconds: float,
        generation: int | None = None,
    ):
        deadline = time.monotonic() + timeout_seconds
        with self._localization_sample_condition:
            while True:
                if generation is not None:
                    self._assert_localization_operation(generation)
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
        automatic = bool(seed.get("_automatic_recovery"))
        source = str(seed.get("source") or "active_relocalize")
        generation = (
            self._start_localization_operation(source, automatic=True)
            if automatic
            else self._start_localization_operation(source)
        )
        return self._active_relocalize_once(seed, generation)

    def _active_relocalize_once(
        self,
        seed: dict,
        generation: int,
        *,
        persist_state: bool = True,
    ) -> dict:
        """Evaluate every bounded seed, then commit only the ranked best match."""
        base_x = float(seed["x"])
        base_y = float(seed["y"])
        base_z = float(seed.get("z", 0.0))
        base_yaw = float(seed["yaw"])
        candidates = self._relocalization_candidates(base_x, base_y, base_z, base_yaw)
        max_attempts = max(
            1,
            min(len(candidates), int(seed.get("max_attempts", len(candidates)))),
        )
        candidate_wait_seconds = max(
            1.0,
            min(10.0, float(seed.get("candidate_wait_seconds", 8.0))),
        )
        deadline = time.monotonic() + max(1.0, float(seed.get("wait_seconds", 180.0)))
        official_snapshot = self.latest_trusted_pose()
        attempt_metadata = {
            key: seed[key]
            for key in ("stage", "source", "waypoint_index")
            if seed.get(key) is not None
        }
        stage_key = str(seed.get("stage") or "").strip()
        active_stage = None
        if stage_key:
            active_stage = {
                "stage": stage_key,
                "status": "searching",
                "started_at": seed.get("stage_started_at") or now_iso(),
            }
            active_stage["updated_at"] = active_stage["started_at"]
        attempts = [
            {**self._waiting_attempt(index, candidate), **attempt_metadata}
            for index, candidate in enumerate(candidates[:max_attempts], start=1)
        ]
        session = {
            "state": "running",
            "mode": "stationary_bounded_search",
            "source": seed.get("source", "operator_seed"),
            "selected_stage": stage_key or None,
            "stages": [active_stage] if active_stage else [],
            "seed": {k: seed.get(k) for k in ("x", "y", "z", "yaw", "waypoint_index")},
            "candidate_count": max_attempts,
            "evaluated_candidate_count": 0,
            "attempts": attempts,
            "best_match_pose": None,
            "best_ndt_candidate": None,
            "live_pose": self._current_live_pose(),
            "motion_commanded": False,
        }
        self._trusted_pose_frozen = True
        try:
            self._report_localization_attempts(session, persist=persist_state)
            early_stopped = False
            for index, candidate in enumerate(candidates[:max_attempts], start=1):
                remaining = deadline - time.monotonic()
                if remaining < 1.0:
                    break
                started_attempt = self._begin_localization_attempt(
                    index,
                    candidate,
                    extra={
                        "source": seed.get("source", "operator_seed"),
                        **({"stage": seed["stage"]} if seed.get("stage") else {}),
                    },
                )
                attempts[index - 1] = started_attempt
                session.update(
                    attempts=attempts,
                    active_candidate_number=index,
                    active_candidate_stage=stage_key or None,
                    evaluated_candidate_count=index - 1,
                    live_pose=started_attempt.get("live_pose"),
                )
                # NDT verification waits for fresh localization samples.  Send
                # the running candidate before entering that wait so the UI
                # can switch this stage from "待执行" to "执行中" immediately.
                self._report_localization_attempts(session, persist=persist_state)
                completed_attempt = self._probe_localization_seed(
                    candidate,
                    generation,
                    wait_seconds=min(candidate_wait_seconds, remaining),
                    index=index,
                    extra={
                        "source": seed.get("source", "operator_seed"),
                        **({"stage": seed["stage"]} if seed.get("stage") else {}),
                    },
                )
                attempts[index - 1] = self._finish_localization_attempt(
                    completed_attempt, started_attempt
                )
                session.update(
                    attempts=attempts,
                    active_candidate_number=None,
                    active_candidate_stage=None,
                    evaluated_candidate_count=index,
                    live_pose=self._current_live_pose(),
                )
                self._report_localization_attempts(session, persist=persist_state)
                if self._candidate_is_optimal(
                    attempts[index - 1].get("ndt_candidate")
                ):
                    early_stopped = True
                    self._skip_waiting_attempts(attempts, "optimal_threshold_reached")
                    session.update({
                        "early_stopped": True,
                        "stop_reason": "ndt_optimal_score",
                        "evaluated_candidate_count": index,
                    })
                    self._report_localization_attempts(session, persist=persist_state)
                    break
            budget_exhausted = any(item.get("status") == "waiting" for item in attempts)
            if budget_exhausted:
                # A timed out bounded search must never leave future points
                # visually "waiting" after the stage has already moved on.
                self._skip_waiting_attempts(attempts, "local_search_budget_exhausted")
                session.update(
                    attempts=attempts,
                    evaluated_candidate_count=self._evaluated_localization_attempt_count(attempts),
                    timed_out=True,
                )
                self._report_localization_attempts(session, persist=persist_state)
            ranked = self._select_ranked_attempt(attempts)
            if ranked is None:
                self._restore_official_pose(official_snapshot, generation)
                latest = self.telemetry.latest_pose() if getattr(self, "telemetry", None) else None
                session.update({
                    "state": "failed",
                    "best_ndt_candidate": self._best_diagnostic_candidate(attempts),
                    "live_pose": self._current_live_pose(),
                    "localization_status": getattr(latest, "localization_status", None),
                    "timed_out": budget_exhausted,
                })
                if active_stage is not None:
                    stage_finished_at = now_iso()
                    active_stage.update({
                        "status": "failed",
                        "updated_at": stage_finished_at,
                        "finished_at": stage_finished_at,
                        "attempts": attempts,
                        "timed_out": session["timed_out"],
                    })
                self._report_localization_attempts(session, persist=persist_state)
                raise ProtocolError(
                    "ACTIVE_RELOCALIZATION_FAILED",
                    f"stationary search exhausted {self._evaluated_localization_attempt_count(attempts)} candidates; "
                    f"localization_status={getattr(latest, 'localization_status', 'unknown')}",
                    details=session,
                )
            self._mark_out_ranked_attempts(attempts, ranked)
            best_candidate = dict(ranked.get("ndt_candidate") or {})
            session["best_ndt_candidate"] = best_candidate
            session["best_match_pose"] = dict(best_candidate.get("matched_pose") or {})
            session["state"] = "committing_best"
            commit_started_at = now_iso()
            session["best_candidate_commit_started_at"] = commit_started_at
            self._report_localization_attempts(session, persist=persist_state)
            self._trusted_pose_frozen = False
            committed = self._commit_best_relocalization_candidate(
                generation,
                {
                    **seed,
                    # The NDT output can be identical for two seed hypotheses.
                    # Preserve the selected attempt identity instead of trying
                    # to recover it from an equal matched pose at commit time.
                    "selected_candidate_index": ranked.get("index"),
                },
                best_candidate,
                attempts,
            )
            commit_finished_at = now_iso()
            if active_stage is not None:
                active_stage.update({
                    "status": "accepted",
                    "updated_at": commit_finished_at,
                    "finished_at": commit_finished_at,
                    "attempts": attempts,
                    "best_ndt_candidate": best_candidate,
                    "best_match_pose": session["best_match_pose"],
                })
            payload = {
                **committed,
                "best_match_pose": session["best_match_pose"],
                "attempts": attempts,
                "early_stopped": early_stopped,
                "stop_reason": "ndt_optimal_score" if early_stopped else "bounded_search_best",
                "evaluated_candidate_count": self._evaluated_localization_attempt_count(attempts),
                "candidate_count": max_attempts,
                "best_candidate_commit_started_at": commit_started_at,
                "best_candidate_commit_finished_at": commit_finished_at,
            }
            self._report_localization_attempts({**session, **payload, "state": "accepted"}, persist=persist_state)
            return payload
        except ProtocolError:
            raise
        finally:
            self._trusted_pose_frozen = False

    def _waiting_attempt(self, index: int, candidate: dict) -> dict:
        seed_pose = {
            key: candidate[key]
            for key in ("x", "y", "z", "yaw")
            if key in candidate and candidate.get(key) is not None
        }
        return {
            "index": index,
            # Keep an explicit display identity in the progress protocol.
            # `index` remains for compatibility with stored older sessions.
            "candidate_number": index,
            "candidate_label": str(candidate.get("candidate_label") or ""),
            "status": "waiting",
            "seed_pose": seed_pose,
            "x": candidate.get("x"),
            "y": candidate.get("y"),
            "z": candidate.get("z"),
            "yaw": candidate.get("yaw"),
            "live_pose": None,
            "ndt_candidate": None,
            "eligible": False,
            "accepted": False,
        }

    def _begin_localization_attempt(
        self,
        index: int,
        candidate: dict,
        *,
        extra: dict | None = None,
    ) -> dict:
        """Create the progress snapshot sent immediately before NDT waits."""
        attempt = self._waiting_attempt(index, candidate)
        attempt.update(extra or {})
        attempt.update({
            "status": "verifying",
            "started_at": now_iso(),
            "updated_at": now_iso(),
            "live_pose": self._current_live_pose(),
        })
        return attempt

    @staticmethod
    def _finish_localization_attempt(attempt: dict, started_attempt: dict) -> dict:
        """Keep an attempt's identity and start time across its final result."""
        completed = dict(attempt or {})
        completed.setdefault("index", started_attempt.get("index"))
        completed.setdefault("seed_pose", started_attempt.get("seed_pose"))
        completed.setdefault("started_at", started_attempt.get("started_at"))
        completed.setdefault("finished_at", now_iso())
        completed["updated_at"] = completed.get("finished_at") or now_iso()
        return completed

    def _probe_localization_seed(
        self,
        seed_pose: dict,
        generation: int,
        *,
        wait_seconds: float,
        index: int,
        extra: dict | None = None,
        require_ndt_observation: bool = False,
    ) -> dict:
        attempt = self._waiting_attempt(index, seed_pose)
        attempt.update(extra or {})
        attempt["status"] = "started"
        attempt["live_pose"] = self._current_live_pose()
        attempt["status"] = "verifying"
        try:
            result = self._set_initial_pose_once({
                **seed_pose,
                "frame_id": "map",
                "wait_seconds": wait_seconds,
                "required_normal_samples": 3,
                "require_absolute": False,
                "covariance_x": 1.0,
                "covariance_y": 1.0,
                "covariance_yaw": 0.274,
            }, generation)
            candidate = dict(result.get("best_ndt_candidate") or {})
            localized = result.get("localized_pose")
            if isinstance(localized, dict) and not candidate.get("matched_pose"):
                candidate["matched_pose"] = dict(localized)
            if (
                not require_ndt_observation
                and isinstance(localized, dict)
                and "eligible" not in candidate
            ):
                candidate["eligible"] = True
                candidate.setdefault("stable_frames", 3)
        except ProtocolError as exc:
            if exc.code == "RELOCALIZATION_SUPERSEDED":
                raise
            candidate = dict((exc.details or {}).get("best_ndt_candidate") or {})
            attempt["error_code"] = exc.code
            attempt["error_message"] = exc.message
            attempt["reject_reason"] = (
                candidate.get("reject_reason")
                or (exc.details or {}).get("reject_reason")
                or "ndt_sample_unavailable"
            )
        attempt["live_pose"] = self._current_live_pose()
        attempt["ndt_candidate"] = candidate or None
        attempt["matched_pose"] = (candidate or {}).get("matched_pose")
        for field in (
            "has_converged", "matching_error", "inlier_fraction", "geometric_rmse",
            "stable_frames", "required_stable_frames", "quality_failures",
        ):
            if field in candidate:
                attempt[field] = candidate[field]
        if candidate.get("eligible"):
            # NDT verification for this seed has finished.  It is eligible
            # for ranking but is not yet the committed localization result.
            # Keep this distinct from the one candidate currently waiting for
            # localization samples so every point has a clear completion
            # event in the UI.
            attempt["status"] = "qualified"
            attempt["eligible"] = True
            attempt.pop("reject_reason", None)
        else:
            attempt["status"] = "rejected"
            attempt["eligible"] = False
            attempt["accepted"] = False
            attempt.setdefault("reject_reason", "quality_gate")
        return attempt

    def _select_ranked_attempt(self, attempts: list[dict]) -> dict | None:
        eligible = [
            item for item in attempts
            if (item.get("ndt_candidate") or {}).get("eligible")
        ]
        if not eligible:
            return None
        return min(eligible, key=lambda item: self._candidate_rank(item.get("ndt_candidate")))

    def _best_diagnostic_candidate(self, attempts: list[dict]) -> dict | None:
        """Return the strongest observed candidate even when every gate rejects it."""
        candidates = [
            (item, dict(item.get("ndt_candidate") or {}))
            for item in attempts
            if item.get("ndt_candidate")
        ]
        if not candidates:
            return None

        def diagnostic_rank(entry: tuple[dict, dict]) -> tuple:
            attempt, candidate = entry
            score = self._finite_or_none(candidate.get("matching_error"))
            inlier = self._finite_or_none(candidate.get("inlier_fraction"))
            return (
                0 if candidate.get("has_converged") is True else 1,
                0 if score is not None else 1,
                score if score is not None else float("inf"),
                -(inlier if inlier is not None else -1.0),
                int(attempt.get("index") or 0),
            )

        return min(candidates, key=diagnostic_rank)[1]

    @staticmethod
    def _mark_out_ranked_attempts(attempts: list[dict], winner: dict) -> None:
        winner_index = winner.get("index")
        for item in attempts:
            if item.get("index") == winner_index:
                continue
            if (item.get("ndt_candidate") or {}).get("eligible"):
                item["status"] = "rejected"
                item["reject_reason"] = "out_ranked"
                item["accepted"] = False

    def _current_live_pose(self) -> dict | None:
        telemetry = getattr(self, "telemetry", None)
        latest = telemetry.latest_pose() if telemetry is not None and hasattr(telemetry, "latest_pose") else None
        return self._pose_payload(latest)

    @staticmethod
    def _pose_payload(pose) -> dict | None:
        if pose is None:
            return None
        if isinstance(pose, dict):
            x, y, yaw = pose.get("x"), pose.get("y"), pose.get("yaw")
            z = pose.get("z")
        else:
            x = getattr(pose, "x", None)
            y = getattr(pose, "y", None)
            yaw = getattr(pose, "yaw", None)
            z = getattr(pose, "z", None)
        values = []
        for value in (x, y, yaw):
            try:
                number = float(value)
            except (TypeError, ValueError):
                return None
            if not math.isfinite(number):
                return None
            values.append(number)
        payload = {"x": values[0], "y": values[1], "yaw": values[2]}
        try:
            z_number = float(z)
        except (TypeError, ValueError):
            z_number = None
        if z_number is not None and math.isfinite(z_number):
            payload["z"] = z_number
        return payload

    def _restore_official_pose(self, snapshot: dict | None, generation: int) -> None:
        pose = self._pose_payload(snapshot)
        if not pose:
            return
        try:
            self._set_initial_pose_once({
                **pose,
                "frame_id": snapshot.get("frame_id", "map") if isinstance(snapshot, dict) else "map",
                "wait_seconds": 3.0,
                "require_absolute": False,
                "required_normal_samples": 0,
            }, generation)
        except ProtocolError as exc:
            if exc.code == "RELOCALIZATION_SUPERSEDED":
                raise
            LOGGER.warning("unable to restore official localization pose after failed search: %s", exc)

    def _report_localization_attempts(self, session: dict, *, persist: bool = True) -> None:
        # Progress consumers may serialize asynchronously.  A shallow copy
        # lets later attempt/stage mutations rewrite an already published
        # "searching" snapshot as "failed", which makes the UI jump back to
        # the previous status and timestamp.
        payload = copy.deepcopy({
            **session,
            "live_pose": session.get("live_pose") or self._current_live_pose(),
            "updated_at": time.time(),
        })
        if persist:
            self._persist_relocalization_state(payload)
        callback = getattr(self, "_attempt_progress_cb", None)
        if not callable(callback):
            return
        try:
            callback({
                "localization_attempts": payload,
                "best_ndt_candidate": payload.get("best_ndt_candidate"),
                "best_match_pose": payload.get("best_match_pose"),
                "live_pose": payload.get("live_pose"),
            })
        except Exception:
            LOGGER.exception("localization attempt progress callback failed")

    def _commit_best_relocalization_candidate(
        self,
        generation: int,
        seed: dict,
        candidate: dict,
        attempts: list[dict],
    ) -> dict:
        best_pose = dict(candidate["matched_pose"])
        commit_started_at = now_iso()
        requested_winner_index = seed.get("selected_candidate_index")
        try:
            winner_index = int(requested_winner_index) if requested_winner_index is not None else None
        except (TypeError, ValueError):
            winner_index = None
        if winner_index is None:
            winner_index = next(
                (item.get("index") for item in attempts if (item.get("ndt_candidate") or {}) is candidate
                 or (item.get("ndt_candidate") or {}).get("matched_pose") == candidate.get("matched_pose")),
                None,
            )
        winner_attempt = next(
            (item for item in attempts if item.get("index") == winner_index),
            None,
        )
        commit_metadata = {
            "best_candidate_index": winner_index,
            "best_candidate_stage": (
                (winner_attempt or {}).get("stage")
                or seed.get("stage")
                or seed.get("source")
                or "operator_seed"
            ),
            "best_candidate_seed_pose": dict((winner_attempt or {}).get("seed_pose") or {}),
            "best_candidate_label": str((winner_attempt or {}).get("candidate_label") or ""),
            "best_candidate_ndt": {
                "matching_error": candidate.get("matching_error"),
                "inlier_fraction": candidate.get("inlier_fraction"),
                "has_converged": candidate.get("has_converged"),
                "stable_frames": candidate.get("stable_frames"),
                "required_stable_frames": candidate.get("required_stable_frames"),
            },
        }
        for item in attempts:
            if item.get("index") == winner_index or (
                winner_index is None
                and (item.get("ndt_candidate") or {}).get("matched_pose") == candidate.get("matched_pose")
            ):
                item["status"] = "committing"
        state = {
            "state": "committing_best",
            "source": seed.get("source", "operator_seed"),
            "attempts": attempts,
            "best_ndt_candidate": candidate,
            "best_match_pose": best_pose,
            "live_pose": self._current_live_pose(),
            "best_candidate_commit_started_at": commit_started_at,
            **commit_metadata,
        }
        self._report_localization_attempts(state)
        try:
            result = self._set_initial_pose_once({
                **best_pose,
                "frame_id": "map",
                "wait_seconds": 20.0,
                "required_normal_samples": 3,
                "require_absolute": True,
                "covariance_x": 0.25,
                "covariance_y": 0.25,
                "covariance_yaw": 0.0685,
            }, generation)
        except ProtocolError as exc:
            if exc.code == "RELOCALIZATION_SUPERSEDED":
                raise
            for item in attempts:
                if item.get("index") != winner_index:
                    continue
                item.update({
                    "status": "failed",
                    "accepted": False,
                    "reject_reason": "lio_handoff_failed",
                    "error_code": exc.code,
                    "error_message": exc.message,
                    "finished_at": now_iso(),
                })
            details = {
                "mode": "stationary_bounded_search",
                "source": seed.get("source", "operator_seed"),
                "attempts": attempts,
                "best_ndt_candidate": candidate,
                "best_match_pose": best_pose,
                "best_ndt_committed": True,
                "handoff_pending": True,
                "motion_commanded": False,
                "best_candidate_commit_started_at": commit_started_at,
                "best_candidate_commit_finished_at": now_iso(),
                **commit_metadata,
            }
            self._report_localization_attempts({**details, "state": "handoff_failed"})
            raise ProtocolError(
                "RELOCALIZATION_HANDOFF_FAILED",
                "best NDT pose was committed but FAST-LIO did not become stable",
                details=details,
            ) from exc
        for item in attempts:
            matched = (item.get("ndt_candidate") or {}).get("matched_pose")
            if matched == candidate.get("matched_pose"):
                item["status"] = "accepted"
                item["accepted"] = True
                item["reject_reason"] = None
            elif item.get("status") not in {"rejected", "failed", "waiting"}:
                item["status"] = "rejected"
                item.setdefault("reject_reason", "out_ranked")
                item["accepted"] = False
        payload = {
            "mode": "stationary_bounded_search",
            "source": seed.get("source", "operator_seed"),
            "attempts": attempts,
            "best_ndt_candidate": candidate,
            "best_match_pose": best_pose,
            "best_ndt_committed": True,
            "localized_pose": result["localized_pose"],
            "localization_status": result["localization_status"],
            "motion_commanded": False,
            "best_candidate_commit_started_at": commit_started_at,
            "best_candidate_commit_finished_at": now_iso(),
            **commit_metadata,
        }
        latest = self.telemetry.latest_pose()
        if self._trusted_pose_cb and latest:
            self._trusted_pose_cb(latest)
            self._last_trusted_pose_report_monotonic = time.monotonic()
            self._last_trusted_pose = {
                "x": float(latest.x),
                "y": float(latest.y),
                "z": float(latest.z),
                "yaw": float(latest.yaw),
                "sampled_at": getattr(latest, "sampled_at", None),
                "frame_id": "map",
            }
        self._report_localization_attempts({**payload, "state": "accepted"})
        return payload

    def _persist_relocalization_state(self, payload: dict) -> None:
        path = self._relocalization_state_file
        if not path:
            return
        try:
            parent = os.path.dirname(path)
            os.makedirs(parent, exist_ok=True)
            temporary = f"{path}.tmp"
            with open(temporary, "w", encoding="utf-8") as stream:
                json.dump(payload, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        except OSError as exc:
            LOGGER.warning("unable to persist relocalization search state: %s", exc)

    @staticmethod
    def _relocalization_candidates(x: float, y: float, z: float, yaw: float) -> list[dict]:
        def normalize(angle: float) -> float:
            return math.atan2(math.sin(angle), math.cos(angle))

        yaw_hypotheses = (
            (0.0, "中心点·原始航向"),
            (math.pi / 4, "中心点·左转 45°"),
            (-math.pi / 4, "中心点·右转 45°"),
            (math.pi / 2, "中心点·左转 90°"),
            (-math.pi / 2, "中心点·右转 90°"),
            (3 * math.pi / 4, "中心点·左转 135°"),
            (-3 * math.pi / 4, "中心点·右转 135°"),
            (math.pi, "中心点·反向 180°"),
        )
        candidates = [
            {
                "x": x,
                "y": y,
                "z": z,
                "yaw": normalize(yaw + offset),
                "candidate_label": label,
            }
            for offset, label in yaw_hypotheses
        ]
        # Expand the position search in bounded rings.  The first ring keeps
        # the correction local; the outer rings recover a robot that stopped
        # near, but not exactly on, the waypoint without allowing an unbounded
        # pose jump.
        for radius in (0.3, 0.6, 1.0):
            candidates.extend(
                {
                    "x": x + dx * radius,
                    "y": y + dy * radius,
                    "z": z,
                    "yaw": normalize(yaw),
                    "candidate_label": f"周边 {direction} {radius:.1f} m",
                }
                for dx, dy, direction in (
                    (1.0, 0.0, "+X"),
                    (-1.0, 0.0, "-X"),
                    (0.0, 1.0, "+Y"),
                    (0.0, -1.0, "-Y"),
                )
            )
        return candidates

    def clear_local_costmap(self, timeout_seconds: float = 1.0) -> bool:
        """Clear rolling lidar marks so Nav2 can retry a local detour."""
        try:
            from nav2_msgs.srv import ClearEntireCostmap
        except ImportError:
            LOGGER.warning("nav2_msgs ClearEntireCostmap is unavailable")
            return False
        client = self._persistent_service_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )
        try:
            if not client.wait_for_service(timeout_sec=min(timeout_seconds, 0.5)):
                return False
            future = client.call_async(ClearEntireCostmap.Request())
            if not self._wait_for_future(future, timeout_seconds):
                return False
            return future.done() and future.exception() is None
        except Exception:
            LOGGER.warning("local costmap clear request failed", exc_info=True)
            return False

    def cancel_navigation(self, timeout_seconds: float = 5.0) -> bool:
        deadline = time.monotonic() + max(0.1, float(timeout_seconds))
        last_failure = "not acknowledged"
        while time.monotonic() < deadline:
            remaining = max(0.1, deadline - time.monotonic())
            try:
                goal_handle = self._goal_handle
                if goal_handle is not None:
                    future = goal_handle.cancel_goal_async()
                else:
                    # Edge may have restarted after it sent a goal. In that
                    # case the local handle is gone while Nav2 continues
                    # executing the goal; cancel every goal on this action.
                    client = self._persistent_service_client(
                        CancelGoal,
                        f"{self._nav_cancel_action}/_action/cancel_goal",
                    )
                    if not client.wait_for_service(timeout_sec=min(remaining, 2.0)):
                        last_failure = "cancel service unavailable"
                        break
                    future = client.call_async(CancelGoal.Request())
                completed = threading.Event()
                future.add_done_callback(lambda _: completed.set())
                completed.wait(timeout=min(remaining, 2.0))
                response = future.result() if future.done() else None
                if bool(response and response.goals_canceling):
                    return True
                # Nav2 may finish or cancel the goal while this request is in
                # flight.  In that race the cancel response legitimately has
                # an empty goals_canceling list; an empty global cancellation
                # result likewise means there is no active goal left to stop.
                # Treat these terminal states as confirmed cancellation.
                if goal_handle is None:
                    return True
                status = getattr(goal_handle, "status", None)
                if self._goal_handle is not goal_handle or status in (4, 5, 6):
                    return True
                last_failure = "not acknowledged"
            except Exception as exc:
                last_failure = str(exc)
                LOGGER.warning("%s cancellation attempt failed: %s", self._nav_cancel_action, exc)
            if time.monotonic() < deadline:
                time.sleep(min(0.10, max(0.0, deadline - time.monotonic())))
        LOGGER.error("%s cancellation was not acknowledged: %s", self._nav_cancel_action, last_failure)
        return False

    def stop_motion(self) -> None:
        """Publish an explicit zero command after a navigation goal is cancelled."""
        zero = Twist()
        # Keep the diagnostic speed source consistent with the command that is
        # being published.  The local subscription normally observes this too,
        # but updating eagerly avoids one stale non-zero status frame.
        zeroed_at = time.monotonic()
        self._raw_forward_command = 0.0
        self._raw_lateral_command = 0.0
        self._raw_turn_command = 0.0
        self._raw_velocity_updated_monotonic = zeroed_at
        self._actual_forward_command = 0.0
        self._actual_lateral_command = 0.0
        self._actual_turn_command = 0.0
        self._actual_velocity_updated_monotonic = zeroed_at
        self._standstill_started_monotonic = None
        # Keep the zero command alive long enough to cover one controller and
        # collision-monitor cycle.  Repeating is safe and makes this operation
        # idempotent when completion/cancel/recovery paths race.
        for _ in range(10):
            # Feed the zero into the collision monitor as well as publishing
            # the final output. Publishing only /cmd_vel races the monitor's
            # last non-zero /cmd_vel_raw and can immediately overwrite stop.
            self._arrival_adjust_cmd_vel_pub.publish(zero)
            self._cmd_vel_pub.publish(zero)
            time.sleep(0.05)

    def _publish_nav_selector(self, publisher, plugin_id: str) -> None:
        message = String()
        message.data = plugin_id
        publisher.publish(message)

    def set_local_controller(self, mode: str) -> None:
        normalized = normalize_local_controller(mode)
        if normalized == getattr(self, "_active_local_controller", None):
            return
        plugin_id = local_controller_plugin_id(normalized)
        publisher = getattr(self, "_controller_selector_pub", None)
        if publisher is not None:
            self._publish_nav_selector(publisher, plugin_id)
        self._active_local_controller = normalized
        LOGGER.info("local controller set to %s (%s)", normalized, plugin_id)

    def set_global_controller(self, mode: str) -> None:
        normalized = normalize_global_controller(mode)
        if normalized == self._active_global_controller:
            return
        plugin_id = global_controller_plugin_id(normalized)
        planner_params = {
            f"{plugin_id}.use_astar": normalized == "navfn",
        }
        # Apply and verify the algorithm parameters before publishing the
        # selector.  A failed write must not silently run a different planner
        # than the one saved and displayed by the platform.
        self._set_remote_parameters(
            "/planner_server",
            planner_params,
            code="GLOBAL_CONTROLLER_FAILED",
        )
        self._publish_nav_selector(self._planner_selector_pub, plugin_id)
        self._active_global_controller = normalized
        LOGGER.info("global controller set to %s (%s)", normalized, plugin_id)

    def set_smoother(self, smoother_id: str) -> None:
        """Select the already-loaded SmootherServer plugin for this leg."""
        selected = str(smoother_id or "savitzky_golay")
        if selected == getattr(self, "_active_smoother", None):
            return
        publisher = getattr(self, "_smoother_selector_pub", None)
        if publisher is not None:
            self._publish_nav_selector(publisher, selected)
        self._active_smoother = selected
        LOGGER.info("path smoother set to %s", selected)

    def set_docking_profile(self, *, final_approach: bool) -> None:
        """Apply the slow, fine-control profile used only for the dock contact leg."""
        vx_max = 0.10 if final_approach else 0.5
        vx_min = -0.10 if final_approach else -0.12
        wz_max = 0.20 if final_approach else 0.5
        fine_control = Bool()
        fine_control.data = bool(final_approach)
        self._fine_control_pub.publish(fine_control)
        self._boundary_base_velocity = {"vx_max": vx_max, "vx_min": vx_min, "vy_max": 0.0}
        boundary_limit = getattr(self, "_boundary_zone_speed_limit", None)
        if boundary_limit is not None:
            vx_max = min(vx_max, boundary_limit)
            vx_min = -min(abs(vx_min), boundary_limit)
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

    def set_safety_profile(
        self,
        *,
        detour_enabled: bool,
        collision_slowdown_enabled: bool = True,
        collision_stop_enabled: bool = True,
    ) -> None:
        """Apply the three-layer avoidance model with fail-closed hard-stop policy.

        ``collision_stop_enabled`` is forced on for ordinary legs. Docking may
        request a temporary exception elsewhere under a separate audited path.
        """
        stop_enabled = True
        if not collision_stop_enabled:
            LOGGER.warning(
                "collision_stop_enabled=false requested; keeping PolygonStop enabled"
            )
        signature = (
            bool(detour_enabled),
            bool(collision_slowdown_enabled),
            bool(stop_enabled),
        )
        if signature == getattr(self, "_safety_profile_signature", None):
            return
        for node_name, parameter_name, value in (
            ("/local_costmap/local_costmap", "obstacle_layer.enabled", bool(detour_enabled)),
            ("/collision_monitor", "PolygonSlow.enabled", bool(collision_slowdown_enabled)),
            ("/collision_monitor", "PolygonStop.enabled", stop_enabled),
        ):
            self._set_remote_parameters(
                node_name,
                {parameter_name: value},
                code="SAFETY_PROFILE_FAILED",
                attempts=2,
            )
        self._safety_profile_signature = signature
        LOGGER.info(
            "safety profile detour=%s slowdown=%s stop=%s",
            detour_enabled,
            collision_slowdown_enabled,
            stop_enabled,
        )

    def list_navigation_capabilities(self) -> dict:
        """Return the planner/controller plugins this Edge build knows how to drive."""
        from .navigation_controllers import navigation_capabilities

        caps = navigation_capabilities()
        try:
            planner_plugins = self._get_remote_parameters("/planner_server", ["planner_plugins"]).get(
                "planner_plugins"
            )
            controller_plugins = self._get_remote_parameters(
                "/controller_server", ["controller_plugins"]
            ).get("controller_plugins")
        except ProtocolError:
            planner_plugins = None
            controller_plugins = None
        caps = dict(caps)
        caps["runtime_planner_plugins"] = planner_plugins
        caps["runtime_controller_plugins"] = controller_plugins
        return caps

    def apply_navigation_profile(
        self,
        *,
        generation: int,
        global_controller: str,
        local_controller: str = "mppi",
        detour_enabled: bool = True,
        collision_slowdown_enabled: bool = True,
        collision_stop_enabled: bool = True,
        require_yaw: bool = False,
        final_approach: bool = False,
        outdoor: bool | None = None,
        smoother_id: str = "savitzky_golay",
        live: bool = False,
        navigation_speed_level: str = "micro",
        reapproach: bool = False,
    ) -> dict:
        """Atomically apply a leg profile, read it back, and roll back on failure."""
        previous = copy.deepcopy(getattr(self, "_last_good_navigation_profile", None))
        normalized_global = normalize_global_controller(global_controller)
        normalized_local = normalize_local_controller(local_controller)
        global_plugin = global_controller_plugin_id(normalized_global)
        local_plugin = local_controller_plugin_id(normalized_local)
        use_outdoor = (
            self._rtk_is_navigation_pose_source() if outdoor is None else bool(outdoor)
        )
        try:
            self.set_global_controller(normalized_global)
            expected_astar = normalized_global == "navfn"
            planner_readback = self._get_remote_parameters(
                "/planner_server",
                [f"{global_plugin}.use_astar"],
            )
            actual_astar = planner_readback.get(f"{global_plugin}.use_astar")
            if actual_astar is not None and bool(actual_astar) != expected_astar:
                raise ProtocolError(
                    "NAV_PROFILE_READBACK_FAILED",
                    f"{global_plugin}.use_astar readback={actual_astar} expected={expected_astar}",
                )

            self.set_safety_profile(
                detour_enabled=detour_enabled,
                collision_slowdown_enabled=collision_slowdown_enabled,
                collision_stop_enabled=collision_stop_enabled,
            )
            safety_readback = self._get_remote_parameters(
                "/local_costmap/local_costmap",
                ["obstacle_layer.enabled"],
            )
            slow_readback = self._get_remote_parameters(
                "/collision_monitor",
                ["PolygonSlow.enabled", "PolygonStop.enabled"],
            )
            if safety_readback.get("obstacle_layer.enabled") is not None and bool(
                safety_readback["obstacle_layer.enabled"]
            ) != bool(detour_enabled):
                raise ProtocolError(
                    "NAV_PROFILE_READBACK_FAILED",
                    "obstacle_layer.enabled readback mismatch",
                )
            if slow_readback.get("PolygonSlow.enabled") is not None and bool(
                slow_readback["PolygonSlow.enabled"]
            ) != bool(collision_slowdown_enabled):
                raise ProtocolError(
                    "NAV_PROFILE_READBACK_FAILED",
                    "PolygonSlow.enabled readback mismatch",
                )
            if slow_readback.get("PolygonStop.enabled") is False:
                raise ProtocolError(
                    "NAV_PROFILE_READBACK_FAILED",
                    "PolygonStop.enabled must remain true",
                )

            self.set_waypoint_profile(
                avoid_obstacles=detour_enabled,
                require_yaw=require_yaw,
                final_approach=final_approach,
                live=live,
                outdoor=use_outdoor,
                local_controller=normalized_local,
                navigation_speed_level=navigation_speed_level,
                reapproach=reapproach,
            )
            self.set_local_controller(normalized_local)
            self.set_smoother(smoother_id)
            self.apply_outdoor_gps_profile(outdoor=use_outdoor)

            applied = {
                "generation": int(generation),
                "global_controller": normalized_global,
                "global_plugin_id": global_plugin,
                "local_controller": normalized_local,
                "local_plugin_id": local_plugin,
                "detour_enabled": bool(detour_enabled),
                "collision_slowdown_enabled": bool(collision_slowdown_enabled),
                "collision_stop_enabled": True,
                "require_yaw": bool(require_yaw),
                "final_approach": bool(final_approach),
                "outdoor": bool(use_outdoor),
                "smoother_id": str(smoother_id),
                "navigation_speed_level": navigation_speed_profile(
                    navigation_speed_level, getattr(self, "safety_config", None)
                ).level,
                "readback": {
                    "planner_use_astar": actual_astar,
                    "obstacle_layer.enabled": safety_readback.get("obstacle_layer.enabled"),
                    "PolygonSlow.enabled": slow_readback.get("PolygonSlow.enabled"),
                    "PolygonStop.enabled": slow_readback.get("PolygonStop.enabled"),
                },
            }
            self._last_good_navigation_profile = copy.deepcopy(applied)
            LOGGER.info(
                "navigation profile applied generation=%s global=%s(%s) local=%s(%s)",
                generation,
                normalized_global,
                global_plugin,
                normalized_local,
                local_plugin,
            )
            return applied
        except ProtocolError:
            if previous is not None:
                try:
                    self._restore_navigation_profile(previous)
                    LOGGER.warning(
                        "rolled back navigation profile to generation=%s after apply failure",
                        previous.get("generation"),
                    )
                except Exception:
                    LOGGER.exception("navigation profile rollback failed")
            raise

    def _restore_navigation_profile(self, snapshot: dict) -> None:
        self.set_global_controller(str(snapshot.get("global_controller") or "theta_star"))
        self.set_safety_profile(
            detour_enabled=bool(snapshot.get("detour_enabled", True)),
            collision_slowdown_enabled=bool(snapshot.get("collision_slowdown_enabled", True)),
            collision_stop_enabled=True,
        )
        self.set_waypoint_profile(
            avoid_obstacles=bool(snapshot.get("detour_enabled", True)),
            require_yaw=bool(snapshot.get("require_yaw", False)),
            final_approach=bool(snapshot.get("final_approach", False)),
            outdoor=bool(snapshot.get("outdoor", False)),
            local_controller=str(snapshot.get("local_controller") or "mppi"),
            navigation_speed_level=str(snapshot.get("navigation_speed_level") or "micro"),
        )
        self.set_local_controller(str(snapshot.get("local_controller") or "mppi"))
        self.set_smoother(str(snapshot.get("smoother_id") or "savitzky_golay"))
        self.apply_outdoor_gps_profile(outdoor=bool(snapshot.get("outdoor", False)))
        self._last_good_navigation_profile = copy.deepcopy(snapshot)

    def set_waypoint_profile(
        self,
        *,
        avoid_obstacles: bool,
        require_yaw: bool,
        final_approach: bool = False,
        live: bool = False,
        outdoor: bool | None = None,
        local_controller: str = "mppi",
        navigation_speed_level: str = "micro",
        reapproach: bool = False,
    ) -> None:
        self.set_local_controller(local_controller)
        yaw_message = Bool()
        yaw_message.data = bool(require_yaw)
        self._goal_yaw_required_pub.publish(yaw_message)
        use_outdoor_profile = (
            self._rtk_is_navigation_pose_source() if outdoor is None else bool(outdoor)
        )
        local_obstacles = bool(avoid_obstacles)
        normalized_local = normalize_local_controller(local_controller)
        speed_profile = navigation_speed_profile(
            navigation_speed_level, getattr(self, "safety_config", None)
        )
        use_mppi = normalized_local == "mppi"
        use_rpp = normalized_local == "rpp"
        use_ilqr = normalized_local == "ilqr"
        signature = (
            normalize_local_controller(local_controller),
            bool(require_yaw),
            bool(final_approach),
            bool(reapproach),
            bool(live),
            bool(use_outdoor_profile),
            bool(local_obstacles),
            speed_profile,
        )
        if (
            not live
            and signature == getattr(self, "_waypoint_profile_signature", None)
        ):
            LOGGER.info(
                "waypoint profile unchanged; skipping Nav2 parameter writes "
                "(local=%s outdoor=%s final_approach=%s)",
                signature[0],
                use_outdoor_profile,
                final_approach,
            )
            return
        follow_applied = False
        params: dict[str, bool | float] = {}
        if use_mppi:
            params = follow_path_patrol_params(
                final_approach=final_approach,
                local_obstacles=local_obstacles,
                require_yaw=require_yaw,
                outdoor=use_outdoor_profile,
                speed_profile=speed_profile,
                reapproach=reapproach,
            )
            self._boundary_base_velocity = {
                "vx_max": float(params["FollowPath.vx_max"]),
                "vx_min": float(params["FollowPath.vx_min"]),
                "vy_max": float(params["FollowPath.vy_max"]),
            }
            boundary_limit = getattr(self, "_boundary_zone_speed_limit", None)
            if boundary_limit is not None:
                params["FollowPath.vx_max"] = min(params["FollowPath.vx_max"], boundary_limit)
                params["FollowPath.vx_min"] = -min(abs(params["FollowPath.vx_min"]), boundary_limit)
                params["FollowPath.vy_max"] = min(params["FollowPath.vy_max"], boundary_limit)
            # Terminal MPPI is a hard safety/settling mode, not a best-effort
            # tuning hint.  Keep these two values authoritative even after a
            # boundary cap or a stale cruise profile has been applied.
            if final_approach:
                params["FollowPath.GoalCritic.enabled"] = True
                params["FollowPath.vx_min"] = 0.0
            try:
                self._set_remote_parameters(
                    "/controller_server",
                    params,
                    code="WAYPOINT_PROFILE_FAILED",
                    attempts=2 if live else 3,
                )
                if final_approach:
                    self._verify_mppi_terminal_profile()
                follow_applied = True
            except ProtocolError:
                LOGGER.warning("unable to apply FollowPath waypoint speed profile")
                # Continuing with the old cruise profile would re-enable
                # reverse hunting exactly at the point where the robot must
                # settle.  Let the caller enter its safe-hold path instead.
                if final_approach:
                    raise
        elif use_rpp:
            params = {
                "RPP.desired_linear_vel": 0.18 if final_approach else speed_profile.vx_mps,
                "RPP.min_linear_vel": 0.03 if final_approach else 0.05,
                "RPP.lookahead_dist": 0.40 if final_approach else 1.2,
                "RPP.min_lookahead_dist": 0.25 if final_approach else 0.6,
                "RPP.max_lookahead_dist": 0.8 if final_approach else 1.8,
                "RPP.use_velocity_scaled_lookahead_dist": not final_approach,
                "RPP.lookahead_time": 1.5 if final_approach else 2.5,
                "RPP.max_angular_vel": 0.30 if final_approach else speed_profile.wz_rps,
                "RPP.rotate_to_heading_threshold": 0.35 if require_yaw else 0.52,
                "RPP.rotate_to_heading_angular_vel": 0.25 if require_yaw else 0.22,
                "RPP.use_regulated_linear_velocity_scaling": True,
                "RPP.angular_deadband": 0.03,
            }
            self._boundary_base_velocity = {
                "vx_max": float(params["RPP.desired_linear_vel"]),
                "vx_min": 0.0,
                "vy_max": 0.0,
            }
            boundary_limit = getattr(self, "_boundary_zone_speed_limit", None)
            if boundary_limit is not None:
                params["RPP.desired_linear_vel"] = min(
                    params["RPP.desired_linear_vel"], boundary_limit
                )
            try:
                self._set_remote_parameters(
                    "/controller_server", params,
                    code="WAYPOINT_PROFILE_FAILED",
                    attempts=2 if live else 3,
                )
                follow_applied = True
            except ProtocolError:
                LOGGER.warning("unable to apply RPP waypoint speed profile")
        elif use_ilqr:
            params = {
                "ILQR.desired_linear_vel": 0.14 if final_approach else speed_profile.vx_mps,
                "ILQR.max_angular_vel": 0.25 if final_approach else speed_profile.wz_rps,
            }
            self._boundary_base_velocity = {
                "vx_max": float(params["ILQR.desired_linear_vel"]),
                "vx_min": 0.0,
                "vy_max": 0.0,
            }
            boundary_limit = getattr(self, "_boundary_zone_speed_limit", None)
            if boundary_limit is not None:
                params["ILQR.desired_linear_vel"] = min(
                    params["ILQR.desired_linear_vel"], boundary_limit
                )
            try:
                self._set_remote_parameters(
                    "/controller_server", params,
                    code="WAYPOINT_PROFILE_FAILED",
                    attempts=2 if live else 3,
                )
                follow_applied = True
            except ProtocolError:
                LOGGER.warning("unable to apply ILQR waypoint speed profile")
        if not live:
            # Prefer the explicit three-layer safety API. CostCritic still tracks
            # avoid_obstacles via FollowPath params above.
            try:
                self.set_safety_profile(
                    detour_enabled=local_obstacles,
                    collision_slowdown_enabled=local_obstacles,
                    collision_stop_enabled=True,
                )
            except ProtocolError:
                LOGGER.warning(
                    "unable to apply safety profile; keeping previous collision layers"
                )
        if not live:
            self._waypoint_profile_signature = signature
        else:
            # Live final-approach writes only the controller; clear the cache so
            # the next full profile apply refreshes costmaps if needed.
            self._waypoint_profile_signature = None
            self._safety_profile_signature = None
        LOGGER.info(
            "waypoint profile local=%s speed=%s outdoor=%s final_approach=%s live=%s follow_applied=%s vx=[%s,%s] wz_max=%s path_align=%s cost=%s cost_weight=%s",
            normalize_local_controller(local_controller),
            speed_profile.level,
            use_outdoor_profile,
            final_approach,
            live,
            follow_applied,
            params.get("FollowPath.vx_min") if use_mppi else (
                params.get("RPP.min_linear_vel") if use_rpp else 0.0
            ),
            params.get("FollowPath.vx_max") if use_mppi else (
                params.get("RPP.desired_linear_vel") if use_rpp else params.get("ILQR.desired_linear_vel")
            ),
            params.get("FollowPath.wz_max") if use_mppi else (
                params.get("RPP.max_angular_vel") if use_rpp else params.get("ILQR.max_angular_vel")
            ),
            params.get("FollowPath.PathAlignCritic.enabled") if use_mppi else None,
            params.get("FollowPath.CostCritic.enabled") if use_mppi else None,
            params.get("FollowPath.CostCritic.cost_weight") if use_mppi else None,
        )

        self._active_navigation_speed_profile = speed_profile
        self._active_navigation_final_approach = bool(final_approach)
        self._active_navigation_reapproach_speed_mps = (
            {"mppi": 0.15, "rpp": 0.18, "ilqr": 0.14}.get(normalized_local)
            if reapproach
            else None
        )
        self._navigation_speed_last_limit_mps = None
        self._navigation_speed_last_update_monotonic = time.monotonic()
        # A new leg starts from its safe terminal speed. Feedback then raises
        # the cap with a bounded ramp; this avoids a parameter-write jump to
        # the remote-monitoring maximum.
        self.update_navigation_speed_envelope(0.0, force=True)

    def _verify_mppi_terminal_profile(self) -> None:
        """Confirm the two non-negotiable terminal MPPI parameters."""
        getter = getattr(self, "_get_remote_parameters", None)
        if (
            not callable(getter)
            or getattr(self, "_nav_service_callback_group", None) is None
        ):
            # Lightweight simulation adapters do not expose ROS parameter
            # services; the write itself is still covered by their tests.
            return
        names = ["FollowPath.GoalCritic.enabled", "FollowPath.vx_min"]
        actual = getter(
            "/controller_server",
            names,
            code="WAYPOINT_PROFILE_READBACK_FAILED",
            attempts=2,
        )
        if actual.get(names[0]) is not True:
            raise ProtocolError(
                "WAYPOINT_PROFILE_READBACK_FAILED",
                f"terminal GoalCritic readback={actual.get(names[0])!r} expected=True",
            )
        value = actual.get(names[1])
        if not isinstance(value, (int, float)) or not math.isclose(
            float(value), 0.0, rel_tol=0.0, abs_tol=1e-6
        ):
            raise ProtocolError(
                "WAYPOINT_PROFILE_READBACK_FAILED",
                f"terminal vx_min readback={value!r} expected=0.0",
            )

    def update_navigation_speed_envelope(
        self, distance_remaining_m: float | None, *, force: bool = False
    ) -> float | None:
        """Publish a smooth controller speed cap for the active route leg.

        The selected tier is an upper bound. Remaining distance creates a
        braking envelope, while the boundary remains a stricter cap. MPPI,
        RPP and iLQR consume the same absolute ``/speed_limit`` topic.
        """
        profile = getattr(self, "_active_navigation_speed_profile", None)
        publisher = getattr(self, "_navigation_speed_limit_pub", None)
        if profile is None or publisher is None:
            return None
        try:
            remaining = max(0.0, float(distance_remaining_m or 0.0))
        except (TypeError, ValueError):
            return None
        self._navigation_speed_last_distance_remaining_m = remaining
        safety = getattr(self, "safety_config", None)
        final_speed = max(0.01, float(getattr(safety, "navigation_speed_final_mps", 0.15)))
        decel = max(0.05, float(getattr(safety, "navigation_speed_decel_mps2", 1.0)))
        accel = max(0.05, float(getattr(safety, "navigation_speed_accel_mps2", 0.8)))
        interval = max(0.05, float(getattr(safety, "navigation_speed_update_seconds", 0.5)))
        reapproach_speed = getattr(
            self, "_active_navigation_reapproach_speed_mps", None
        )
        if reapproach_speed is not None:
            desired = float(reapproach_speed)
        elif getattr(self, "_active_navigation_final_approach", False):
            desired = final_speed
        elif profile.level == "micro":
            # The established micro profile is a constant 0.30 m/s cap; only
            # explicitly selected low/medium/high legs use the envelope ramp.
            desired = float(profile.vx_mps)
        else:
            brake_cap = math.sqrt(final_speed * final_speed + 2.0 * decel * remaining)
            desired = min(float(profile.vx_mps), brake_cap)
        boundary = getattr(self, "_boundary_zone_speed_limit", None)
        if boundary is not None:
            desired = min(desired, float(boundary))
        now = time.monotonic()
        previous = getattr(self, "_navigation_speed_last_limit_mps", None)
        previous_at = getattr(self, "_navigation_speed_last_update_monotonic", now)
        elapsed = max(0.0, now - previous_at)
        if previous is None:
            limited = (
                desired
                if profile.level == "micro" or reapproach_speed is not None
                else final_speed
            )
        elif desired >= previous:
            limited = min(desired, previous + accel * elapsed)
        else:
            limited = max(desired, previous - decel * elapsed)
        if (
            not force
            and previous is not None
            and now - previous_at < interval
            and abs(limited - previous) < 1e-3
        ):
            return previous
        message = SpeedLimit()
        message.speed_limit = float(limited)
        message.percentage = False
        publisher.publish(message)
        self._navigation_speed_last_limit_mps = float(limited)
        self._navigation_speed_last_update_monotonic = now
        return float(limited)

    def apply_outdoor_gps_profile(self, *, outdoor: bool | None = None) -> None:
        """Select an outdoor RTK line planner or restore the indoor map planner."""
        use_outdoor_profile = (
            self._rtk_is_navigation_pose_source() if outdoor is None else bool(outdoor)
        )
        if getattr(self, "_outdoor_planner_profile", None) == use_outdoor_profile:
            return
        # The default must also be a registered planner.  Falling back to the
        # removed GridBased id makes a profile write fail while the UI still
        # reports Theta*, so normalize an empty selection to ThetaStar.
        raw_controller = getattr(self, "_active_global_controller", None)
        plugin_id = global_controller_plugin_id(normalize_global_controller(raw_controller))
        params = (
            {
                f"{plugin_id}.allow_straight_line_fallback": True,
                f"{plugin_id}.prefer_straight_line": True,
                f"{plugin_id}.tolerance": 2.0,
            }
            if use_outdoor_profile
            else {
                f"{plugin_id}.allow_straight_line_fallback": False,
                f"{plugin_id}.prefer_straight_line": False,
                f"{plugin_id}.tolerance": 0.5,
            }
        )
        self._set_remote_parameters(
            "/planner_server",
            params,
            code="WAYPOINT_PROFILE_FAILED",
            attempts=2,
        )
        self._outdoor_planner_profile = use_outdoor_profile

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

    def set_arrival_goal_tolerance(
        self, tolerance_m: float, *, yaw_tolerance_rad: float = 0.25
    ) -> None:
        """Set and read back the Nav2 radius for the next waypoint goal."""
        tolerance = max(0.01, float(tolerance_m))
        expected = {
            "general_goal_checker.xy_goal_tolerance": tolerance,
            "general_goal_checker.required_yaw_goal_tolerance": max(
                0.01, float(yaw_tolerance_rad)
            ),
        }
        names = list(expected)
        previous = self._get_remote_parameters(
            "/controller_server",
            names,
            code="ARRIVAL_GOAL_TOLERANCE_READBACK_FAILED",
            attempts=2,
        )
        self._set_remote_parameters(
            "/controller_server", expected,
            code="ARRIVAL_GOAL_TOLERANCE_FAILED",
            attempts=4,
        )
        actual = self._get_remote_parameters(
            "/controller_server",
            names,
            code="ARRIVAL_GOAL_TOLERANCE_READBACK_FAILED",
            attempts=3,
        )
        mismatches = {
            name: {"expected": expected[name], "actual": actual.get(name)}
            for name in names
            if not isinstance(actual.get(name), (int, float))
            or not math.isclose(
                float(actual[name]), float(expected[name]), rel_tol=0.0, abs_tol=1e-6
            )
        }
        if mismatches:
            rollback = {
                name: value
                for name, value in previous.items()
                if isinstance(value, (bool, int, float))
            }
            if len(rollback) == len(expected):
                try:
                    self._set_remote_parameters(
                        "/controller_server",
                        rollback,
                        code="ARRIVAL_GOAL_TOLERANCE_ROLLBACK_FAILED",
                        attempts=2,
                    )
                except ProtocolError:
                    LOGGER.exception("failed to roll back Nav2 arrival tolerances")
            raise ProtocolError(
                "ARRIVAL_GOAL_TOLERANCE_READBACK_FAILED",
                f"Nav2 arrival tolerance readback mismatch: {mismatches}",
            )
        self._last_arrival_goal_tolerance_readback = dict(actual)

    def set_arrival_micro_goal_profile(
        self, *, enabled: bool, tolerance_m: float = 0.15
    ) -> None:
        """Temporarily tighten Nav2 only for one post-yaw micro goal.

        The Edge still performs its own localization and pose confirmation
        after the action result; this profile is only a motion controller
        contract and must never be treated as an arrival verdict.
        """
        xy_tolerance = max(0.01, float(tolerance_m)) if enabled else 0.35
        planner_tolerance = 0.05 if enabled else (
            2.0 if getattr(self, "_outdoor_planner_profile", False) else 0.5
        )
        plugin_id = global_controller_plugin_id(
            normalize_global_controller(getattr(self, "_active_global_controller", None))
        )
        try:
            self._set_remote_parameters(
                "/controller_server",
                {
                    "general_goal_checker.xy_goal_tolerance": xy_tolerance,
                    "general_goal_checker.required_yaw_goal_tolerance": 0.25,
                    "progress_checker.required_movement_radius": 0.05 if enabled else 0.15,
                    "progress_checker.movement_time_allowance": 8.0 if enabled else 10.0,
                },
                code="ARRIVAL_MICRO_GOAL_PROFILE_FAILED",
                attempts=4 if enabled else 2,
            )
            self._set_remote_parameters(
                "/planner_server",
                {f"{plugin_id}.tolerance": planner_tolerance},
                code="ARRIVAL_MICRO_GOAL_PROFILE_FAILED",
                attempts=4 if enabled else 2,
            )
        except ProtocolError:
            LOGGER.warning("unable to set arrival micro-goal profile enabled=%s", enabled)
            if enabled:
                raise

    def _remote_param_values_match(
        self,
        node_name: str,
        values: dict[str, bool | int | float],
    ) -> bool:
        cached = getattr(self, "_remote_param_cache", None) or {}
        node_cache = cached.get(node_name)
        if not node_cache:
            return False
        return all(node_cache.get(name) == value for name, value in values.items())

    def _mark_remote_param_unavailable(self, node_name: str) -> None:
        store = getattr(self, "_remote_param_unavailable_until", None)
        if store is None:
            self._remote_param_unavailable_until = {}
            store = self._remote_param_unavailable_until
        store[node_name] = time.monotonic() + self._REMOTE_PARAM_UNAVAILABLE_COOLDOWN_SECONDS

    def _remote_param_is_cooling_down(self, node_name: str) -> bool:
        store = getattr(self, "_remote_param_unavailable_until", None) or {}
        until = store.get(node_name)
        return until is not None and time.monotonic() < until

    def _get_remote_parameters(
        self,
        node_name: str,
        names: list[str],
        *,
        code: str = "NAV_PROFILE_READBACK_FAILED",
        attempts: int = 3,
    ) -> dict[str, bool | int | float | list | None]:
        """Read Nav2 parameters through the ROS get_parameters service."""
        if not names:
            return {}
        if not ROS_AVAILABLE or GetParameters is None:
            raise ProtocolError(code, "ROS get_parameters unavailable")
        last_error = ""
        for _ in range(max(1, attempts)):
            client = self._persistent_service_client(
                GetParameters,
                f"{node_name}/get_parameters",
            )
            try:
                if not client.wait_for_service(timeout_sec=0.35):
                    last_error = f"{node_name} get_parameters service is unavailable"
                    continue
                request = GetParameters.Request()
                request.names = list(names)
                future = client.call_async(request)
                if not self._wait_for_future(future, 1.0):
                    last_error = f"{node_name} get_parameters timed out"
                    continue
                response = future.result()
                values = getattr(response, "values", None) or []
                result: dict[str, bool | int | float | list | None] = {}
                for name, value in zip(names, values):
                    result[name] = self._parameter_value_to_python(value)
                return result
            except Exception as exc:
                last_error = str(exc)
            time.sleep(0.05)
        raise ProtocolError(code, last_error or f"unable to get parameters on {node_name}")

    @staticmethod
    def _parameter_value_to_python(value) -> bool | int | float | list | str | None:
        if value is None:
            return None
        value_type = int(getattr(value, "type", 0) or 0)
        if value_type == ParameterType.PARAMETER_BOOL:
            return bool(value.bool_value)
        if value_type == ParameterType.PARAMETER_INTEGER:
            return int(value.integer_value)
        if value_type == ParameterType.PARAMETER_DOUBLE:
            return float(value.double_value)
        if value_type == ParameterType.PARAMETER_STRING:
            return str(value.string_value)
        if value_type == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(value.bool_array_value)
        if value_type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(value.integer_array_value)
        if value_type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(value.double_array_value)
        if value_type == ParameterType.PARAMETER_STRING_ARRAY:
            return list(value.string_array_value)
        return None

    def _set_remote_parameters(
        self,
        node_name: str,
        values: dict[str, bool | int | float],
        *,
        code: str,
        attempts: int = 8,
    ) -> None:
        """Set Nav2 parameters through its ROS service, without spawning ros2 CLI processes."""
        if self._remote_param_values_match(node_name, values):
            return
        if self._remote_param_is_cooling_down(node_name):
            raise ProtocolError(
                code,
                f"{node_name} parameter service recently unavailable; skipping until cooldown ends",
            )
        last_error = ""
        service_missing = False
        for _ in range(max(1, attempts)):
            client = self._persistent_service_client(
                SetParameters,
                f"{node_name}/set_parameters",
            )
            try:
                if not client.wait_for_service(timeout_sec=0.25):
                    last_error = f"{node_name} parameter service is unavailable"
                    service_missing = True
                    # Missing services almost never appear mid-patrol; fail fast.
                    break
                request = SetParameters.Request()
                request.parameters = [
                    self._parameter_message(name, value) for name, value in values.items()
                ]
                future = client.call_async(request)
                if not self._wait_for_future(future, 0.75):
                    last_error = f"{node_name} parameter request timed out"
                    service_missing = True
                else:
                    response = future.result()
                    failures = [
                        result.reason or "rejected"
                        for result in response.results
                        if not result.successful
                    ]
                    if not failures:
                        cache = getattr(self, "_remote_param_cache", None)
                        if cache is None:
                            self._remote_param_cache = {}
                            cache = self._remote_param_cache
                        merged = dict(cache.get(node_name) or {})
                        merged.update(values)
                        cache[node_name] = merged
                        unavailable = getattr(self, "_remote_param_unavailable_until", None)
                        if unavailable is not None:
                            unavailable.pop(node_name, None)
                        return
                    last_error = "; ".join(failures)
                    service_missing = False
            except Exception as exc:
                last_error = str(exc)
            time.sleep(0.05)
        if service_missing:
            self._mark_remote_param_unavailable(node_name)
        raise ProtocolError(code, last_error or f"unable to set parameters on {node_name}")

    def set_boundary_speed_limit(self, speed_limit_mps: float | None) -> None:
        """Apply the most restrictive active map-zone speed to the selected controller."""
        maximum = max(
            0.05,
            float(
                getattr(
                    getattr(self, "safety_config", None),
                    "navigation_boundary_speed_limit_max_mps",
                    3.0,
                )
            ),
        )
        self._boundary_zone_speed_limit = (
            None if speed_limit_mps is None else max(0.05, min(maximum, float(speed_limit_mps)))
        )
        base = getattr(self, "_boundary_base_velocity", {"vx_max": 0.30, "vx_min": -0.12, "vy_max": 0.5})
        limit = base["vx_max"] if self._boundary_zone_speed_limit is None else min(base["vx_max"], self._boundary_zone_speed_limit)
        active = normalize_local_controller(getattr(self, "_active_local_controller", None))
        if active == "rpp":
            controller_params = {"RPP.desired_linear_vel": limit}
        elif active == "ilqr":
            controller_params = {"ILQR.desired_linear_vel": limit}
        else:
            controller_params = {
                "FollowPath.vx_max": limit,
                "FollowPath.vx_min": -min(abs(base["vx_min"]), limit),
                "FollowPath.vy_max": min(base["vy_max"], limit),
            }
        self._set_remote_parameters(
            "/controller_server",
            controller_params,
            code="BOUNDARY_SPEED_LIMIT_FAILED",
            attempts=2,
        )
        self.update_navigation_speed_envelope(
            getattr(self, "_navigation_speed_last_distance_remaining_m", 0.0), force=True
        )

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

    def is_robot_stopped(self, timeout_seconds: float | None = None) -> bool:
        if timeout_seconds is None:
            timeout_seconds = self.safety_config.stop_confirmation_seconds + 3.0
        deadline = time.monotonic() + max(0.0, float(timeout_seconds))
        while time.monotonic() < deadline:
            # _is_stopped_from_velocity owns the only continuous-stillness
            # timer. Adding another hold window here used to make a configured
            # one-second confirmation take roughly two seconds.
            if self._is_stopped_from_velocity(time.monotonic()):
                return True
            time.sleep(0.05)
        return False

    def _velocity_command_requests_motion(
        self, forward: float, lateral: float, turn: float
    ) -> bool:
        linear_threshold = float(getattr(
            self.safety_config, "stop_speed_threshold_mps", 0.03
        ))
        angular_threshold = float(getattr(
            self.safety_config, "standstill_angular_speed_threshold", 0.025
        ))
        return (
            math.hypot(float(forward), float(lateral)) > linear_threshold
            or abs(float(turn)) > angular_threshold
        )

    def _is_stopped_from_velocity(self, now: float) -> bool:
        """Evaluate the last command without requiring static zero republishing.

        Collision Monitor intentionally stops publishing /cmd_vel after a
        stationary timeout. A stale *zero* therefore remains valid, while a
        stale non-zero command is never accepted for recovery.
        """
        # Do not inspect odom until FAST-LIO has had time to publish a stable
        # stream after startup.  This prevents zero/default odom from falsely
        # confirming a stop during initialization.
        data_delay = float(getattr(self.safety_config, "standstill_data_valid_delay", 1.2))
        if now - float(getattr(self, "_lio_started_monotonic", now)) < max(0.0, data_delay):
            self._standstill_started_monotonic = None
            return False
        odom_received = float(getattr(self, "_odom_received_monotonic", 0.0) or 0.0)
        if odom_received <= 0.0 or now - odom_received > 0.30:
            self._standstill_started_monotonic = None
            return False
        linear_threshold = float(getattr(
            self.safety_config, "standstill_linear_speed_threshold", 0.03
        ))
        angular_threshold = float(getattr(
            self.safety_config, "standstill_angular_speed_threshold", 0.025
        ))
        odom_planar = math.hypot(self._odom_linear_x, self._odom_linear_y)
        odom_still = odom_planar <= linear_threshold and abs(self._odom_angular_z) <= angular_threshold
        if not odom_still:
            self._standstill_started_monotonic = None
            return False
        # The last value from each side of the control chain is an independent
        # safety veto. /cmd_vel_raw captures upstream motion intent while
        # /cmd_vel captures Collision Monitor output. A source that last said
        # non-zero remains unsafe even when stale; only a later zero from that
        # source (or stop_motion's explicit dual zero) clears its veto.
        for forward, lateral, turn in (
            (self._raw_forward_command, self._raw_lateral_command,
             self._raw_turn_command),
            (self._actual_forward_command, self._actual_lateral_command,
             self._actual_turn_command),
        ):
            if self._velocity_command_requests_motion(forward, lateral, turn):
                self._standstill_started_monotonic = None
                return False
        self._standstill_started_monotonic = self._standstill_started_monotonic or now
        return now - self._standstill_started_monotonic >= float(
            getattr(self.safety_config, "standstill_hold_time", 1.0)
        )


class RosRuntime:
    _MAX_CONSECUTIVE_SPIN_FAILURES = 20

    def __init__(
        self,
        node: RosAdapter,
        unexpected_exit_callback: Callable[[str], None] | None = None,
    ) -> None:
        self.node = node
        # Safety pose and /cmd_vel callbacks have their own callback group.
        # Reserve executor capacity for them when control/telemetry callbacks
        # are busy, otherwise a blocked recovery callback can freeze the Edge
        # cache even though ROS itself is still publishing fresh data.
        self.executor = MultiThreadedExecutor(num_threads=5)
        self.executor.add_node(node)
        self._stopped = threading.Event()
        self._unexpected_exit_callback = unexpected_exit_callback
        self._exit_reason = "not_started"
        self.thread = threading.Thread(target=self._spin, daemon=True, name="ros-executor")
        node._ros_executor_alive_provider = self.is_alive

    def is_alive(self) -> bool:
        return bool(self.thread.is_alive() and not self._stopped.is_set())

    def _spin(self) -> None:
        # spin() itself dies on the first callback exception. A destroyed
        # service client used to raise InvalidHandle here and freeze every
        # pose, speed, and status callback until Edge was restarted.
        consecutive_failures = 0
        self._exit_reason = "running"
        try:
            while not self._stopped.is_set():
                if rclpy is not None and hasattr(rclpy, "ok") and not rclpy.ok():
                    self._exit_reason = "rclpy_context_not_ok"
                    break
                try:
                    self.executor.spin_once(timeout_sec=0.1)
                    consecutive_failures = 0
                except Exception as exc:
                    # A callback/service race can throw once without
                    # invalidating the executor. Retry it, but do not leave a
                    # permanently broken wait set spinning forever while MQTT
                    # continues to report online.
                    consecutive_failures += 1
                    LOGGER.exception(
                        "ROS executor callback failed (%d/%d); keeping subscriptions alive",
                        consecutive_failures,
                        self._MAX_CONSECUTIVE_SPIN_FAILURES,
                    )
                    if consecutive_failures >= self._MAX_CONSECUTIVE_SPIN_FAILURES:
                        self._exit_reason = (
                            f"consecutive_spin_failures:{type(exc).__name__}"
                        )
                        break
                    time.sleep(0.05)
        except BaseException as exc:
            self._exit_reason = f"executor_thread_terminated:{type(exc).__name__}"
            raise
        finally:
            if not self._stopped.is_set():
                callback = self._unexpected_exit_callback
                reason = self._exit_reason or "executor_spin_ended"
                LOGGER.critical("ROS executor stopped unexpectedly: %s", reason)
                if callable(callback):
                    callback(reason)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self._stopped.set()
        self.executor.shutdown()
        self.node.destroy_node()
        self.thread.join(timeout=3)
