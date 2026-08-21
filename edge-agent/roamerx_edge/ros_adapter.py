from __future__ import annotations

import logging
import json
import math
import threading
import time
from collections import deque
from typing import Callable

from .config import RosConfig, SafetyConfig
from .protocol import ProtocolError
from .safety_policy import RuntimeSafetyState
from .telemetry_collector import TelemetryCollector

LOGGER = logging.getLogger(__name__)

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import FollowWaypoints
    from action_msgs.srv import CancelGoal
    from lifecycle_msgs.srv import GetState
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from robots_dog_msgs.msg import Localization
    from localization.msg import ScanMatchingStatus
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from rcl_interfaces.msg import Parameter as ParameterMessage, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    Node = object


class RosAdapter(Node):
    def __init__(
        self,
        ros_config: RosConfig,
        safety_config: SafetyConfig,
        telemetry: TelemetryCollector,
        safety_state: RuntimeSafetyState,
    ) -> None:
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS2 Python packages are not available")
        super().__init__("roamerx_edge_agent")
        self.ros_config = ros_config
        self.safety_config = safety_config
        self.telemetry = telemetry
        self.safety_state = safety_state
        self._goal_handle = None
        self._result_cb: Callable | None = None
        self._feedback_cb: Callable | None = None
        self._localization_failure_cb: Callable | None = None
        self._localization_recovery_cb: Callable | None = None
        self._trusted_pose_cb: Callable | None = None
        self._last_trusted_pose: dict | None = None
        self._last_trusted_pose_report_monotonic = 0.0
        self._localization_sample_condition = threading.Condition()
        self._localization_sample_sequence = 0
        self._localization_status_samples = deque(maxlen=100)
        self._localization_lost_count = 0
        self._localization_failure_notified = False
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
        self.create_subscription(
            Localization,
            ros_config.localization_topic,
            self._on_localization,
            10,
        )
        self.create_subscription(Twist, ros_config.cmd_vel_raw_topic, self._on_cmd_vel_raw, 10)
        self.create_subscription(Twist, ros_config.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(LaserScan, ros_config.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, "/sensor_health", self._on_sensor_health, 2)
        self.create_subscription(String, "/localization/decision", self._on_localization_decision, 10)
        if ros_config.scan_matching_status_topic:
            self.create_subscription(
                ScanMatchingStatus,
                ros_config.scan_matching_status_topic,
                self._on_scan_matching_status,
                qos_profile_sensor_data,
            )
        self._action_client = ActionClient(self, FollowWaypoints, ros_config.follow_waypoints_action)
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 8)
        self._rtk_initial_pose_client = self.create_client(Trigger, "/localization/seed_from_rtk")
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
        if isinstance(payload, dict):
            self.telemetry.on_localization_decision(payload)

    def set_localization_policy(self, source: str, phase: str) -> dict:
        source = "rtk" if str(source).lower() == "rtk" else "ndt"
        phase = "moving" if str(phase).lower() == "moving" else "stationary"
        msg = String()
        msg.data = f"{phase}:{source}"
        self._localization_policy_pub.publish(msg)
        return {"topic": "/localization/policy", "source": source, "phase": phase}

    def localization_decision(self) -> dict:
        return self.telemetry.localization_decision()

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
                daemon=True,
                name="localization-loss-handler",
            ).start()

    def set_localization_failure_callback(self, callback: Callable) -> None:
        self._localization_failure_cb = callback

    def set_localization_recovery_callback(self, callback: Callable) -> None:
        self._localization_recovery_cb = callback

    def set_trusted_pose_callback(self, callback: Callable) -> None:
        self._trusted_pose_cb = callback

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
        score = float(getattr(msg, "matching_error", float("inf")))
        healthy = bool(getattr(msg, "has_converged", False)) and math.isfinite(score) and (
            score <= self.safety_config.ndt_failure_score
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
                daemon=True,
                name="ndt-failure-handler",
            ).start()

    def _absolute_localization_stable(self) -> bool:
        decision = self.telemetry.localization_decision()
        return bool(
            decision.get("active_source") in {"ndt_imu", "rtk_imu"}
            and decision.get("absolute_stable")
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

    def _on_sensor_health(self, msg) -> None:
        try:
            sensors = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            LOGGER.warning("invalid /sensor_health payload")
            return
        for name, details in sensors.items():
            if isinstance(details, dict):
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

    def wait_until_ready(self, timeout_seconds: float = 10.0) -> bool:
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
        # A zero timeout is used by the periodic health reporter.  It must
        # still perform one non-blocking probe rather than immediately
        # reporting Nav2 as unavailable.
        while True:
            remaining = max(0.0, deadline - time.monotonic())
            if not self._action_client.wait_for_server(timeout_sec=min(0.5, remaining)):
                ready = False
            else:
                ready = all(self._lifecycle_node_is_active(name) for name in required_nodes)
            if ready:
                self.safety_state.nav_ready = True
                return True
            if time.monotonic() >= deadline:
                break
            time.sleep(0.2)
        self.safety_state.nav_ready = False
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
        goal = FollowWaypoints.Goal()
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(waypoint["x"])
            pose.pose.position.y = float(waypoint["y"])
            yaw = float(waypoint.get("yaw", 0.0))
            pose.pose.orientation.z = __import__("math").sin(yaw / 2)
            pose.pose.orientation.w = __import__("math").cos(yaw / 2)
            goal.poses.append(pose)
        self._feedback_cb = feedback_cb
        self._result_cb = result_cb
        future = self._action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
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
        for _ in range(3):
            self._remote_teleop_action_pub.publish(msg)
            time.sleep(0.08)
        return {"topic": "/remote_teleop_action", "action": msg.data, "publish_count": 3}

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
            for dx, dy in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))
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
                CancelGoal, f"{self.ros_config.follow_waypoints_action}/_action/cancel_goal")
            if not client.wait_for_service(timeout_sec=min(timeout_seconds, 2.0)):
                self.destroy_client(client)
                LOGGER.error("FollowWaypoints cancel service is unavailable")
                return False
            future = client.call_async(CancelGoal.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        completed.wait(timeout=timeout_seconds)
        response = future.result() if future.done() else None
        cancelled = bool(response and response.goals_canceling)
        if not cancelled:
            LOGGER.error("FollowWaypoints cancellation was not acknowledged")
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

    def set_waypoint_profile(self, *, avoid_obstacles: bool, require_yaw: bool) -> None:
        enabled = "true" if avoid_obstacles else "false"
        yaw_message = Bool()
        yaw_message.data = bool(require_yaw)
        self._goal_yaw_required_pub.publish(yaw_message)
        for node_name, parameter_name in (
            ("/local_costmap/local_costmap", "obstacle_layer.enabled"),
            ("/global_costmap/global_costmap", "obstacle_layer.enabled"),
            ("/collision_monitor", "PolygonStop.enabled"),
            ("/collision_monitor", "PolygonSlow.enabled"),
        ):
            self._set_remote_parameters(
                node_name,
                {parameter_name: bool(avoid_obstacles)},
                code="WAYPOINT_PROFILE_FAILED",
            )

    def set_goal_precision(self, *, enabled: bool) -> None:
        """Select the tight pose tolerances used only for the dock contact point."""
        xy_tolerance = (
            self.safety_config.docking_goal_tolerance_m if enabled else 0.35
        )
        yaw_tolerance = (
            self.safety_config.docking_goal_yaw_tolerance_rad if enabled else 0.25
        )
        self._set_remote_parameters(
            "/controller_server",
            {
                "general_goal_checker.xy_goal_tolerance": float(xy_tolerance),
                "general_goal_checker.required_yaw_goal_tolerance": float(yaw_tolerance),
            },
            code="GOAL_PRECISION_PROFILE_FAILED",
        )

    def _set_remote_parameters(self, node_name: str, values: dict[str, bool | float], *, code: str) -> None:
        """Set Nav2 parameters through its ROS service, without spawning ros2 CLI processes."""
        last_error = ""
        for _ in range(8):
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
    def _parameter_message(name: str, value: bool | float) -> ParameterMessage:
        parameter = ParameterMessage()
        parameter.name = name
        parameter.value = ParameterValue()
        if isinstance(value, bool):
            parameter.value.type = ParameterType.PARAMETER_BOOL
            parameter.value.bool_value = value
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
