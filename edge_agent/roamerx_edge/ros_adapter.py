from __future__ import annotations

import logging
import math
import threading
import time
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
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from robots_dog_msgs.msg import Localization
    from localization.msg import ScanMatchingStatus
    from std_msgs.msg import String

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
        self._latest_speed = 0.0
        self.create_subscription(
            Localization,
            ros_config.localization_topic,
            self._on_localization,
            10,
        )
        if ros_config.scan_matching_status_topic:
            self.create_subscription(
                ScanMatchingStatus,
                ros_config.scan_matching_status_topic,
                self._on_scan_matching_status,
                qos_profile_sensor_data,
            )
        self._action_client = ActionClient(self, FollowWaypoints, ros_config.follow_waypoints_action)
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 8)
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._teleop_action_pub = self.create_publisher(String, "/teleop_action", 10)

    def _on_localization(self, msg) -> None:
        self._latest_speed = float(msg.speed)
        self.telemetry.on_localization(msg)

    def _on_scan_matching_status(self, msg) -> None:
        LOGGER.debug(
            "scan matching status: converged=%s matching_error=%.3f inlier_fraction=%.3f",
            bool(getattr(msg, "has_converged", False)),
            float(getattr(msg, "matching_error", 0.0)),
            float(getattr(msg, "inlier_fraction", 0.0)),
        )
        self.telemetry.on_scan_matching_status(msg)

    def wait_until_ready(self, timeout_seconds: float = 10.0) -> bool:
        ready = self._action_client.wait_for_server(timeout_sec=timeout_seconds)
        self.safety_state.nav_ready = bool(ready)
        return bool(ready)

    def send_waypoints(self, waypoints: list[dict], feedback_cb: Callable, result_cb: Callable) -> bool:
        if not self.wait_until_ready(timeout_seconds=5):
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
        self._cmd_vel_pub.publish(msg)
        return {
            "topic": "/cmd_vel",
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
        for _ in range(5):
            self._initial_pose_pub.publish(msg)
            time.sleep(0.2)
        deadline = time.monotonic() + float(pose.get("wait_seconds", 8.0))
        latest = self.telemetry.latest_pose()
        while time.monotonic() < deadline:
            latest = self.telemetry.latest_pose()
            if latest and latest.localization_status == "normal":
                break
            time.sleep(0.2)
        if not latest or latest.localization_status != "normal":
            status = latest.localization_status if latest else "unknown"
            raise ProtocolError("INITIAL_POSE_NOT_ACCEPTED", f"localization_status={status}")
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

    def cancel_navigation(self, timeout_seconds: float = 5.0) -> bool:
        if self._goal_handle is None:
            return True
        future = self._goal_handle.cancel_goal_async()
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        completed.wait(timeout=timeout_seconds)
        return bool(future.done() and future.result() and future.result().goals_canceling)

    def is_robot_stopped(self) -> bool:
        deadline = time.monotonic() + self.safety_config.stop_confirmation_seconds + 3
        stable_since = None
        while time.monotonic() < deadline:
            if abs(self._latest_speed) <= self.safety_config.stop_speed_threshold_mps:
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
