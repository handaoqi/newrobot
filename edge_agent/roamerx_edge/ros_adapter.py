from __future__ import annotations

import logging
import threading
import time
from typing import Callable

from .config import RosConfig, SafetyConfig
from .safety_policy import RuntimeSafetyState
from .telemetry_collector import TelemetryCollector

LOGGER = logging.getLogger(__name__)

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import FollowWaypoints
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from robots_dog_msgs.msg import Localization

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
        self._action_client = ActionClient(self, FollowWaypoints, ros_config.follow_waypoints_action)

    def _on_localization(self, msg) -> None:
        self._latest_speed = float(msg.speed)
        self.telemetry.on_localization(msg)

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
            # action_msgs/GoalStatus: SUCCEEDED=4, CANCELED=5, ABORTED=6
            mapped = "succeeded" if status == 4 else "cancelled" if status == 5 else "failed"
            if self._result_cb:
                self._result_cb(mapped, "" if mapped != "failed" else f"goal_status={status}")
        except Exception as exc:
            LOGGER.exception("navigation result callback failed")
            if self._result_cb:
                self._result_cb("failed", str(exc))
        finally:
            self._goal_handle = None

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
