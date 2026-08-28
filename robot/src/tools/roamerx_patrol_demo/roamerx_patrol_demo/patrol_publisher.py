"""Publish deterministic indoor patrol data using standard ROS2 messages."""

from __future__ import annotations

import base64
import json
import math
import time
from typing import Iterable

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import BatteryState, CompressedImage, Imu, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from .bag_contract import REQUIRED_TOPICS, validate_topic_names
from .scenario import EPOCH_NS, DEFAULT_WAYPOINTS, PatrolConfig, PatrolScenario, RobotSample


_PNG_1X1 = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII="
)


def _qos(depth: int, reliability: ReliabilityPolicy, durability: DurabilityPolicy) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability,
        durability=durability,
    )


def _stamp(ns: int):
    from builtin_interfaces.msg import Time

    value = Time()
    value.sec = ns // 1_000_000_000
    value.nanosec = ns % 1_000_000_000
    return value


def _quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class PatrolPublisher(Node):
    """A bounded publisher suitable for demos and offline recording."""

    def __init__(self) -> None:
        super().__init__("roamerx_patrol_demo")
        self.declare_parameter("duration_sec", 120.0)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("linear_speed_mps", 0.35)
        self.declare_parameter("angular_speed_rps", 0.6)
        self.declare_parameter("topic_namespace", "")
        self.declare_parameter("waypoints", "7.0,1.0;7.0,6.0;2.0,6.0;2.0,3.0")
        self.declare_parameter("seed", 20260828)
        self.declare_parameter("publish_camera", True)
        self.declare_parameter("publish_scan", True)
        self.declare_parameter("publish_imu", True)

        self.config = PatrolConfig(
            duration_sec=float(self.get_parameter("duration_sec").value),
            publish_rate_hz=float(self.get_parameter("publish_rate_hz").value),
            linear_speed_mps=float(self.get_parameter("linear_speed_mps").value),
            angular_speed_rps=float(self.get_parameter("angular_speed_rps").value),
            topic_namespace=str(self.get_parameter("topic_namespace").value),
            waypoints=self._parse_waypoints(str(self.get_parameter("waypoints").value)),
            seed=int(self.get_parameter("seed").value),
            publish_camera=bool(self.get_parameter("publish_camera").value),
            publish_scan=bool(self.get_parameter("publish_scan").value),
            publish_imu=bool(self.get_parameter("publish_imu").value),
        )
        self.scenario = PatrolScenario(self.config)
        self._start_monotonic = time.monotonic()
        self._last_elapsed_ns = -1
        self._finished = False
        #: Resolved once the patrol has run to completion; main() spins on it.
        self._done = Future()
        self._next: dict[str, int] = {name: 0 for name in ("tf", "odom", "cmd", "scan", "imu", "image", "path", "plan", "status", "battery", "diagnostics")}
        self._period_ns = {
            "tf": 50_000_000,
            "odom": 50_000_000,
            "cmd": 50_000_000,
            "scan": 100_000_000,
            "imu": 20_000_000,
            "image": 500_000_000,
            "path": 500_000_000,
            "plan": 1_000_000_000,
            "status": 1_000_000_000,
            "battery": 1_000_000_000,
            "diagnostics": 1_000_000_000,
        }
        reliable = ReliabilityPolicy.RELIABLE
        best_effort = ReliabilityPolicy.BEST_EFFORT
        volatile = DurabilityPolicy.VOLATILE
        transient = DurabilityPolicy.TRANSIENT_LOCAL
        self._pub: dict[str, object] = {
            "clock": self.create_publisher(Clock, self._topic("/clock"), _qos(10, reliable, volatile)),
            "map": self.create_publisher(OccupancyGrid, self._topic("/map"), _qos(1, reliable, transient)),
            "tf": self.create_publisher(TFMessage, self._topic("/tf"), _qos(100, reliable, volatile)),
            "tf_static": self.create_publisher(TFMessage, self._topic("/tf_static"), _qos(1, reliable, transient)),
            "odom": self.create_publisher(Odometry, self._topic("/odom"), _qos(10, reliable, volatile)),
            "cmd": self.create_publisher(Twist, self._topic("/cmd_vel"), _qos(10, reliable, volatile)),
            "scan": self.create_publisher(LaserScan, self._topic("/scan"), _qos(5, best_effort, volatile)),
            "image": self.create_publisher(CompressedImage, self._topic("/camera/front/image/compressed"), _qos(2, best_effort, volatile)),
            "imu": self.create_publisher(Imu, self._topic("/imu/data"), _qos(10, best_effort, volatile)),
            "battery": self.create_publisher(BatteryState, self._topic("/battery_state"), _qos(10, reliable, volatile)),
            "diagnostics": self.create_publisher(DiagnosticArray, self._topic("/diagnostics"), _qos(10, reliable, volatile)),
            "plan": self.create_publisher(Path, self._topic("/plan"), _qos(1, reliable, volatile)),
            "goal": self.create_publisher(PoseStamped, self._topic("/goal_pose"), _qos(1, reliable, volatile)),
            "path": self.create_publisher(Path, self._topic("/patrol/trajectory"), _qos(2, reliable, volatile)),
            "status": self.create_publisher(String, self._topic("/patrol/status"), _qos(1, reliable, transient)),
        }
        self._publish_map()
        self._publish_static_tf()
        tick_hz = max(1.0, min(100.0, self.config.publish_rate_hz))
        self._timer = self.create_timer(1.0 / tick_hz, self._tick)

    @property
    def done_future(self) -> Future:
        return self._done

    @staticmethod
    def _parse_waypoints(raw: str) -> tuple[tuple[float, float], ...]:
        if not raw.strip():
            return DEFAULT_WAYPOINTS
        points: list[tuple[float, float]] = []
        try:
            for item in raw.split(";"):
                x, y = item.split(",")
                points.append((float(x), float(y)))
        except (ValueError, TypeError) as exc:
            raise ValueError("waypoints must be semicolon-separated x,y pairs") from exc
        return tuple(points)

    def _topic(self, name: str) -> str:
        namespace = self.config.topic_namespace.rstrip("/")
        return f"{namespace}{name}" if namespace else name

    def _publish_map(self) -> None:
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.header.stamp = _stamp(EPOCH_NS)
        message.info.resolution = 0.05
        message.info.width = 200
        message.info.height = 160
        message.info.origin.position.x = 0.0
        message.info.origin.position.y = 0.0
        data = [0] * (message.info.width * message.info.height)
        for y in range(message.info.height):
            for x in range(message.info.width):
                if x in (0, message.info.width - 1) or y in (0, message.info.height - 1):
                    data[y * message.info.width + x] = 100
        # A small fixed obstacle leaves the route visible while making the
        # diagnostics/avoidance segment meaningful.
        for y in range(112, 124):
            for x in range(78, 92):
                data[y * message.info.width + x] = 100
        message.data = data
        self._pub["map"].publish(message)

    def _transform(self, parent: str, child: str, stamp_ns: int, x=0.0, y=0.0, z=0.0, yaw=0.0) -> TransformStamped:
        message = TransformStamped()
        message.header.stamp = _stamp(stamp_ns)
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x = float(x)
        message.transform.translation.y = float(y)
        message.transform.translation.z = float(z)
        qx, qy, qz, qw = _quaternion(yaw)
        message.transform.rotation.x = qx
        message.transform.rotation.y = qy
        message.transform.rotation.z = qz
        message.transform.rotation.w = qw
        return message

    def _publish_static_tf(self) -> None:
        transforms = [
            self._transform("map", "odom", EPOCH_NS),
            self._transform("base_link", "base_scan", EPOCH_NS, z=0.18),
            self._transform("base_link", "imu_link", EPOCH_NS, z=0.25),
            self._transform("base_link", "camera_link", EPOCH_NS, x=0.20, z=0.35),
        ]
        self._pub["tf_static"].publish(TFMessage(transforms=transforms))

    def _publish_dynamic_tf(self, sample: RobotSample) -> None:
        transform = self._transform("odom", "base_link", sample.time_ns, sample.x, sample.y, yaw=sample.yaw)
        self._pub["tf"].publish(TFMessage(transforms=[transform]))

    def _pose(self, sample: RobotSample, frame: str = "map") -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = _stamp(sample.time_ns)
        pose.header.frame_id = frame
        pose.pose.position.x = sample.x
        pose.pose.position.y = sample.y
        qx, qy, qz, qw = _quaternion(sample.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _publish_odom(self, sample: RobotSample) -> None:
        message = Odometry()
        message.header.stamp = _stamp(sample.time_ns)
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose = self._pose(sample, "odom").pose
        message.twist.twist.linear.x = sample.speed_mps
        message.twist.twist.angular.z = sample.angular_speed_rps
        self._pub["odom"].publish(message)

    def _publish_cmd(self, sample: RobotSample) -> None:
        message = Twist()
        message.linear.x = sample.speed_mps
        message.angular.z = sample.angular_speed_rps
        self._pub["cmd"].publish(message)

    def _publish_scan(self, sample: RobotSample) -> None:
        message = LaserScan()
        message.header.stamp = _stamp(sample.time_ns)
        message.header.frame_id = "base_scan"
        message.angle_min = -math.pi
        message.angle_max = math.pi
        message.angle_increment = math.pi / 90.0
        message.range_min = 0.05
        message.range_max = 12.0
        message.ranges = [2.5 + 0.4 * math.sin(sample.yaw + i * message.angle_increment) for i in range(181)]
        self._pub["scan"].publish(message)

    def _publish_imu(self, sample: RobotSample) -> None:
        message = Imu()
        message.header.stamp = _stamp(sample.time_ns)
        message.header.frame_id = "imu_link"
        message.orientation_covariance[0] = 0.02
        message.orientation_covariance[4] = 0.02
        message.orientation_covariance[8] = 0.04
        qx, qy, qz, qw = _quaternion(sample.yaw)
        message.orientation.x = qx
        message.orientation.y = qy
        message.orientation.z = qz
        message.orientation.w = qw
        message.linear_acceleration.z = 9.81
        self._pub["imu"].publish(message)

    def _publish_image(self, sample: RobotSample) -> None:
        message = CompressedImage()
        message.header.stamp = _stamp(sample.time_ns)
        message.header.frame_id = "camera_link"
        message.format = "png"
        message.data = list(_PNG_1X1)
        self._pub["image"].publish(message)

    def _publish_plan_and_goal(self, sample: RobotSample) -> None:
        target_index = min(max(sample.waypoint_index - 1, 0), len(self.config.waypoints) - 1)
        target = self.config.waypoints[target_index]
        target_sample = RobotSample(sample.time_ns, target[0], target[1], sample.yaw, 0.0, 0.0, sample.phase, target_index, sample.diagnostic_level)
        goal = self._pose(target_sample)
        self._pub["goal"].publish(goal)
        path = Path()
        path.header.stamp = _stamp(sample.time_ns)
        path.header.frame_id = "map"
        for i in range(21):
            ratio = i / 20.0
            p = RobotSample(sample.time_ns, sample.x + (target[0] - sample.x) * ratio, sample.y + (target[1] - sample.y) * ratio, sample.yaw, 0.0, 0.0, sample.phase, target_index, sample.diagnostic_level)
            path.poses.append(self._pose(p))
        self._pub["plan"].publish(path)

    def _publish_trajectory(self, sample: RobotSample, elapsed_ns: int) -> None:
        path = Path()
        path.header.stamp = _stamp(sample.time_ns)
        path.header.frame_id = "map"
        path.poses = [self._pose(item) for item in self.scenario.trajectory_until(elapsed_ns)]
        self._pub["path"].publish(path)

    def _publish_status(self, sample: RobotSample) -> None:
        message = String()
        message.data = json.dumps(
            {
                "schema": "roamerx.patrol-status.v1",
                "stamp_ns": sample.time_ns,
                "phase": sample.phase,
                "waypoint_index": sample.waypoint_index,
                "waypoint_count": len(self.config.waypoints),
                "x": round(sample.x, 5),
                "y": round(sample.y, 5),
                "yaw": round(sample.yaw, 5),
                "speed_mps": round(sample.speed_mps, 5),
                "completed": sample.phase == "COMPLETED",
            },
            separators=(",", ":"),
        )
        self._pub["status"].publish(message)

    def _publish_battery(self, sample: RobotSample, elapsed_ns: int) -> None:
        message = BatteryState()
        message.header.stamp = _stamp(sample.time_ns)
        message.percentage = max(0.0, 0.90 - 0.04 * min(elapsed_ns / (self.config.duration_sec * 1_000_000_000), 1.0))
        message.voltage = 24.0
        message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self._pub["battery"].publish(message)

    def _publish_diagnostics(self, sample: RobotSample) -> None:
        status = DiagnosticStatus()
        status.name = "indoor_patrol"
        status.hardware_id = "roamerx-demo"
        # DiagnosticStatus.level is an IDL `byte`, which rclpy binds to a
        # length-1 bytes object rather than to int.
        status.level = bytes([sample.diagnostic_level])
        status.message = "obstacle avoidance" if sample.diagnostic_level else "OK"
        status.values = [KeyValue(key="phase", value=sample.phase), KeyValue(key="frame", value="map->odom->base_link")]
        message = DiagnosticArray()
        message.header.stamp = _stamp(sample.time_ns)
        message.status = [status]
        self._pub["diagnostics"].publish(message)

    def _publish_clock(self, sample: RobotSample) -> None:
        message = Clock()
        message.clock = _stamp(sample.time_ns)
        self._pub["clock"].publish(message)

    def _due(self, name: str, elapsed_ns: int) -> bool:
        if elapsed_ns < self._next[name]:
            return False
        self._next[name] += self._period_ns[name]
        # Avoid a burst after a blocked callback; one latest sample is more
        # useful for visualization than replaying stale demo frames.
        if self._next[name] < elapsed_ns - 2 * self._period_ns[name]:
            self._next[name] = elapsed_ns + self._period_ns[name]
        return True

    def _tick(self) -> None:
        elapsed_ns = int(max(0.0, time.monotonic() - self._start_monotonic) * 1_000_000_000)
        elapsed_ns = min(elapsed_ns, int(self.config.duration_sec * 1_000_000_000))
        if elapsed_ns < self._last_elapsed_ns:
            return
        self._last_elapsed_ns = elapsed_ns
        sample = self.scenario.sample(elapsed_ns)
        self._publish_clock(sample)
        if self._due("tf", elapsed_ns):
            self._publish_dynamic_tf(sample)
        if self._due("odom", elapsed_ns):
            self._publish_odom(sample)
        if self._due("cmd", elapsed_ns):
            self._publish_cmd(sample)
        if self.config.publish_scan and self._due("scan", elapsed_ns):
            self._publish_scan(sample)
        if self.config.publish_imu and self._due("imu", elapsed_ns):
            self._publish_imu(sample)
        if self.config.publish_camera and self._due("image", elapsed_ns):
            self._publish_image(sample)
        if self._due("plan", elapsed_ns):
            self._publish_plan_and_goal(sample)
        if self._due("path", elapsed_ns):
            self._publish_trajectory(sample, elapsed_ns)
        if self._due("status", elapsed_ns):
            self._publish_status(sample)
        if self._due("battery", elapsed_ns):
            self._publish_battery(sample, elapsed_ns)
        if self._due("diagnostics", elapsed_ns):
            self._publish_diagnostics(sample)
        if sample.phase == "COMPLETED" and not self._finished:
            # Publish the terminal state once and allow DDS to flush briefly;
            # ros2 bag record receives the final message before shutdown.
            self._publish_status(sample)
            self._publish_trajectory(sample, elapsed_ns)
            self._finished = True
            self.get_logger().info("indoor patrol demo completed")
            self._timer.cancel()
            self.create_timer(0.2, self._shutdown_once)

    def _shutdown_once(self) -> None:
        # Resolving a future rather than destroying the node here: this runs
        # inside a timer callback of this very node, and tearing the node down
        # from within its own executor callback leaves the executor spinning
        # forever instead of returning from rclpy.spin.
        if not self._done.done():
            self._done.set_result(True)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PatrolPublisher()
        rclpy.spin_until_future_complete(node, node.done_future)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f"roamerx_patrol_demo: invalid configuration: {exc}")
        raise SystemExit(2)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
