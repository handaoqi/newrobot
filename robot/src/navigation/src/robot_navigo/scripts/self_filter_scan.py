#!/usr/bin/env python3
"""Filter fixed robot-body reflections from a base_link LaserScan."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SelfFilterScan(Node):
    """Remove only the calibrated chassis/foreleg reflection rectangle."""

    def __init__(self) -> None:
        super().__init__("self_filter_scan")
        self.declare_parameter("self_x_min", 0.10)
        self.declare_parameter("self_x_max", 0.46)
        self.declare_parameter("self_y_min", -0.19)
        self.declare_parameter("self_y_max", 0.10)
        self._x_min = float(self.get_parameter("self_x_min").value)
        self._x_max = float(self.get_parameter("self_x_max").value)
        self._y_min = float(self.get_parameter("self_y_min").value)
        self._y_max = float(self.get_parameter("self_y_max").value)
        self._publisher = self.create_publisher(
            LaserScan, "scan_out", qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "scan_in", self._on_scan, qos_profile_sensor_data
        )
        self._warned_frame = False
        self.get_logger().info(
            f"self filter active: {self._x_min:.2f}<=x<={self._x_max:.2f}, "
            f"{self._y_min:.2f}<=y<={self._y_max:.2f} in base_link"
        )

    def _on_scan(self, scan: LaserScan) -> None:
        if scan.header.frame_id != "base_link":
            if not self._warned_frame:
                self.get_logger().warning(
                    "expected base_link scan, received %s; self filtering disabled",
                    scan.header.frame_id,
                )
                self._warned_frame = True
            self._publisher.publish(scan)
            return

        filtered = LaserScan()
        filtered.header = scan.header
        filtered.angle_min = scan.angle_min
        filtered.angle_max = scan.angle_max
        filtered.angle_increment = scan.angle_increment
        filtered.time_increment = scan.time_increment
        filtered.scan_time = scan.scan_time
        filtered.range_min = scan.range_min
        filtered.range_max = scan.range_max
        filtered.ranges = list(scan.ranges)
        filtered.intensities = list(scan.intensities)

        for index, distance in enumerate(filtered.ranges):
            if not math.isfinite(distance):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            if self._x_min <= x <= self._x_max and self._y_min <= y <= self._y_max:
                filtered.ranges[index] = math.inf
        self._publisher.publish(filtered)


def main() -> None:
    rclpy.init()
    node = SelfFilterScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
