#!/usr/bin/env python3
"""Filter fixed robot-body reflections from a base_link LaserScan."""

from __future__ import annotations

import math
import time

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
        self.declare_parameter("performance_log_interval_seconds", 10.0)
        self._x_min = float(self.get_parameter("self_x_min").value)
        self._x_max = float(self.get_parameter("self_x_max").value)
        self._y_min = float(self.get_parameter("self_y_min").value)
        self._y_max = float(self.get_parameter("self_y_max").value)
        self._performance_log_interval = max(
            1.0, float(self.get_parameter("performance_log_interval_seconds").value)
        )
        self._angle_key: tuple[int, float, float] | None = None
        self._cosines: list[float] = []
        self._sines: list[float] = []
        self._window_started_at = time.perf_counter()
        self._window_scans = 0
        self._window_filtered_points = 0
        self._window_processing_seconds = 0.0
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
        processing_started_at = time.perf_counter()
        if scan.header.frame_id != "base_link":
            if not self._warned_frame:
                self.get_logger().warning(
                    "expected base_link scan, received %s; self filtering disabled",
                    scan.header.frame_id,
                )
                self._warned_frame = True
            self._publisher.publish(scan)
            return

        angle_key = (len(scan.ranges), scan.angle_min, scan.angle_increment)
        if angle_key != self._angle_key:
            self._angle_key = angle_key
            self._cosines = [
                math.cos(scan.angle_min + index * scan.angle_increment)
                for index in range(len(scan.ranges))
            ]
            self._sines = [
                math.sin(scan.angle_min + index * scan.angle_increment)
                for index in range(len(scan.ranges))
            ]

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

        filtered_points = 0
        for index, distance in enumerate(filtered.ranges):
            if not math.isfinite(distance):
                continue
            x = distance * self._cosines[index]
            y = distance * self._sines[index]
            if self._x_min <= x <= self._x_max and self._y_min <= y <= self._y_max:
                filtered.ranges[index] = math.inf
                filtered_points += 1
        self._publisher.publish(filtered)
        self._window_scans += 1
        self._window_filtered_points += filtered_points
        self._window_processing_seconds += time.perf_counter() - processing_started_at
        now = time.perf_counter()
        elapsed = now - self._window_started_at
        if elapsed >= self._performance_log_interval:
            self.get_logger().info(
                "self filter perf scans=%d rate=%.2fHz avg_ms=%.3f filtered_points=%d"
                % (
                    self._window_scans,
                    self._window_scans / elapsed,
                    1000.0 * self._window_processing_seconds / max(self._window_scans, 1),
                    self._window_filtered_points,
                )
            )
            self._window_started_at = now
            self._window_scans = 0
            self._window_filtered_points = 0
            self._window_processing_seconds = 0.0


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
