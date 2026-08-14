#!/usr/bin/env python3
"""Provide explicit IMU covariance for robot_localization without changing data."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovarianceSanitizer(Node):
    def __init__(self) -> None:
        super().__init__('imu_covariance_sanitizer')
        self._publisher = self.create_publisher(Imu, '/front_lidar/imu_sanitized', 100)
        self.create_subscription(Imu, '/front_lidar/imu', self._callback, 100)

    def _callback(self, message: Imu) -> None:
        # Livox leaves covariance entries at zero. The official UKF treats zero
        # as invalid certainty; only yaw and yaw rate are consumed in 2D mode.
        if message.orientation_covariance[8] <= 0.0:
            message.orientation_covariance[8] = 0.03
        if message.angular_velocity_covariance[8] <= 0.0:
            message.angular_velocity_covariance[8] = 0.02
        self._publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = ImuCovarianceSanitizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
