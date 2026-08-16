#!/usr/bin/env python3
"""Normalize custom NDT odometry for robot_localization observation input."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class NdtOdomSanitizer(Node):
    def __init__(self) -> None:
        super().__init__('ndt_odom_sanitizer')
        self._publisher = self.create_publisher(Odometry, '/odom/ndt_odom_sanitized', 20)
        self.create_subscription(Odometry, '/odom/localization_odom', self._callback, 20)

    def _callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return
        if pose.orientation.w * pose.orientation.w + pose.orientation.x * pose.orientation.x + \
                pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z < 0.5:
            return

        # The custom localizer encodes confidence in covariance[0] and may leave
        # the standard covariance entries at zero. Provide explicit conservative
        # NDT measurement uncertainty for the official global UKF.
        confidence = min(1.0, max(0.0, message.pose.covariance[0]))
        xy_variance = 0.05 + (1.0 - confidence) * 0.45
        yaw_variance = 0.03 + (1.0 - confidence) * 0.20
        message.pose.covariance = [0.0] * 36
        message.pose.covariance[0] = xy_variance
        message.pose.covariance[7] = xy_variance
        message.pose.covariance[14] = 1000.0
        message.pose.covariance[21] = 1000.0
        message.pose.covariance[28] = 1000.0
        message.pose.covariance[35] = yaw_variance
        message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = NdtOdomSanitizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
