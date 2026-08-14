#!/usr/bin/env python3
"""Republish only changed controller odometry with ROS reception timestamps."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomStampSanitizer(Node):
    def __init__(self) -> None:
        super().__init__('odom_stamp_sanitizer')
        self._last_pose = None
        self._publisher = self.create_publisher(Odometry, '/odom/mc_odom_sanitized', 20)
        self.create_subscription(Odometry, '/odom/mc_odom', self._callback, 100)

    def _callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        current = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        )
        if self._last_pose is not None:
            distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(current[:3], self._last_pose[:3])))
            orientation_delta = abs(sum(a * b for a, b in zip(current[3:], self._last_pose[3:])))
            if distance < 1e-4 and orientation_delta > 0.999999:
                return
        self._last_pose = current
        message.header.stamp = self.get_clock().now().to_msg()
        if message.pose.covariance[0] <= 0.0:
            message.pose.covariance[0] = 0.05
        if message.pose.covariance[7] <= 0.0:
            message.pose.covariance[7] = 0.05
        if message.pose.covariance[35] <= 0.0:
            message.pose.covariance[35] = 0.05
        self._publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = OdomStampSanitizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
