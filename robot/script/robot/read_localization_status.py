#!/usr/bin/env python3
"""Read one localization status without using the ROS 2 CLI daemon."""

from __future__ import annotations

import argparse
import time

import rclpy
from robots_dog_msgs.msg import Localization


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init(args=None)
    node = rclpy.create_node("read_localization_status")
    messages: list[Localization] = []
    subscription = node.create_subscription(Localization, "/localization_info", messages.append, 10)
    deadline = time.monotonic() + max(0.1, args.timeout)
    try:
        while not messages and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, deadline - time.monotonic()))
        if not messages:
            return 1
        print(messages[-1].status)
        return 0
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
