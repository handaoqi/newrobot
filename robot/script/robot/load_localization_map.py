#!/usr/bin/env python3
"""Load the active PCD through a live ROS 2 service client.

The generic ``ros2 service call`` command may consult the ros2cli daemon.  A
stale graph entry can therefore make startup believe that an old localization
service is available.  This helper creates a fresh node, waits for the live
service, and validates the structured response before reporting success.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from robots_dog_msgs.srv import LoadMap


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd_path")
    parser.add_argument("--timeout", type=float, default=25.0)
    args = parser.parse_args()

    timeout = max(0.1, float(args.timeout))
    deadline = time.monotonic() + timeout
    rclpy.init(args=None)
    node = rclpy.create_node("roamerx_load_localization_map")
    client = node.create_client(LoadMap, "/load_map_service")
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            print(
                f"ERROR: /load_map_service was not available within {timeout:.1f}s",
                file=sys.stderr,
            )
            return 1

        request = LoadMap.Request()
        request.pcd_path = args.pcd_path
        future = client.call_async(request)
        remaining = max(0.1, deadline - time.monotonic())
        rclpy.spin_until_future_complete(node, future, timeout_sec=remaining)
        if not future.done():
            print(
                f"ERROR: /load_map_service did not respond within {timeout:.1f}s",
                file=sys.stderr,
            )
            return 1

        response = future.result()
        if response is None:
            print("ERROR: /load_map_service returned no response", file=sys.stderr)
            return 1
        if not response.success:
            print(
                f"ERROR: localization rejected PCD map: {response.message}",
                file=sys.stderr,
            )
            return 1
        print(f"Localization PCD loaded: {args.pcd_path} ({response.message})")
        return 0
    except Exception as exc:
        print(f"ERROR: localization map load failed: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_client(client)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
