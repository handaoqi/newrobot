#!/usr/bin/env python3
"""Read-only post-save localization smoke test; never publishes motion commands."""
import argparse
import math
import time

import rclpy
from robots_dog_msgs.msg import Localization
from localization.msg import ScanMatchingStatus


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--min-normal", type=int, default=3)
    parser.add_argument("--mapping-type", choices=("indoor", "outdoor"), default="indoor")
    args = parser.parse_args()
    rclpy.init(args=None)
    node = rclpy.create_node("post_save_static_localization_check")
    latest = {"loc": None, "match": None}
    node.create_subscription(Localization, "/localization_info", lambda m: latest.__setitem__("loc", m), 10)
    node.create_subscription(ScanMatchingStatus, "/localization/scan_matching_status", lambda m: latest.__setitem__("match", m), 10)
    normal = 0
    match_ok = False
    deadline = time.monotonic() + max(1.0, args.timeout)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            loc, match = latest["loc"], latest["match"]
            outdoor_ready = args.mapping_type != "outdoor" or int(getattr(loc, "coord_type", 0)) == 1
            if loc is not None and outdoor_ready and int(loc.status) == 3 and float(getattr(loc, "speed", 0.0)) <= 0.15:
                normal += 1
            else:
                normal = 0
            if match is not None and bool(getattr(match, "has_converged", False)) and float(getattr(match, "matching_error", 999.0)) < 0.50 and float(getattr(match, "inlier_fraction", 0.0)) >= 0.05:
                match_ok = True
            if normal >= args.min_normal and match_ok:
                print("PASS normal_samples=%d match=1" % normal)
                return 0
        print("FAIL normal_samples=%d match=%d" % (normal, int(match_ok)))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
