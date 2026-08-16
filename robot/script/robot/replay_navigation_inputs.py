#!/usr/bin/env python3
"""Replay navigation sensor inputs with live-equivalent ROS timestamps."""

import argparse
import time

import rclpy
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2


def reliable_qos(depth):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--start-offset", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=None)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("navigation_input_replayer")
    publishers = {
        "/front_lidar": (node.create_publisher(PointCloud2, "/front_lidar", reliable_qos(10)), PointCloud2),
        "/front_lidar/imu": (node.create_publisher(Imu, "/front_lidar/imu", reliable_qos(100)), Imu),
        "/odom/mc_odom": (node.create_publisher(Odometry, "/odom/mc_odom", reliable_qos(100)), Odometry),
    }

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    first_ns = None
    bag_start_ns = None
    wall_start = None
    count = 0
    while rclpy.ok() and reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if bag_start_ns is None:
            bag_start_ns = stamp_ns
        bag_elapsed = (stamp_ns - bag_start_ns) / 1e9
        if bag_elapsed < args.start_offset:
            continue
        if args.duration is not None and bag_elapsed > args.start_offset + args.duration:
            break
        if topic not in publishers:
            continue
        if first_ns is None:
            first_ns = stamp_ns
            wall_start = time.monotonic()
        delay = wall_start + (stamp_ns - first_ns) / 1e9 / args.rate - time.monotonic()
        if delay > 0:
            time.sleep(delay)

        publisher, message_type = publishers[topic]
        message = deserialize_message(data, message_type)
        now_ns = node.get_clock().now().nanoseconds
        if topic in ("/front_lidar", "/front_lidar/imu"):
            original_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            now_ns += original_ns - stamp_ns
        message.header.stamp.sec = now_ns // 1_000_000_000
        message.header.stamp.nanosec = now_ns % 1_000_000_000
        publisher.publish(message)
        count += 1
        if count % 10000 == 0:
            print(f"published={count} elapsed={(stamp_ns-first_ns)/1e9:.1f}s", flush=True)

    print(f"complete published={count}", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
