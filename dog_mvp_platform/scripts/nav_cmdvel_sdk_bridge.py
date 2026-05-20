#!/usr/bin/env python3
import argparse
import json
import math
import signal
import time
import urllib.request
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist


class CmdVelSdkBridge:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("dog_mvp_cmdvel_sdk_bridge")
        self.last_msg_at = 0.0
        self.last_send_at = 0.0
        self.last_stop_at = 0.0
        self.last_error = ""
        self.last_cmd = {"linear": 0.0, "angular": 0.0}
        self.status_path = Path(args.status_file)
        self.running = True
        self.node.create_subscription(Twist, args.cmd_topic, self.on_cmd_vel, 10)

    def post(self, path, payload=None, timeout=1.2):
        data = json.dumps(payload or {}).encode("utf-8")
        req = urllib.request.Request(
            self.args.sdk_bridge + path,
            data=data,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))

    def clamp(self, value, low, high):
        return max(low, min(float(value), high))

    def write_status(self, active):
        status = {
            "ok": True,
            "active": active,
            "cmd_topic": self.args.cmd_topic,
            "last_msg_age": round(time.time() - self.last_msg_at, 3) if self.last_msg_at else None,
            "last_cmd": self.last_cmd,
            "last_error": self.last_error,
            "time": time.time(),
        }
        tmp = self.status_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(status, ensure_ascii=False), encoding="utf-8")
        tmp.replace(self.status_path)

    def on_cmd_vel(self, msg):
        now = time.time()
        if now - self.last_send_at < self.args.period:
            return

        linear = self.clamp(msg.linear.x, -self.args.max_linear, self.args.max_linear)
        angular = self.clamp(msg.angular.z, -self.args.max_angular, self.args.max_angular)
        if abs(linear) < self.args.deadband_linear:
            linear = 0.0
        if abs(angular) < self.args.deadband_angular:
            angular = 0.0

        self.last_msg_at = now
        self.last_send_at = now
        self.last_cmd = {"linear": round(linear, 4), "angular": round(angular, 4)}

        try:
            self.post(
                "/move",
                {
                    "linear": linear,
                    "angular": angular,
                    "duration": self.args.duration,
                },
            )
            self.last_error = ""
        except Exception as exc:
            self.last_error = str(exc)
        self.write_status(active=bool(linear or angular))

    def stop_sdk(self):
        try:
            self.post("/stop", {}, timeout=1.0)
        except Exception as exc:
            self.last_error = str(exc)

    def run(self):
        signal.signal(signal.SIGTERM, lambda *_: setattr(self, "running", False))
        signal.signal(signal.SIGINT, lambda *_: setattr(self, "running", False))

        self.post("/stand", {}, timeout=2.0)
        self.write_status(active=False)

        while rclpy.ok() and self.running:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            now = time.time()
            if self.last_msg_at and now - self.last_msg_at > self.args.cmd_timeout:
                if now - self.last_stop_at > 0.4:
                    self.stop_sdk()
                    self.last_stop_at = now
                    self.last_cmd = {"linear": 0.0, "angular": 0.0}
                    self.write_status(active=False)
            elif not self.last_msg_at and now - self.last_stop_at > 1.0:
                self.stop_sdk()
                self.last_stop_at = now
                self.write_status(active=False)

        self.stop_sdk()
        self.write_status(active=False)
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-bridge", default="http://192.168.234.1:9095")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--status-file", default="/tmp/dog_nav_cmdvel_sdk_bridge.status.json")
    parser.add_argument("--period", type=float, default=0.10)
    parser.add_argument("--duration", type=float, default=0.35)
    parser.add_argument("--cmd-timeout", type=float, default=0.65)
    parser.add_argument("--max-linear", type=float, default=0.20)
    parser.add_argument("--max-angular", type=float, default=0.55)
    parser.add_argument("--deadband-linear", type=float, default=0.015)
    parser.add_argument("--deadband-angular", type=float, default=0.02)
    args = parser.parse_args()

    rclpy.init()
    bridge = CmdVelSdkBridge(args)
    try:
        bridge.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
