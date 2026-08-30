#!/usr/bin/env python3
"""Motion-free post-save localization check; never publishes velocity commands."""
import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robots_dog_msgs.msg import Localization
from localization.msg import ScanMatchingStatus


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_yaw(pose: PoseStamped) -> float:
    orientation = pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def read_saved_terminal_pose(map_dir: str) -> dict | None:
    if not map_dir:
        return None
    trajectory = Path(map_dir) / "map.txt"
    if not trajectory.is_file():
        return None
    latest = None
    try:
        lines = trajectory.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return None
    for line in lines:
        fields = line.split()
        if len(fields) < 3 or line.lstrip().startswith("#"):
            continue
        try:
            latest = {"x": float(fields[0]), "y": float(fields[1]), "yaw_rad": float(fields[2])}
        except ValueError:
            continue
    if latest is not None:
        latest["yaw_deg"] = math.degrees(latest["yaw_rad"])
    return latest


def read_map_context(map_dir: str) -> dict:
    manifest = Path(map_dir) / "map_manifest.json" if map_dir else None
    if manifest is None or not manifest.is_file():
        return {}
    try:
        payload = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return {
        "coordinate_mode": str(payload.get("coordinate_mode") or ""),
        "scene_scope": str(payload.get("scene_scope") or ""),
    }


def output_result(*, state: str, reason_code: str, message: str, args, latest: dict,
                  normal_samples: int, expected_pose: dict | None,
                  diagnostics: dict) -> int:
    loc = latest.get("loc")
    match = latest.get("match")
    pose_message = latest.get("pose")
    pose = None
    position_error_m = None
    yaw_error_deg = None
    if pose_message is not None:
        yaw_rad = quaternion_yaw(pose_message)
        pose = {
            "x": float(pose_message.pose.position.x),
            "y": float(pose_message.pose.position.y),
            "z": float(pose_message.pose.position.z),
            "yaw_rad": yaw_rad,
            "yaw_deg": math.degrees(yaw_rad),
        }
        if expected_pose is not None:
            position_error_m = math.hypot(
                pose["x"] - expected_pose["x"],
                pose["y"] - expected_pose["y"],
            )
            yaw_error_deg = abs(math.degrees(normalize_angle(
                pose["yaw_rad"] - expected_pose["yaw_rad"],
            )))
    payload = {
        "schema": "roamerx.post-save-localization-check.v1",
        "state": state,
        "accurate": state == "passed",
        "reason_code": reason_code,
        "message": message,
        "mapping_type": args.mapping_type,
        "map_dir": str(Path(args.map_dir).resolve()) if args.map_dir else "",
        "frame_id": "map",
        **read_map_context(args.map_dir),
        "sampled_at": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
        "pose": pose,
        "saved_terminal_pose": expected_pose,
        "position_error_m": position_error_m,
        "yaw_error_deg": yaw_error_deg,
        "thresholds": {
            "max_position_error_m": args.max_position_error,
            "max_yaw_error_deg": args.max_yaw_error_deg,
            "max_matching_error": args.max_matching_error,
            "min_inlier_fraction": args.min_inlier_fraction,
            "max_static_speed_mps": args.max_static_speed,
            "max_pose_step_m": args.max_pose_step,
            "max_yaw_step_deg": args.max_yaw_step_deg,
            "required_normal_samples": args.min_normal,
            "required_pose_samples": args.min_pose_samples,
        },
        "localization": {
            "status": int(loc.status) if loc is not None else None,
            "coord_type": int(loc.coord_type) if loc is not None else None,
            "speed_mps": float(getattr(loc, "speed", 0.0)) if loc is not None else None,
            "normal_samples": normal_samples,
        },
        "quality": {
            "has_converged": bool(getattr(match, "has_converged", False)) if match is not None else False,
            "matching_error": float(getattr(match, "matching_error", math.inf)) if match is not None else None,
            "inlier_fraction": float(getattr(match, "inlier_fraction", 0.0)) if match is not None else None,
        },
        "diagnostics": diagnostics,
    }
    print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    return 0 if state == "passed" else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--min-normal", type=int, default=3)
    parser.add_argument("--mapping-type", choices=("indoor", "outdoor"), default="indoor")
    parser.add_argument("--map-dir", default="")
    parser.add_argument("--max-position-error", type=float, default=0.50)
    parser.add_argument("--max-yaw-error-deg", type=float, default=10.0)
    parser.add_argument("--max-matching-error", type=float, default=0.50)
    parser.add_argument("--min-inlier-fraction", type=float, default=0.05)
    parser.add_argument("--max-static-speed", type=float, default=0.15)
    parser.add_argument("--min-pose-samples", type=int, default=3)
    parser.add_argument("--max-pose-step", type=float, default=0.15)
    parser.add_argument("--max-yaw-step-deg", type=float, default=3.0)
    parser.add_argument("--initial-pose-subscriber-timeout", type=float, default=10.0)
    args = parser.parse_args()
    rclpy.init(args=None)
    node = rclpy.create_node("post_save_static_localization_check")
    latest = {
        "loc": None,
        "match": None,
        "pose": None,
        "loc_sequence": 0,
        "match_sequence": 0,
        "pose_sequence": 0,
    }
    expected_pose = read_saved_terminal_pose(args.map_dir)

    def update_latest(key: str, message) -> None:
        latest[key] = message
        latest[f"{key}_sequence"] += 1

    node.create_subscription(Localization, "/localization_info", lambda m: update_latest("loc", m), 10)
    # The deployed localization configuration publishes ScanMatchingStatus on
    # /status (the pose topic is /localization/scan_match_pose).
    node.create_subscription(ScanMatchingStatus, "/status", lambda m: update_latest("match", m), 10)
    node.create_subscription(PoseStamped, "/localization/scan_match_pose", lambda m: update_latest("pose", m), 10)
    initial_pose_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 8)
    normal = 0
    last_loc_sequence = 0
    last_match_sequence = 0
    last_pose_sequence = 0
    loc_samples = 0
    match_samples = 0
    pose_samples = 0
    last_pose = None
    max_pose_step_m = 0.0
    max_yaw_step_deg = 0.0
    seed_published = False
    seed_source = "rtk_map_initialization" if args.mapping_type == "outdoor" else "saved_terminal_pose"
    deadline = time.monotonic() + max(1.0, args.timeout)
    try:
        if expected_pose is None:
            return output_result(
                state="failed", reason_code="SAVED_TERMINAL_POSE_MISSING",
                message="地图缺少保存终点，无法核对定位位置", args=args,
                latest=latest, normal_samples=normal, expected_pose=expected_pose,
                diagnostics={"seed_source": seed_source, "seed_published": False},
            )
        map_context = read_map_context(args.map_dir)
        if args.mapping_type == "outdoor" and map_context.get("coordinate_mode") != "global_enu":
            return output_result(
                state="failed", reason_code="OUTDOOR_MAP_CONTEXT_INVALID",
                message="室外地图缺少已锁定的全局 ENU 坐标上下文", args=args,
                latest=latest, normal_samples=normal, expected_pose=expected_pose,
                diagnostics={"seed_source": seed_source, "seed_published": False},
            )

        # Indoor validation deliberately seeds the new localization process at
        # the saved terminal pose. Outdoor maps are seeded by LoadMapCallBack's
        # GNSS/ENU initialization and must not be allowed to bypass that path.
        if args.mapping_type == "indoor":
            subscriber_deadline = min(
                deadline,
                time.monotonic() + max(0.1, args.initial_pose_subscriber_timeout),
            )
            while initial_pose_pub.get_subscription_count() == 0 and time.monotonic() < subscriber_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if initial_pose_pub.get_subscription_count() == 0:
                return output_result(
                    state="failed", reason_code="INITIAL_POSE_SUBSCRIBER_MISSING",
                    message="本次地图定位栈未就绪：/initialpose 无订阅者", args=args,
                    latest=latest, normal_samples=normal, expected_pose=expected_pose,
                    diagnostics={"seed_source": seed_source, "seed_published": False},
                )
            seed = PoseWithCovarianceStamped()
            seed.header.frame_id = "map"
            seed.pose.pose.position.x = expected_pose["x"]
            seed.pose.pose.position.y = expected_pose["y"]
            seed.pose.pose.orientation.z = math.sin(expected_pose["yaw_rad"] / 2.0)
            seed.pose.pose.orientation.w = math.cos(expected_pose["yaw_rad"] / 2.0)
            seed.pose.covariance[0] = 0.25
            seed.pose.covariance[7] = 0.25
            seed.pose.covariance[35] = 0.0685
            initial_pose_pub.publish(seed)
            seed_published = True

        # Ignore everything received before the candidate-map seed boundary.
        last_loc_sequence = latest["loc_sequence"]
        last_match_sequence = latest["match_sequence"]
        last_pose_sequence = latest["pose_sequence"]
        latest.update({"loc": None, "match": None, "pose": None})

        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            loc, match = latest["loc"], latest["match"]
            # The deployed Localization message reports the local map pose for
            # both indoor and ENU-aligned outdoor maps. Outdoor identity comes
            # from map_manifest.json; coord_type must not make outdoor checks
            # fail before the message implementation exposes geodetic fields.
            if latest["loc_sequence"] > last_loc_sequence and loc is not None:
                last_loc_sequence = latest["loc_sequence"]
                loc_samples += 1
                if int(loc.status) == 3 and float(getattr(loc, "speed", 0.0)) <= args.max_static_speed:
                    normal += 1
                else:
                    normal = 0
            if latest["match_sequence"] > last_match_sequence and match is not None:
                match_samples += latest["match_sequence"] - last_match_sequence
                last_match_sequence = latest["match_sequence"]
            if latest["pose_sequence"] > last_pose_sequence and latest["pose"] is not None:
                pose_samples += latest["pose_sequence"] - last_pose_sequence
                last_pose_sequence = latest["pose_sequence"]
                current_pose = latest["pose"]
                if last_pose is not None:
                    pose_step = math.hypot(
                        float(current_pose.pose.position.x) - float(last_pose.pose.position.x),
                        float(current_pose.pose.position.y) - float(last_pose.pose.position.y),
                    )
                    yaw_step = abs(math.degrees(normalize_angle(
                        quaternion_yaw(current_pose) - quaternion_yaw(last_pose),
                    )))
                    max_pose_step_m = max(max_pose_step_m, pose_step)
                    max_yaw_step_deg = max(max_yaw_step_deg, yaw_step)
                last_pose = current_pose
            match_ok = bool(
                match is not None
                and getattr(match, "has_converged", False)
                and float(getattr(match, "matching_error", math.inf)) < args.max_matching_error
                and float(getattr(match, "inlier_fraction", 0.0)) >= args.min_inlier_fraction
            )
            pose_message = latest["pose"]
            if (
                normal >= args.min_normal
                and match_samples > 0
                and match_ok
                and pose_message is not None
                and pose_samples >= args.min_pose_samples
            ):
                yaw_rad = quaternion_yaw(pose_message)
                position_error = math.hypot(
                    float(pose_message.pose.position.x) - expected_pose["x"],
                    float(pose_message.pose.position.y) - expected_pose["y"],
                )
                yaw_error = abs(math.degrees(normalize_angle(yaw_rad - expected_pose["yaw_rad"])))
                pose_stable = (
                    max_pose_step_m <= args.max_pose_step
                    and max_yaw_step_deg <= args.max_yaw_step_deg
                )
                if (
                    position_error <= args.max_position_error
                    and yaw_error <= args.max_yaw_error_deg
                    and pose_stable
                ):
                    return output_result(
                        state="passed", reason_code="LOCALIZATION_ACCURATE",
                        message="静止定位准确", args=args, latest=latest,
                        normal_samples=normal, expected_pose=expected_pose,
                        diagnostics={
                            "seed_source": seed_source,
                            "seed_published": seed_published,
                            "fresh_localization_samples": loc_samples,
                            "fresh_scan_match_samples": match_samples,
                            "fresh_pose_samples": pose_samples,
                            "max_pose_step_m": max_pose_step_m,
                            "max_yaw_step_deg": max_yaw_step_deg,
                        },
                    )
        diagnostics = {
            "seed_source": seed_source,
            "seed_published": seed_published,
            "fresh_localization_samples": loc_samples,
            "fresh_scan_match_samples": match_samples,
            "fresh_pose_samples": pose_samples,
            "max_pose_step_m": max_pose_step_m,
            "max_yaw_step_deg": max_yaw_step_deg,
        }
        if latest["pose"] is None:
            reason_code, message = "LOCALIZATION_POSE_MISSING", "未收到本次地图的定位姿态"
        elif latest["loc"] is None:
            reason_code, message = "LOCALIZATION_STATUS_MISSING", "未收到本次地图的新定位状态"
        elif float(getattr(latest["loc"], "speed", 0.0)) > args.max_static_speed:
            reason_code, message = "ROBOT_NOT_STATIONARY", "机器人速度未达到静止门槛"
        elif int(getattr(latest["loc"], "status", 0)) != 3 or normal < args.min_normal:
            reason_code, message = "LOCALIZATION_NOT_NORMAL", "定位状态未达到静止且正常"
        elif latest["match"] is None or not bool(getattr(latest["match"], "has_converged", False)):
            reason_code, message = "SCAN_MATCH_NOT_CONVERGED", "点云匹配未收敛"
        elif (
            float(getattr(latest["match"], "matching_error", math.inf)) >= args.max_matching_error
            or float(getattr(latest["match"], "inlier_fraction", 0.0)) < args.min_inlier_fraction
        ):
            reason_code, message = "SCAN_MATCH_QUALITY_LOW", "点云匹配质量未达到阈值"
        elif max_pose_step_m > args.max_pose_step or max_yaw_step_deg > args.max_yaw_step_deg:
            reason_code, message = "LOCALIZATION_POSE_UNSTABLE", "静止定位相邻输出跳变超过阈值"
        else:
            reason_code, message = "LOCALIZATION_POSITION_MISMATCH", "定位位置或航向偏差超过阈值"
        return output_result(
            state="failed", reason_code=reason_code, message=message,
            args=args, latest=latest, normal_samples=normal,
            expected_pose=expected_pose,
            diagnostics=diagnostics,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
