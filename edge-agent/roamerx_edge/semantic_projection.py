"""Calibrated LiDAR-camera projection used by the read-only scene viewer.

The runtime adapter can publish this result as robots_dog_msgs/SemanticObjectArray.
Keeping the geometry pure makes the safety-critical rejection gates testable
without ROS, a camera, or a running robot.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


MIN_SUPPORT_POINTS = 8
MAX_SYNC_DELTA_MS = 100.0
MAX_RANGE_M = 20.0
MAX_POSITION_STD_M = 0.5


class ProjectionRejected(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class CameraCalibration:
    calibration_id: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    lidar_to_camera: tuple[tuple[float, float, float, float], ...]
    distortion: tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)

    def matrix(self) -> np.ndarray:
        matrix = np.asarray(self.lidar_to_camera, dtype=np.float64)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ProjectionRejected("calibration_invalid", "LiDAR到相机外参必须为有限4x4矩阵")
        if min(self.fx, self.fy) <= 0 or min(self.width, self.height) <= 0:
            raise ProjectionRejected("calibration_invalid", "相机内参或图像尺寸无效")
        return matrix


def validate_sync(image_stamp: float, cloud_stamp: float, maximum_ms: float = MAX_SYNC_DELTA_MS) -> float:
    delta_ms = abs(float(image_stamp) - float(cloud_stamp)) * 1000.0
    if not np.isfinite(delta_ms) or delta_ms > maximum_ms:
        raise ProjectionRejected("time_sync_exceeded", f"图像与点云时间差 {delta_ms:.1f}ms 超过 {maximum_ms:.1f}ms")
    return delta_ms


def _project_pixels(camera_points: np.ndarray, calibration: CameraCalibration) -> tuple[np.ndarray, np.ndarray]:
    z = camera_points[:, 2]
    x = camera_points[:, 0] / z
    y = camera_points[:, 1] / z
    k1, k2, p1, p2, k3 = calibration.distortion
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    distorted_x = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    distorted_y = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return calibration.fx * distorted_x + calibration.cx, calibration.fy * distorted_y + calibration.cy


def _foreground_cluster(points: np.ndarray) -> np.ndarray:
    depths = points[:, 2]
    lower, upper = float(np.min(depths)), float(np.max(depths))
    if upper - lower < 0.5:
        return points
    edges = np.arange(lower, upper + 0.5, 0.5)
    counts, edges = np.histogram(depths, bins=max(1, len(edges) - 1))
    # Prefer the nearest well-supported peak, not a distant wall filling the box.
    candidates = np.flatnonzero(counts >= MIN_SUPPORT_POINTS)
    if not len(candidates):
        return points
    index = int(candidates[0])
    center = (edges[index] + edges[index + 1]) / 2
    return points[np.abs(depths - center) <= 0.45]


def project_detection(
    points_lidar: Iterable[Iterable[float]],
    bbox: dict,
    calibration: CameraCalibration,
    lidar_to_map: Iterable[Iterable[float]],
    *,
    image_stamp: float,
    cloud_stamp: float,
    class_name: str,
    track_id: str,
    confidence: float,
) -> dict:
    sync_delta_ms = validate_sync(image_stamp, cloud_stamp)
    points = np.asarray(points_lidar, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ProjectionRejected("point_cloud_invalid", "点云必须为N×3数组")
    points = points[:, :3]
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ProjectionRejected("no_valid_points", "点云没有有限坐标")

    homogeneous = np.column_stack((points, np.ones(len(points))))
    camera_points = (calibration.matrix() @ homogeneous.T).T[:, :3]
    in_front = camera_points[:, 2] > 0.05
    points, camera_points = points[in_front], camera_points[in_front]
    if not len(points):
        raise ProjectionRejected("outside_camera", "点云均位于相机后方")
    u, v = _project_pixels(camera_points, calibration)
    x, y = float(bbox.get("x", 0)), float(bbox.get("y", 0))
    width, height = float(bbox.get("width", 0)), float(bbox.get("height", 0))
    if width <= 0 or height <= 0:
        raise ProjectionRejected("bbox_invalid", "检测框尺寸无效")
    # Ten percent inset reduces background points on the box boundary.
    inset_x, inset_y = width * 0.1, height * 0.1
    selected = (u >= x + inset_x) & (u <= x + width - inset_x) & (v >= y + inset_y) & (v <= y + height - inset_y)
    foreground_camera = _foreground_cluster(camera_points[selected])
    if len(foreground_camera) < MIN_SUPPORT_POINTS:
        raise ProjectionRejected("insufficient_points", f"检测框内仅有 {len(foreground_camera)} 个前景点")
    center_camera = np.median(foreground_camera, axis=0)
    if float(np.linalg.norm(center_camera)) > MAX_RANGE_M:
        raise ProjectionRejected("range_exceeded", "目标超过20米可靠投影范围")

    map_matrix = np.asarray(lidar_to_map, dtype=np.float64)
    if map_matrix.shape != (4, 4) or not np.isfinite(map_matrix).all():
        raise ProjectionRejected("tf_missing", "缺少点云时刻的LiDAR到map变换")
    camera_to_lidar = np.linalg.inv(calibration.matrix())
    center_lidar = camera_to_lidar @ np.append(center_camera, 1.0)
    center_map = (map_matrix @ center_lidar)[:3]
    spread = np.linalg.norm(foreground_camera - center_camera, axis=1)
    position_std = float(np.std(spread))
    if position_std > MAX_POSITION_STD_M:
        raise ProjectionRejected("position_uncertain", f"位置标准差 {position_std:.3f}m 超限")
    dimensions = np.maximum(0.1, np.percentile(foreground_camera, 95, axis=0) - np.percentile(foreground_camera, 5, axis=0))
    dynamic = str(class_name).lower() not in {"wall", "building"}
    return {
        "track_id": str(track_id),
        "class_name": str(class_name).lower(),
        "confidence": max(0.0, min(1.0, float(confidence))),
        "position": {"x": float(center_map[0]), "y": float(center_map[1]), "z": float(center_map[2])},
        "dimensions": {"x": float(dimensions[0]), "y": float(dimensions[1]), "z": float(dimensions[2])},
        "position_std_m": position_std,
        "dynamic": dynamic,
        "projection_method": "lidar_camera_cluster",
        "calibration_id": calibration.calibration_id,
        "source_bbox": [int(x), int(y), int(width), int(height)],
        "sync_delta_ms": sync_delta_ms,
        "selected_points": int(len(foreground_camera)),
    }


class SemanticTrackFilter:
    """Small constant-velocity alpha-beta filter keyed by YOLO track ID."""

    def __init__(self, ttl_seconds: float = 1.5, alpha: float = 0.72, beta: float = 0.18):
        self.ttl_seconds = float(ttl_seconds)
        self.alpha = float(alpha)
        self.beta = float(beta)
        self._tracks: dict[str, dict] = {}

    def update(self, item: dict, observed_at: float) -> dict:
        track_id = str(item["track_id"])
        measurement = np.array([item["position"][axis] for axis in ("x", "y", "z")], dtype=float)
        previous = self._tracks.get(track_id)
        if previous:
            dt = max(1e-3, float(observed_at) - previous["at"])
            prediction = previous["position"] + previous["velocity"] * dt
            residual = measurement - prediction
            position = prediction + self.alpha * residual
            velocity = previous["velocity"] + self.beta * residual / dt
        else:
            position, velocity = measurement, np.zeros(3)
        self._tracks[track_id] = {"at": float(observed_at), "position": position, "velocity": velocity}
        result = dict(item)
        result["position"] = {axis: float(position[index]) for index, axis in enumerate(("x", "y", "z"))}
        return result

    def expire(self, now: float) -> list[str]:
        expired = [track_id for track_id, value in self._tracks.items() if float(now) - value["at"] > self.ttl_seconds]
        for track_id in expired:
            self._tracks.pop(track_id, None)
        return expired
