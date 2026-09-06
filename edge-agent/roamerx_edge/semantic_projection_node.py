"""ROS2 bridge from atomic YOLO snapshots and LiDAR to 3D scene objects.

This node is read-only with respect to robot control. It publishes visualization
evidence only, and fails closed when calibration, timestamps, point support, or
the cloud-time map transform are not trustworthy.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path

import numpy as np
import yaml

from .semantic_projection import CameraCalibration, ProjectionRejected, SemanticTrackFilter, project_detection

LOGGER = logging.getLogger(__name__)


def load_calibration(path: str | Path) -> CameraCalibration:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    payload = payload.get("calibration", payload)
    image_size = payload.get("image_size", [payload.get("width"), payload.get("height")])
    camera_matrix = payload.get("camera_matrix", [])
    if len(camera_matrix) == 9:
        fx, fy, cx, cy = camera_matrix[0], camera_matrix[4], camera_matrix[2], camera_matrix[5]
    else:
        fx, fy, cx, cy = (payload.get(key) for key in ("fx", "fy", "cx", "cy"))
    return CameraCalibration(
        calibration_id=str(payload.get("id") or payload.get("calibration_id") or ""),
        width=int(image_size[0]), height=int(image_size[1]),
        fx=float(fx), fy=float(fy), cx=float(cx), cy=float(cy),
        distortion=tuple(float(value) for value in payload.get("distortion", [0, 0, 0, 0, 0])),
        lidar_to_camera=tuple(tuple(float(value) for value in row) for row in payload["lidar_to_camera"]),
    )


def pointcloud_xyz(message, max_points: int = 200_000) -> np.ndarray:
    fields = {field.name: field for field in message.fields}
    if not all(axis in fields for axis in ("x", "y", "z")) or message.point_step <= 0:
        raise ProjectionRejected("point_cloud_invalid", "PointCloud2缺少float32 x/y/z字段")
    total = min(int(message.width) * max(1, int(message.height)), len(message.data) // int(message.point_step))
    stride = max(1, int(np.ceil(total / max(1, max_points))))
    endian = ">" if message.is_bigendian else "<"
    axes = []
    raw = memoryview(message.data)
    for axis in ("x", "y", "z"):
        field = fields[axis]
        if int(field.datatype) != 7:
            raise ProjectionRejected("point_cloud_invalid", f"PointCloud2字段{axis}不是float32")
        values = np.ndarray(
            shape=(total,), dtype=f"{endian}f4", buffer=raw,
            offset=int(field.offset), strides=(int(message.point_step),),
        )
        axes.append(values[::stride].astype(np.float64, copy=True))
    return np.column_stack(axes)


def transform_matrix(transform) -> np.ndarray:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    x, y, z, w = (float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w))
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        raise ProjectionRejected("tf_invalid", "TF四元数无效")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), float(translation.x)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), float(translation.y)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), float(translation.z)],
        [0, 0, 0, 1],
    ], dtype=np.float64)
    return matrix


def main(args=None) -> None:
    try:
        import rclpy
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from robots_dog_msgs.msg import SemanticObject3D, SemanticObjectArray
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from tf2_ros import Buffer, TransformListener
    except ImportError as exc:  # pragma: no cover - depends on the robot ROS install
        raise SystemExit(f"ROS2 semantic projection dependencies are unavailable: {exc}") from exc

    class SemanticProjectionNode(Node):
        def __init__(self):
            super().__init__("roamerx_semantic_projection")
            self.declare_parameter("detection_path", "/run/roamerx/person_detections.json")
            self.declare_parameter("calibration_file", "/home/dogrobot/runtime/nx-edge/conf/semantic_projection.yaml")
            self.declare_parameter("cloud_topic", "/front_lidar")
            self.declare_parameter("map_frame", "map")
            self.declare_parameter("poll_hz", 10.0)
            self.declare_parameter("max_cloud_points", 200000)
            self._detection_path = Path(self.get_parameter("detection_path").value)
            calibration_path = self.get_parameter("calibration_file").value
            try:
                self._calibration = load_calibration(calibration_path)
                self._calibration.matrix()
            except Exception as exc:
                self.get_logger().error(f"semantic projection calibration rejected: {exc}")
                raise
            self._map_frame = str(self.get_parameter("map_frame").value)
            self._maximum_points = int(self.get_parameter("max_cloud_points").value)
            self._last_detection_mtime_ns = -1
            self._cloud = None
            self._tracker = SemanticTrackFilter()
            self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._objects_pub = self.create_publisher(SemanticObjectArray, "/perception/semantic_objects", 10)
            self._status_pub = self.create_publisher(String, "/perception/projection_status", 10)
            self.create_subscription(PointCloud2, self.get_parameter("cloud_topic").value, self._on_cloud, qos_profile_sensor_data)
            poll_hz = max(1.0, float(self.get_parameter("poll_hz").value))
            self.create_timer(1.0 / poll_hz, self._poll)
            self.get_logger().info(
                f"semantic projection ready calibration={self._calibration.calibration_id} cloud_topic={self.get_parameter('cloud_topic').value}"
            )

        def _on_cloud(self, message):
            try:
                points = pointcloud_xyz(message, self._maximum_points)
                cloud_stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) / 1e9
                self._cloud = (message.header, cloud_stamp, points)
                self.get_logger().debug(f"semantic cloud stamp={cloud_stamp:.6f} points={len(points)}")
            except ProjectionRejected as exc:
                self._publish_status({"state": "rejected", "rejection_reason": exc.code, "detail": str(exc)})

        def _poll(self):
            try:
                stat = self._detection_path.stat()
            except FileNotFoundError:
                return
            if stat.st_mtime_ns == self._last_detection_mtime_ns:
                return
            self._last_detection_mtime_ns = stat.st_mtime_ns
            try:
                frame = json.loads(self._detection_path.read_text(encoding="utf-8"))
                self._process(frame)
            except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
                self.get_logger().warning(f"semantic detection snapshot rejected: {exc}")
                self._publish_status({"state": "rejected", "rejection_reason": "detection_snapshot_invalid", "detail": str(exc)})

        def _process(self, frame):
            if self._cloud is None:
                self._publish_status({"state": "waiting", "rejection_reason": "point_cloud_unavailable"})
                return
            header, cloud_stamp, points = self._cloud
            image_stamp = float(frame.get("captured_at_unix") or 0)
            if image_stamp < 1_000_000_000 or cloud_stamp < 1_000_000_000:
                self._publish_status({"state": "rejected", "rejection_reason": "time_basis_invalid", "image_stamp": image_stamp, "cloud_stamp": cloud_stamp})
                return
            try:
                transform = self._tf_buffer.lookup_transform(self._map_frame, header.frame_id, Time.from_msg(header.stamp), timeout=Duration(seconds=.05))
                lidar_to_map = transform_matrix(transform)
            except Exception as exc:
                self._publish_status({"state": "rejected", "rejection_reason": "tf_missing", "detail": str(exc)})
                return

            output = SemanticObjectArray()
            output.header = header
            output.header.frame_id = self._map_frame
            rejected = []
            selected_points = 0
            for detection in frame.get("detections", [])[:50]:
                try:
                    result = project_detection(
                        points, detection.get("bbox") or {}, self._calibration, lidar_to_map,
                        image_stamp=image_stamp, cloud_stamp=cloud_stamp,
                        class_name=detection.get("label", "unknown"), track_id=detection.get("track_id", ""),
                        confidence=detection.get("confidence", 0),
                    )
                    result = self._tracker.update(result, image_stamp)
                    item = SemanticObject3D()
                    item.track_id = result["track_id"]
                    item.class_name = result["class_name"]
                    item.confidence = result["confidence"]
                    item.pose.position.x, item.pose.position.y, item.pose.position.z = (result["position"][axis] for axis in ("x", "y", "z"))
                    item.pose.orientation.w = 1.0
                    item.dimensions.x, item.dimensions.y, item.dimensions.z = (result["dimensions"][axis] for axis in ("x", "y", "z"))
                    item.position_std_m = result["position_std_m"]
                    item.dynamic = result["dynamic"]
                    item.projection_method = result["projection_method"]
                    item.calibration_id = result["calibration_id"]
                    item.source_bbox = result["source_bbox"]
                    output.objects.append(item)
                    selected_points += result["selected_points"]
                except ProjectionRejected as exc:
                    rejected.append({"track_id": str(detection.get("track_id", "")), "reason": exc.code})
            self._tracker.expire(image_stamp)
            self._objects_pub.publish(output)
            if rejected:
                self.get_logger().warning(
                    f"semantic projection rejected count={len(rejected)} reasons={','.join(item['reason'] for item in rejected)}"
                )
            self._publish_status({
                "state": "ok" if output.objects else "rejected", "accepted": len(output.objects),
                "rejected": rejected, "selected_points": selected_points,
                "sync_delta_ms": abs(image_stamp - cloud_stamp) * 1000,
                "calibration_id": self._calibration.calibration_id,
                "source_frame_id": int(frame.get("source_frame_id") or 0),
                "rejection_reason": rejected[0]["reason"] if rejected and not output.objects else "",
            })

        def _publish_status(self, payload):
            message = String()
            message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            self._status_pub.publish(message)

    rclpy.init(args=args)
    node = SemanticProjectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
