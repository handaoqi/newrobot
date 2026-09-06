import numpy as np
import pytest

from roamerx_edge.semantic_projection import CameraCalibration, ProjectionRejected, SemanticTrackFilter, project_detection


def calibration():
    return CameraCalibration(
        calibration_id="front-v1", width=640, height=480,
        fx=400, fy=400, cx=320, cy=240,
        lidar_to_camera=((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)),
    )


def target_points():
    xs = np.linspace(-0.18, 0.18, 5)
    ys = np.linspace(-0.25, 0.25, 5)
    return np.array([[x, y, 4.0 + (x * .03)] for x in xs for y in ys])


def test_calibrated_projection_returns_map_pose_and_evidence():
    result = project_detection(
        target_points(), {"x": 285, "y": 205, "width": 70, "height": 70}, calibration(), np.eye(4),
        image_stamp=100.0, cloud_stamp=100.04, class_name="person", track_id="p-7", confidence=.91,
    )
    assert result["track_id"] == "p-7"
    assert result["selected_points"] >= 8
    assert result["sync_delta_ms"] == pytest.approx(40)
    assert result["position"]["z"] == pytest.approx(4, abs=.05)


def test_projection_rejects_stale_or_unsupported_measurements():
    with pytest.raises(ProjectionRejected) as stale:
        project_detection(target_points(), {"x": 0, "y": 0, "width": 640, "height": 480}, calibration(), np.eye(4), image_stamp=10, cloud_stamp=10.2, class_name="person", track_id="p", confidence=.9)
    assert stale.value.code == "time_sync_exceeded"
    with pytest.raises(ProjectionRejected) as sparse:
        project_detection(target_points()[:3], {"x": 0, "y": 0, "width": 640, "height": 480}, calibration(), np.eye(4), image_stamp=10, cloud_stamp=10.01, class_name="person", track_id="p", confidence=.9)
    assert sparse.value.code == "insufficient_points"


def test_track_filter_smooths_and_expires_dynamic_objects():
    tracker = SemanticTrackFilter(ttl_seconds=1.5)
    base = {"track_id": "p-1", "position": {"x": 0, "y": 0, "z": 0}}
    tracker.update(base, 1.0)
    moved = tracker.update({**base, "position": {"x": 1, "y": 0, "z": 0}}, 2.0)
    assert 0 < moved["position"]["x"] < 1
    assert tracker.expire(3.6) == ["p-1"]
