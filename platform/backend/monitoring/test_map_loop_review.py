import csv
import io
import json
import zipfile

import pytest

from monitoring.map_loop_review import LoopReviewError, audit_map_package, normalize_thresholds


def _package(path):
    candidates = io.StringIO()
    writer = csv.DictWriter(candidates, fieldnames=[
        "query_index", "match_index", "rank", "distance", "estimated_yaw_rad",
        "geometric_verified", "accepted", "rejection_reason", "dx", "dy", "dyaw",
    ])
    writer.writeheader()
    writer.writerow({
        "query_index": 0, "match_index": 40, "rank": 1, "distance": 0.12,
        "estimated_yaw_rad": 0.0, "geometric_verified": "True", "accepted": "True",
        "rejection_reason": "", "dx": 0.2, "dy": 0.1, "dyaw": 0.0,
    })
    keyframes = io.StringIO()
    keyframe_writer = csv.DictWriter(keyframes, fieldnames=["index", "lidar_x", "lidar_y", "yaw"])
    keyframe_writer.writeheader()
    keyframe_writer.writerow({"index": 0, "lidar_x": 0, "lidar_y": 0, "yaw": 0})
    keyframe_writer.writerow({"index": 40, "lidar_x": 5, "lidar_y": 0, "yaw": 0})
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr("scan_context/loop_candidates.csv", candidates.getvalue())
        archive.writestr("scan_context/index.json", json.dumps({"keyframe_count": 41}))
        archive.writestr("keyframes/keyframes.csv", keyframes.getvalue())


def test_audit_marks_candidate_eligible(tmp_path):
    package = tmp_path / "map.zip"
    _package(package)
    review = audit_map_package(str(package))
    assert review["eligible_count"] == 1
    assert review["candidates"][0]["candidate_id"] == "0:40"
    assert review["candidates"][0]["checks"]["slam_distance"] is True


def test_threshold_validation_rejects_inverted_slam_range():
    with pytest.raises(LoopReviewError):
        normalize_thresholds({"slam_distance_min_m": 20, "slam_distance_max_m": 2})
