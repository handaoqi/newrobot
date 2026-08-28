import json

import pytest

from roamerx_edge.map_coordinate import MapConstraintError
from roamerx_edge.map_package_finalize import finalize_map_package
from roamerx_edge.recording_manifest import REQUIRED_TOPICS, write_recording_manifest


def test_finalize_local_only_map_writes_manifest_and_keeps_raw(tmp_path):
    session = tmp_path / "20260821_180000_001"
    keyframes = session / "keyframes"
    keyframes.mkdir(parents=True)
    (session / "map.yaml").write_text("resolution: 0.05\n")
    (session / "map.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    (session / "map.pcd").write_bytes(b"raw-pcd")
    (keyframes / "keyframes.csv").write_text(
        "index,stamp,x,y,z,yaw,world_x,world_y,world_z,world_qx,world_qy,world_qz,world_qw,"
        "lidar_x,lidar_y,lidar_z,lidar_qx,lidar_qy,lidar_qz,lidar_qw,point_count,"
        "preintegration_file,scan_context_index,rtk_valid\n"
        "0,100.0,1.0,2.0,0.0,0.0,1.0,2.0,0.0,0.0,0.0,0.0,1.0,1.0,2.0,0.0,0.0,0.0,0.0,1.0,10,"
        "imu_preintegration/preint_00000.json,0,0\n"
    )
    preint = session / "imu_preintegration"
    preint.mkdir()
    (preint / "preint_00000.json").write_text(json.dumps({
        "schema_version": 1,
        "keyframe_index": 0,
        "delta_rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "error_status": "no_previous_keyframe",
    }))

    manifest = finalize_map_package(session, requested_scene_scope="indoor")

    assert manifest["coordinate_mode"] == "local_only"
    assert manifest["scene_scope"] == "indoor"
    assert manifest["mapping_type"] == "indoor"
    assert manifest["use_gps"] is False
    assert manifest["use_loop"] is False
    assert manifest["auto_loop_optimization_enabled"] is False
    assert manifest["loop_optimization_policy"] == "manual_review"
    assert manifest["loop_status"] == "no_valid_loop"
    assert manifest["loop_closure_count"] == 0
    assert manifest["localization_mode"] == "ndt"
    assert manifest["quaternion_order"] == "xyzw"
    assert (session / "map_raw.pcd").read_bytes() == b"raw-pcd"
    assert (session / "trajectory_raw.csv").exists()
    assert (session / "trajectory_optimized.csv").exists()
    assert (session / "scan_context" / "index.json").exists()


def test_finalize_rejects_outdoor_map_without_locked_origin(tmp_path):
    with pytest.raises(MapConstraintError) as error:
        finalize_map_package(tmp_path / "outdoor", requested_scene_scope="outdoor")

    assert error.value.code == "MAP_RTK_ORIGIN_REQUIRED"


def test_outdoor_manifest_carries_origin_lock_audit_fields(tmp_path):
    session = tmp_path / "outdoor-fixed"
    session.mkdir()
    (session / "gnss_origin.yaml").write_text(
        "alignment_locked: 1\n"
        "origin_lock_session_id: origin-123\n"
        "locked_at_unix: 1234.5\n"
        "position_spread_m: 0.012\n"
        "confirmed_heading_deg: 93.2\n"
        "heading_offset_deg: 180.0\n"
        "heading_confirmed_at_unix: 1240.0\n"
    )

    manifest = finalize_map_package(session, requested_scene_scope="outdoor")

    assert manifest["mapping_type"] == "outdoor"
    assert manifest["coordinate_mode"] == "rtk_fixed"
    assert manifest["use_loop"] is False
    assert manifest["auto_loop_optimization_enabled"] is False
    assert manifest["loop_optimization_policy"] == "manual_review"
    assert manifest["origin_lock_session_id"] == "origin-123"
    assert manifest["origin_position_spread_m"] == pytest.approx(0.012)
    assert manifest["origin_confirmed_heading_deg"] == pytest.approx(93.2)
    assert manifest["origin_heading_offset_deg"] == pytest.approx(180.0)
    assert manifest["origin_heading_confirmed_at_unix"] == pytest.approx(1240.0)


def test_explicit_auto_loop_policy_is_scene_independent(tmp_path):
    session = tmp_path / "explicit-loop-policy"
    session.mkdir()

    manifest = finalize_map_package(
        session,
        requested_scene_scope="indoor",
        auto_loop_optimization_enabled=True,
    )

    assert manifest["use_gps"] is False
    assert manifest["use_loop"] is True
    assert manifest["auto_loop_optimization_enabled"] is True
    assert manifest["loop_optimization_policy"] == "automatic"


def test_recording_manifest_lists_required_topics(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        """
rosbag2_bagfile_information:
  storage_identifier: sqlite3
  starting_time:
    nanoseconds_since_epoch: 1000000000
  duration:
    nanoseconds: 2000000000
  message_count: 9
  topics_with_message_count:
    - topic_metadata:
        name: /front_lidar
        type: sensor_msgs/msg/PointCloud2
        serialization_format: cdr
        offered_qos_profiles: []
      message_count: 1
    - topic_metadata:
        name: /front_lidar/imu
        type: sensor_msgs/msg/Imu
        serialization_format: cdr
        offered_qos_profiles: []
      message_count: 1
    - topic_metadata:
        name: /fix
        type: sensor_msgs/msg/NavSatFix
        serialization_format: cdr
        offered_qos_profiles: []
      message_count: 1
    - topic_metadata:
        name: /rtk_pvh
        type: robots_dog_msgs/msg/UniRtkPvh
        serialization_format: cdr
        offered_qos_profiles: []
      message_count: 1
    - topic_metadata:
        name: /rtk/ntrip_status
        type: std_msgs/msg/String
        serialization_format: cdr
        offered_qos_profiles: []
      message_count: 1
    - topic_metadata:
        name: /odom/localization_odom
        type: nav_msgs/msg/Odometry
        serialization_format: cdr
        offered_qos_profiles: []
      message_count: 1
    - topic_metadata:
        name: /slam_odom
        type: nav_msgs/msg/Odometry
        serialization_format: cdr
        offered_qos_profiles: []
      message_count: 1
    - topic_metadata:
        name: /tf
        type: tf2_msgs/msg/TFMessage
        serialization_format: cdr
        offered_qos_profiles: []
      message_count: 1
    - topic_metadata:
        name: /tf_static
        type: tf2_msgs/msg/TFMessage
        serialization_format: cdr
        offered_qos_profiles: []
      message_count: 1
"""
    )

    manifest = write_recording_manifest(bag, tmp_path / "recording_manifest.yaml")

    assert manifest["missing_required_topics"] == []
    assert {topic["name"] for topic in manifest["topics"]} == set(REQUIRED_TOPICS)
    assert (tmp_path / "recording_manifest.yaml").exists()
