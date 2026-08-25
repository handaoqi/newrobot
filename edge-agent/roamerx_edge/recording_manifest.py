"""Write recording_manifest.yaml from a rosbag2 session directory."""

from __future__ import annotations

import sqlite3
import time
from pathlib import Path
from typing import Any

import yaml

REQUIRED_TOPICS = (
    "/front_lidar",
    "/front_lidar/imu",
    "/fix",
    "/rtk_pvh",
    "/rtk/ntrip_status",
    "/odom/localization_odom",
    "/slam_odom",
    "/tf",
    "/tf_static",
)


def write_recording_manifest(bag_dir: str | Path, output_path: str | Path | None = None) -> dict[str, Any]:
    bag = Path(bag_dir)
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"rosbag metadata.yaml not found: {metadata_path}")
    raw = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    info = raw.get("rosbag2_bagfile_information") or raw
    topics = []
    missing = []
    present = set()
    for entry in info.get("topics_with_message_count") or []:
        metadata = entry.get("topic_metadata") or {}
        name = str(metadata.get("name") or "")
        present.add(name)
        topics.append(
            {
                "name": name,
                "type": str(metadata.get("type") or ""),
                "serialization_format": str(metadata.get("serialization_format") or ""),
                "message_count": int(entry.get("message_count") or 0),
                "offered_qos_profiles": metadata.get("offered_qos_profiles") or [],
            }
        )
    for name in REQUIRED_TOPICS:
        if name not in present:
            missing.append(name)
    starting_ns = int(((info.get("starting_time") or {}).get("nanoseconds_since_epoch")) or 0)
    duration_ns = int(((info.get("duration") or {}).get("nanoseconds")) or 0)
    start_unix = starting_ns / 1e9 if starting_ns else 0.0
    end_unix = start_unix + (duration_ns / 1e9 if duration_ns else 0.0)
    system_now = time.time()
    manifest = {
        "schema_version": 1,
        "bag_dir": str(bag),
        "storage_identifier": str(info.get("storage_identifier") or ""),
        "topics": topics,
        "required_topics": list(REQUIRED_TOPICS),
        "missing_required_topics": missing,
        "start_time_unix": start_unix,
        "end_time_unix": end_unix,
        "duration_seconds": duration_ns / 1e9 if duration_ns else 0.0,
        "message_count": int(info.get("message_count") or sum(item["message_count"] for item in topics)),
        "dropped_messages": [],
        "ros_to_system_time_offset_seconds": (system_now - end_unix) if end_unix else 0.0,
        "rtk_valid_intervals": _rtk_valid_intervals(bag, info),
    }
    destination = Path(output_path) if output_path else bag / "recording_manifest.yaml"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(yaml.safe_dump(manifest, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return manifest


def _rtk_valid_intervals(bag: Path, info: dict) -> list[dict]:
    db_files = [bag / name for name in (info.get("relative_file_paths") or []) if str(name).endswith(".db3")]
    if not db_files:
        db_files = list(bag.glob("*.db3"))
    if not db_files:
        return []
    try:
        connection = sqlite3.connect(f"file:{db_files[0]}?mode=ro", uri=True)
    except sqlite3.Error:
        return []
    try:
        row = connection.execute("SELECT id FROM topics WHERE name = '/fix'").fetchone()
        if not row:
            return []
        stamps = [
            int(stamp) / 1e9
            for (stamp,) in connection.execute(
                "SELECT timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp",
                (row[0],),
            )
        ]
    except sqlite3.Error:
        return []
    finally:
        connection.close()
    if not stamps:
        return []
    return [{"start_unix": stamps[0], "end_unix": stamps[-1], "sample_count": len(stamps), "source": "bag_fix_timestamps"}]
