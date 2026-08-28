"""Validation helpers shared by the demo script and tests.

The validator is intentionally conservative: it never repairs, reindexes, or
deletes a bag.  Full rosbag metadata/count validation is delegated to the ROS
CLI after this cheap filesystem check.
"""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import json
from pathlib import Path
import os
import re
from typing import Iterable


REQUIRED_TOPICS: tuple[str, ...] = (
    "/clock",
    "/map",
    "/tf",
    "/tf_static",
    "/odom",
    "/cmd_vel",
    "/scan",
    "/camera/front/image/compressed",
    "/imu/data",
    "/battery_state",
    "/diagnostics",
    "/plan",
    "/goal_pose",
    "/patrol/trajectory",
    "/patrol/status",
)
_TOPIC_RE = re.compile(r"^/[A-Za-z0-9_./-]+$")


class BagContractError(ValueError):
    """The selected directory is not a safe complete Bag input."""


@dataclass(frozen=True)
class BagInspection:
    path: Path
    metadata: Path
    #: The storage files themselves, sqlite3 (.db3) or mcap.  Which one it is
    #: comes from metadata.yaml; this check only cares that some payload exists
    #: beside the metadata.
    data_files: tuple[Path, ...]


def _contained(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def inspect_bag_directory(path: str | os.PathLike[str], allowed_root: str | os.PathLike[str] | None = None) -> BagInspection:
    candidate = Path(path).expanduser().resolve(strict=False)
    if not candidate.is_dir():
        raise BagContractError(f"Bag directory does not exist: {candidate}")
    if allowed_root is not None:
        root = Path(allowed_root).expanduser().resolve(strict=False)
        if not _contained(candidate, root):
            raise BagContractError(f"Bag path is outside allowed root: {candidate}")
    metadata = candidate / "metadata.yaml"
    if not metadata.is_file():
        raise BagContractError(f"Bag directory is missing metadata.yaml: {candidate}")
    data_files = tuple(sorted(p for p in candidate.iterdir() if p.is_file() and p.suffix in (".db3", ".mcap")))
    if not data_files:
        raise BagContractError(f"Bag directory contains no .db3 or .mcap files: {candidate}")
    return BagInspection(candidate, metadata, data_files)


def validate_topic_names(namespace: str, names: Iterable[str]) -> list[str]:
    """Apply a ROS namespace and reject traversal-like topic names."""
    prefix = namespace.rstrip("/")
    if prefix and not _TOPIC_RE.fullmatch(prefix):
        raise ValueError(f"invalid namespace: {namespace}")
    result: list[str] = []
    for raw in names:
        value = raw if raw.startswith("/") else f"/{raw}"
        full = f"{prefix}{value}" if prefix else value
        if "//" in full or "/../" in full or not _TOPIC_RE.fullmatch(full):
            raise ValueError(f"invalid topic name: {raw}")
        result.append(full)
    return result


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate a complete rosbag2 directory (sqlite3 or mcap)")
    parser.add_argument("path")
    parser.add_argument("--allowed-root")
    options = parser.parse_args(args)
    try:
        result = inspect_bag_directory(options.path, options.allowed_root)
    except BagContractError as exc:
        parser.error(str(exc))
        return 2
    print(json.dumps({"path": str(result.path), "metadata": str(result.metadata), "data_files": [str(p) for p in result.data_files]}, ensure_ascii=False))
    return 0
