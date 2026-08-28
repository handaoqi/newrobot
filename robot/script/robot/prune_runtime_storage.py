#!/usr/bin/env python3
"""Retention for the two directories that grow without bound on the NX.

`runtime/nx-edge/data/rosbags/` and `runtime/nx-edge/data/jszr/map/` had no
cleanup of any kind. At the time this was written they held 34 GB of mapping
bags across 62 sessions and 19 GB of maps across 141 sessions, on a 233 GB disk
that was already 78% full. The only existing guard is mapping_rosbag.sh
refusing to *start* a recording below MIN_FREE_GB, which turns a slow leak into
a hard stop on the mapping feature rather than preventing it.

Three limits are applied. The newest ``keep_recent`` sessions are kept
unconditionally; past that, a session goes if it trips *either* the age limit
or the total-size budget. The two answer different questions - "this is too old
to be worth keeping" and "we cannot afford this much" - and requiring both
means the size budget only ever bites once the data is also stale, which is
exactly when the disk is already full.

Deletion is refused for anything still referenced:

* a bag named by ``raw_recording`` in any ``map_manifest.json`` - that field is
  the provenance link between a map and the data it was built from, and there
  is no way to rebuild the map without it;
* a map session that any symlink in the map root resolves into - those links
  are how the running system finds the active map (see
  ``mapping_adapter._refresh_current_map_links``);
* the session currently being recorded, passed via --exclude;
* anything under a root that is not shaped like a dated session directory -
  ``manual_edits/`` in the map root holds platform-side edited maps keyed by
  map id, and nothing there is regenerable from a recording.

Dry run is the default. Nothing is removed without --apply.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

# Every session this tool is allowed to reclaim is named for the moment it was
# created: 20260816_150729, optionally with a label suffix. Anything else under
# these roots was put there by something other than the recorder or the mapper,
# so this tool does not get to have an opinion about it.
SESSION_NAME = re.compile(r"^\d{8}_\d{6}")

DEFAULT_MAP_ROOT = Path("/home/dogrobot/runtime/nx-edge/data/jszr/map")
DEFAULT_BAG_ROOTS = (
    Path("/home/dogrobot/runtime/nx-edge/data/rosbags/mapping"),
    Path("/home/dogrobot/runtime/nx-edge/data/rosbags/navigation"),
)

GIB = 1024 ** 3


@dataclass
class Limits:
    """Retention bounds. None disables that dimension entirely."""

    keep_recent: int = 10
    max_age_days: float | None = 30.0
    max_total_gib: float | None = 20.0

    def describe(self) -> dict:
        return {
            "keep_recent": self.keep_recent,
            "max_age_days": self.max_age_days,
            "max_total_gib": self.max_total_gib,
        }


@dataclass
class Session:
    path: Path
    size_bytes: int
    mtime: float
    protected_by: list[str] = field(default_factory=list)

    @property
    def protected(self) -> bool:
        return bool(self.protected_by)


def directory_size(path: Path) -> int:
    total = 0
    for root, _dirs, files in os.walk(path, onerror=lambda _e: None):
        for name in files:
            candidate = Path(root) / name
            try:
                # lstat, not stat: a broken or outbound symlink must not be
                # counted (or followed) while measuring.
                stat = candidate.lstat()
            except OSError:
                continue
            if not os.path.islink(candidate):
                total += stat.st_size
    return total


def referenced_recordings(map_root: Path) -> dict[str, str]:
    """Map every bag path named by a manifest's raw_recording to its map dir."""
    referenced: dict[str, str] = {}
    if not map_root.is_dir():
        return referenced
    for manifest in map_root.glob("*/map_manifest.json"):
        try:
            data = json.loads(manifest.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            # An unreadable manifest is the strongest possible reason not to
            # delete things: it may reference a bag we cannot see.
            print(f"WARNING: unreadable manifest, treating its session as opaque: {manifest}",
                  file=sys.stderr)
            continue
        raw = str(data.get("raw_recording") or "").strip()
        if raw:
            referenced[os.path.normpath(raw)] = manifest.parent.name
    return referenced


def linked_map_sessions(map_root: Path) -> set[str]:
    """Session directories reachable through the root-level 'current map' links."""
    linked: set[str] = set()
    if not map_root.is_dir():
        return linked
    for entry in map_root.iterdir():
        if not entry.is_symlink():
            continue
        try:
            target = entry.resolve()
        except OSError:
            continue
        if not target.exists():
            # A dangling link names no session, so it protects nothing. Skipping
            # keeps this set meaningful instead of seeded with phantom names.
            continue
        try:
            relative = target.relative_to(map_root.resolve())
        except ValueError:
            continue
        if relative.parts:
            linked.add(relative.parts[0])
    return linked


def baseline_protection(path: Path, excluded: set[str]) -> list[str]:
    """Protections that hold regardless of which root the session is under."""
    reasons = []
    if os.path.normpath(str(path)) in excluded:
        reasons.append("excluded by caller")
    if not SESSION_NAME.match(path.name):
        reasons.append("not a dated session directory")
    return reasons


def collect_sessions(root: Path, protector) -> list[Session]:
    if not root.is_dir():
        return []
    sessions = []
    for entry in sorted(root.iterdir()):
        if entry.is_symlink() or not entry.is_dir():
            continue
        try:
            mtime = entry.stat().st_mtime
        except OSError:
            continue
        sessions.append(
            Session(
                path=entry,
                size_bytes=directory_size(entry),
                mtime=mtime,
                protected_by=protector(entry),
            )
        )
    # Newest first: everything downstream reasons in "keep the first N".
    sessions.sort(key=lambda item: item.mtime, reverse=True)
    return sessions


def select_for_deletion(sessions: list[Session], limits: Limits, now: float) -> list[tuple[Session, str]]:
    """Pick sessions past keep_recent that trip the age limit or the budget.

    Protected sessions still consume the budget they occupy - they are real
    bytes on the disk - but are never selected.
    """
    selected: list[tuple[Session, str]] = []
    running_total = 0
    age_cutoff = None if limits.max_age_days is None else now - limits.max_age_days * 86400
    size_budget = None if limits.max_total_gib is None else limits.max_total_gib * GIB

    for index, session in enumerate(sessions):
        running_total += session.size_bytes
        if index < limits.keep_recent:
            continue
        reasons = []
        if age_cutoff is not None and session.mtime < age_cutoff:
            reasons.append(f"older than {limits.max_age_days:g}d")
        if size_budget is not None and running_total > size_budget:
            reasons.append(f"beyond {limits.max_total_gib:g}GiB budget")
        # Either limit is sufficient. They answer different questions - "this is
        # too old to be worth keeping" and "we cannot afford this much" - and
        # requiring both means the size budget can never bite until the data is
        # also stale, which is exactly when the disk is already full. Recent
        # data is protected by keep_recent above, not by this conjunction.
        if not reasons or session.protected:
            continue
        selected.append((session, " and ".join(reasons)))
    return selected


def prune_root(root: Path, limits: Limits, protector, apply: bool, now: float) -> dict:
    sessions = collect_sessions(root, protector)
    candidates = select_for_deletion(sessions, limits, now)
    removed, failed = [], []

    for session, reason in candidates:
        entry = {
            "path": str(session.path),
            "size_bytes": session.size_bytes,
            "age_days": round((now - session.mtime) / 86400, 1),
            "reason": reason,
        }
        if not apply:
            removed.append(entry)
            continue
        try:
            shutil.rmtree(session.path)
        except OSError as exc:
            entry["error"] = str(exc)
            failed.append(entry)
            continue
        removed.append(entry)

    return {
        "root": str(root),
        "exists": root.is_dir(),
        "session_count": len(sessions),
        "total_bytes": sum(item.size_bytes for item in sessions),
        "protected_count": sum(1 for item in sessions if item.protected),
        "protected": [
            {"path": str(item.path), "protected_by": item.protected_by}
            for item in sessions
            if item.protected
        ],
        "deleted" if apply else "would_delete": removed,
        "reclaimed_bytes": sum(item["size_bytes"] for item in removed if "error" not in item),
        "failed": failed,
    }


def disk_usage(path: Path) -> dict:
    probe = path if path.exists() else path.anchor or "/"
    usage = shutil.disk_usage(probe)
    return {
        "path": str(probe),
        "total_bytes": usage.total,
        "used_bytes": usage.used,
        "free_bytes": usage.free,
        "free_gib": round(usage.free / GIB, 2),
        "used_percent": round(usage.used / usage.total * 100, 1) if usage.total else None,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map-root", type=Path, default=DEFAULT_MAP_ROOT)
    parser.add_argument(
        "--bag-root",
        type=Path,
        action="append",
        dest="bag_roots",
        help="Repeatable. Defaults to the mapping and navigation bag roots.",
    )
    parser.add_argument(
        "--exclude",
        action="append",
        default=[],
        help="Repeatable. A session path never to touch, e.g. the bag being recorded right now.",
    )
    parser.add_argument("--bag-keep-recent", type=int, default=10)
    parser.add_argument("--bag-max-age-days", type=float, default=30.0)
    parser.add_argument("--bag-max-total-gib", type=float, default=20.0)
    parser.add_argument("--map-keep-recent", type=int, default=20)
    parser.add_argument("--map-max-age-days", type=float, default=60.0)
    parser.add_argument("--map-max-total-gib", type=float, default=12.0)
    parser.add_argument("--skip-maps", action="store_true", help="Prune bags only.")
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Actually delete. Without it this only reports what it would do.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    now = time.time()
    map_root: Path = args.map_root
    bag_roots = args.bag_roots or list(DEFAULT_BAG_ROOTS)
    excluded = {os.path.normpath(str(item)) for item in args.exclude}

    referenced = referenced_recordings(map_root)
    linked = linked_map_sessions(map_root)

    def bag_protector(path: Path) -> list[str]:
        reasons = baseline_protection(path, excluded)
        owner = referenced.get(os.path.normpath(str(path)))
        if owner:
            reasons.append(f"raw_recording of map {owner}")
        return reasons

    def map_protector(path: Path) -> list[str]:
        reasons = baseline_protection(path, excluded)
        if path.name in linked:
            reasons.append("target of a current-map symlink")
        return reasons

    bag_limits = Limits(args.bag_keep_recent, args.bag_max_age_days, args.bag_max_total_gib)
    map_limits = Limits(args.map_keep_recent, args.map_max_age_days, args.map_max_total_gib)

    report = {
        "schema": "roamerx.storage-retention.v1",
        "generated_at_unix": round(now, 3),
        "applied": bool(args.apply),
        "disk": disk_usage(map_root),
        "limits": {"bags": bag_limits.describe(), "maps": map_limits.describe()},
        "roots": [prune_root(root, bag_limits, bag_protector, args.apply, now) for root in bag_roots],
    }
    if not args.skip_maps:
        report["roots"].append(prune_root(map_root, map_limits, map_protector, args.apply, now))

    report["reclaimed_bytes"] = sum(item["reclaimed_bytes"] for item in report["roots"])
    report["failed_count"] = sum(len(item["failed"]) for item in report["roots"])
    print(json.dumps(report, ensure_ascii=False))
    return 1 if report["failed_count"] else 0


if __name__ == "__main__":
    sys.exit(main())
