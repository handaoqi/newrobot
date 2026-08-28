#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import time
from pathlib import Path

from prune_runtime_storage import (
    Limits,
    collect_sessions,
    linked_map_sessions,
    main,
    referenced_recordings,
    select_for_deletion,
)

DAY = 86400.0


def _session(root: Path, name: str, *, size: int, age_days: float) -> Path:
    path = root / name
    path.mkdir(parents=True)
    (path / "data.bin").write_bytes(b"\0" * size)
    stamp = time.time() - age_days * DAY
    os.utime(path, (stamp, stamp))
    return path


def _no_protection(_path: Path) -> list[str]:
    return []


def test_keep_recent_wins_over_both_limits(tmp_path):
    for index in range(5):
        _session(tmp_path, f"s{index}", size=4096, age_days=400 - index)
    sessions = collect_sessions(tmp_path, _no_protection)

    selected = select_for_deletion(
        sessions, Limits(keep_recent=5, max_age_days=1.0, max_total_gib=0.0), time.time()
    )

    # Ancient and far over budget, but all five are inside the keep window.
    assert selected == []


def test_either_limit_alone_is_enough(tmp_path):
    _session(tmp_path, "new_small", size=1024, age_days=1)
    _session(tmp_path, "old_small", size=1024, age_days=99)
    sessions = collect_sessions(tmp_path, _no_protection)
    now = time.time()

    by_age = select_for_deletion(sessions, Limits(0, 30.0, None), now)
    assert [item[0].path.name for item in by_age] == ["old_small"]
    assert "older than 30d" in by_age[0][1]

    by_size = select_for_deletion(sessions, Limits(1, None, 0.0), now)
    # keep_recent=1 spares the newest; the next one is over a zero budget.
    assert [item[0].path.name for item in by_size] == ["old_small"]
    assert "budget" in by_size[0][1]


def test_protected_sessions_are_never_selected(tmp_path):
    _session(tmp_path, "keeper", size=1024, age_days=99)
    _session(tmp_path, "expendable", size=1024, age_days=99)

    def protector(path: Path) -> list[str]:
        return ["raw_recording of map m1"] if path.name == "keeper" else []

    sessions = collect_sessions(tmp_path, protector)
    selected = select_for_deletion(sessions, Limits(0, 1.0, None), time.time())

    assert [item[0].path.name for item in selected] == ["expendable"]


def test_raw_recording_references_are_discovered(tmp_path):
    map_root = tmp_path / "map"
    (map_root / "m1").mkdir(parents=True)
    (map_root / "m1" / "map_manifest.json").write_text(
        json.dumps({"raw_recording": "/bags/mapping/20260101_a/"}), encoding="utf-8"
    )
    (map_root / "m2").mkdir()
    (map_root / "m2" / "map_manifest.json").write_text(
        json.dumps({"raw_recording": ""}), encoding="utf-8"
    )

    referenced = referenced_recordings(map_root)

    # Normalized, so a trailing slash in the manifest still matches a real path.
    assert referenced == {"/bags/mapping/20260101_a": "m1"}


def test_unreadable_manifest_does_not_abort_the_scan(tmp_path, capsys):
    map_root = tmp_path / "map"
    (map_root / "broken").mkdir(parents=True)
    (map_root / "broken" / "map_manifest.json").write_text("{not json", encoding="utf-8")
    (map_root / "ok").mkdir()
    (map_root / "ok" / "map_manifest.json").write_text(
        json.dumps({"raw_recording": "/bags/x"}), encoding="utf-8"
    )

    assert referenced_recordings(map_root) == {"/bags/x": "ok"}
    assert "unreadable manifest" in capsys.readouterr().err


def test_current_map_symlinks_protect_their_session(tmp_path):
    map_root = tmp_path / "map"
    session = map_root / "20260101_120000"
    session.mkdir(parents=True)
    (session / "map.pcd").write_bytes(b"x")
    (map_root / "map.pcd").symlink_to(session / "map.pcd")
    (map_root / "dangling").symlink_to(map_root / "gone" / "map.pcd")
    (map_root / "outbound").symlink_to(tmp_path / "elsewhere")

    assert linked_map_sessions(map_root) == {"20260101_120000"}


def test_symlinked_sessions_are_not_themselves_candidates(tmp_path):
    _session(tmp_path, "real", size=1024, age_days=99)
    (tmp_path / "alias").symlink_to(tmp_path / "real")

    sessions = collect_sessions(tmp_path, _no_protection)

    # The alias must not be walked as a second session, or the same bytes get
    # counted twice and deleting one breaks the other.
    assert [item.path.name for item in sessions] == ["real"]


def test_directory_size_ignores_symlinked_content(tmp_path):
    session = _session(tmp_path, "s", size=2048, age_days=1)
    (tmp_path / "big").write_bytes(b"\0" * 100_000)
    (session / "link").symlink_to(tmp_path / "big")

    sessions = collect_sessions(tmp_path, _no_protection)
    only = next(item for item in sessions if item.path.name == "s")

    assert only.size_bytes == 2048


def test_dry_run_is_the_default_and_deletes_nothing(tmp_path, capsys):
    bags = tmp_path / "bags"
    _session(bags, "20260101_120000_old", size=1024, age_days=99)
    map_root = tmp_path / "map"
    map_root.mkdir()

    exit_code = main([
        "--map-root", str(map_root),
        "--bag-root", str(bags),
        "--bag-keep-recent", "0",
        "--bag-max-age-days", "30",
    ])

    report = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert report["applied"] is False
    assert (bags / "20260101_120000_old").exists()
    root_report = next(item for item in report["roots"] if item["root"] == str(bags))
    assert [item["path"] for item in root_report["would_delete"]] == [str(bags / "20260101_120000_old")]


def test_apply_deletes_and_respects_exclude_and_manifest(tmp_path, capsys):
    bags = tmp_path / "bags"
    doomed = _session(bags, "20260101_120000_doomed", size=1024, age_days=99)
    recording = _session(bags, "20260102_120000_recording", size=1024, age_days=99)
    sourced = _session(bags, "20260103_120000_sourced", size=1024, age_days=99)

    map_root = tmp_path / "map"
    (map_root / "m1").mkdir(parents=True)
    (map_root / "m1" / "map_manifest.json").write_text(
        json.dumps({"raw_recording": str(sourced)}), encoding="utf-8"
    )

    exit_code = main([
        "--map-root", str(map_root),
        "--bag-root", str(bags),
        "--bag-keep-recent", "0",
        "--bag-max-age-days", "30",
        "--exclude", str(recording),
        "--skip-maps",
        "--apply",
    ])

    report = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert not doomed.exists()
    assert recording.exists(), "the in-flight recording must survive"
    assert sourced.exists(), "a map's raw_recording must survive"
    root_report = report["roots"][0]
    assert [item["path"] for item in root_report["deleted"]] == [str(doomed)]
    assert sorted(item["path"] for item in root_report["protected"]) == sorted(
        [str(recording), str(sourced)]
    )
    assert report["reclaimed_bytes"] >= 1024


def test_missing_roots_are_reported_not_fatal(tmp_path, capsys):
    exit_code = main([
        "--map-root", str(tmp_path / "absent-map"),
        "--bag-root", str(tmp_path / "absent-bags"),
    ])

    report = json.loads(capsys.readouterr().out)
    assert exit_code == 0
    assert all(item["exists"] is False for item in report["roots"])
    assert report["reclaimed_bytes"] == 0


def test_a_directory_that_is_not_a_dated_session_is_never_touched(tmp_path):
    map_root = tmp_path / "map"
    edits = map_root / "manual_edits" / "map_101_legacy-mapdata-101"
    edits.mkdir(parents=True)
    (edits / "map.pcd").write_bytes(b"\0" * 4096)
    stamp = time.time() - 400 * DAY
    os.utime(map_root / "manual_edits", (stamp, stamp))
    _session(map_root, "20260101_120000", size=1024, age_days=400)

    exit_code = main([
        "--map-root", str(map_root),
        "--bag-root", str(tmp_path / "absent"),
        "--map-keep-recent", "0",
        "--map-max-age-days", "30",
        "--apply",
    ])

    assert exit_code == 0
    # manual_edits holds platform-side edited maps keyed by map id. Nothing
    # regenerates them, and they are older than any age limit worth setting.
    assert edits.exists()
    assert not (map_root / "20260101_120000").exists()
