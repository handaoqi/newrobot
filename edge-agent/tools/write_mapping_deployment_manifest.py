#!/usr/bin/env python3
"""Write the runtime fingerprint required before starting mapping."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import time
from pathlib import Path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def artifact(path: Path) -> dict:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError(resolved)
    return {
        "path": str(resolved),
        "size_bytes": resolved.stat().st_size,
        "sha256": sha256_file(resolved),
    }


def git_value(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args], capture_output=True, text=True, check=False
    )
    return (result.stdout or "").strip() if result.returncode == 0 else ""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path("/home/dogrobot"))
    parser.add_argument("--slam-binary", type=Path, default=Path(
        "/home/dogrobot/robot/install/robot_slam/lib/robot_slam/mapping"))
    parser.add_argument("--slam-params", type=Path, default=Path(
        "/home/dogrobot/robot/install/robot_slam/share/robot_slam/config/config.yaml"))
    parser.add_argument("--mapping-adapter", type=Path, default=Path(
        "/home/dogrobot/edge-agent/roamerx_edge/mapping_adapter.py"))
    parser.add_argument("--mapping-unit-file", type=Path, default=Path(
        "/etc/systemd/system/roamerx-mapping.service"))
    parser.add_argument("--mapping-unit", default="roamerx-mapping.service")
    parser.add_argument("--slam-command", default=(
        "exec /home/dogrobot/robot/install/robot_slam/lib/robot_slam/mapping "
        "--ros-args --params-file /home/dogrobot/robot/install/robot_slam/share/robot_slam/config/config.yaml"
    ))
    parser.add_argument("--output", type=Path, default=Path(
        "/home/dogrobot/runtime/nx-edge/conf/mapping-deployment.json"))
    args = parser.parse_args()

    payload = {
        "schema": "roamerx.mapping-deployment.v1",
        "generated_at_unix": round(time.time(), 3),
        "git_commit": git_value(args.repo, "rev-parse", "HEAD"),
        "git_dirty": bool(git_value(args.repo, "status", "--porcelain")),
        "configuration": {
            "mapping_unit": args.mapping_unit,
            "slam_command": args.slam_command,
        },
        "artifacts": {
            "slam_binary": artifact(args.slam_binary),
            "slam_params_file": artifact(args.slam_params),
            "mapping_adapter": artifact(args.mapping_adapter),
            "mapping_unit_file": artifact(args.mapping_unit_file),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(args.output)
    print(json.dumps(payload, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
