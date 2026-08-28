from __future__ import annotations

import argparse
import json
import os
import sys

from .config import EdgeConfig
from .mapping_adapter import MappingAdapter
from .media_client import MediaClient
from .protocol import ProtocolError


DEFAULT_CONFIG = "/home/dogrobot/runtime/nx-edge/conf/edge-agent.yaml"


def build_adapter(config_path: str) -> MappingAdapter:
    config = EdgeConfig.load(config_path)
    return MappingAdapter(config.mapping, MediaClient(config.media, config.robot.id))


def print_status(payload: dict) -> None:
    print(json.dumps(payload, ensure_ascii=False, indent=2, default=str))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Canonical on-robot mapping control. Cloud mapping.start uses the same MappingAdapter."
    )
    parser.add_argument("action", choices=["start", "save", "stop", "restart", "status"])
    parser.add_argument("--config", default=os.environ.get("EDGE_CONFIG", DEFAULT_CONFIG))
    parser.add_argument("--map-name", default="")
    parser.add_argument("--record-rosbag", action="store_true")
    parser.add_argument("--upload", action="store_true", help="package and upload after save (cloud path)")
    parser.add_argument(
        "--keep-slam",
        action="store_true",
        help="leave the SLAM mapping node running after save, to keep mapping from the same session "
             "(checkpoint save). Without this, save stops it, matching the cloud mapping.save default.",
    )
    parser.add_argument("--wait-seconds", type=float, default=90.0)
    args = parser.parse_args(argv)

    adapter = build_adapter(args.config)
    command = {
        "map_name": args.map_name,
        "record_rosbag": args.record_rosbag,
        "upload": args.upload,
        "package": args.upload,
        "stop_process": not args.keep_slam,
    }
    try:
        if args.action == "status":
            print_status(adapter.status())
            return 0
        if args.action == "start":
            result = adapter.start_mapping(command)
            result = adapter.wait_until_ready_for_motion(args.wait_seconds)
            readiness = result.get("readiness") or {}
            print(readiness.get("message") or "mapping is ready")
            print_status(result)
            return 0
        if args.action == "save":
            result = adapter.save_mapping(command)
            print_status(result)
            return 0
        if args.action == "stop":
            if adapter.status().get("process_alive"):
                try:
                    adapter.save_mapping({**command, "stop_process": True})
                except ProtocolError as exc:
                    if exc.code != "MAPPING_NOT_READY":
                        raise
                    adapter.cancel_mapping(command)
            else:
                adapter.cancel_mapping(command)
            print_status(adapter.status())
            return 0
        adapter.cancel_mapping(command)
        result = adapter.start_mapping(command)
        result = adapter.wait_until_ready_for_motion(args.wait_seconds)
        print_status(result)
        return 0
    except ProtocolError as exc:
        print(f"{exc.code}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
