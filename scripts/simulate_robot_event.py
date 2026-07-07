from __future__ import annotations

import argparse
import hashlib
import json
import mimetypes
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


DEFAULT_BASE_URL = "http://127.0.0.1:8000"


def post_json(url: str, payload: dict, timeout: int) -> tuple[int, dict]:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    request = Request(
        url,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=timeout) as response:
        response_body = response.read().decode("utf-8")
        return response.status, json.loads(response_body) if response_body else {}


def encode_multipart(fields: dict[str, str], files: dict[str, Path]) -> tuple[bytes, str]:
    boundary = f"----robot-sim-{uuid.uuid4().hex}"
    chunks: list[bytes] = []

    for name, value in fields.items():
        chunks.extend(
            [
                f"--{boundary}\r\n".encode(),
                f'Content-Disposition: form-data; name="{name}"\r\n\r\n'.encode(),
                str(value).encode("utf-8"),
                b"\r\n",
            ]
        )

    for name, path in files.items():
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        chunks.extend(
            [
                f"--{boundary}\r\n".encode(),
                f'Content-Disposition: form-data; name="{name}"; filename="{path.name}"\r\n'.encode(),
                f"Content-Type: {content_type}\r\n\r\n".encode(),
                path.read_bytes(),
                b"\r\n",
            ]
        )

    chunks.append(f"--{boundary}--\r\n".encode())
    return b"".join(chunks), boundary


def upload_snapshot(base_url: str, image_path: Path, sequence_id: str, args: argparse.Namespace) -> str:
    content = image_path.read_bytes()
    fields = {
        "robot_code": args.robot_code,
        "camera_id": args.camera_id,
        "media_type": "snapshot",
        "event_time": args.event_time,
        "sequence_id": sequence_id,
        "sha256": hashlib.sha256(content).hexdigest(),
    }
    body, boundary = encode_multipart(fields, {"file": image_path})
    request = Request(
        f"{base_url}/api/device/media/upload/",
        data=body,
        headers={"Content-Type": f"multipart/form-data; boundary={boundary}"},
        method="POST",
    )
    with urlopen(request, timeout=args.timeout) as response:
        payload = json.loads(response.read().decode("utf-8"))
        return payload.get("url", "")


def build_payload(args: argparse.Namespace, sequence_id: str, snapshot_url: str) -> dict:
    return {
        "sequence_id": sequence_id,
        "robot_code": args.robot_code,
        "robot_name": args.robot_name,
        "reported_at": args.event_time,
        "position": {
            "name": args.location,
            "latitude": args.latitude,
            "longitude": args.longitude,
        },
        "motion": {
            "speed": 0.0,
            "heading": 83.5,
        },
        "power": {
            "battery_level": args.battery,
            "charging": False,
        },
        "network": {
            "signal_strength": args.signal,
            "network_type": args.network_type,
        },
        "runtime": {
            "mode": "auto",
            "status": "warning",
        },
        "video": {
            "camera_id": args.camera_id,
            "stream_id": args.stream_id,
            "frame_width": 1280,
            "frame_height": 720,
            "play_urls": {
                "flv": args.flv_url,
                "hls": args.hls_url,
            },
        },
        "detections": [
            {
                "type": "vehicle_illegal_parking",
                "label": "自行车违停",
                "risk_level": "medium",
                "confidence": args.confidence,
                "camera_id": args.camera_id,
                "stream_id": args.stream_id,
                "object_class": "bicycle",
                "track_id": f"sim-{uuid.uuid4().hex[:8]}",
                "event_time": args.event_time,
                "snapshot_url": snapshot_url,
                "bbox": {
                    "x": 320,
                    "y": 180,
                    "width": 260,
                    "height": 210,
                },
            }
        ],
    }


def parse_args() -> argparse.Namespace:
    now = datetime.now(timezone.utc).isoformat()
    parser = argparse.ArgumentParser(description="Simulate a robot inspection event upload.")
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL, help="backend origin, default http://127.0.0.1:8000")
    parser.add_argument("--robot-code", default="ZSL-1A-07")
    parser.add_argument("--robot-name", default="南入口巡检机器人")
    parser.add_argument("--location", default="太阳宫公园南入口")
    parser.add_argument("--latitude", type=float, default=39.983521)
    parser.add_argument("--longitude", type=float, default=116.447153)
    parser.add_argument("--camera-id", default="front")
    parser.add_argument("--stream-id", default="dog_ZSL-1A-07_front")
    parser.add_argument("--flv-url", default="http://127.0.0.1:8080/live/dog_ZSL-1A-07_front.live.flv")
    parser.add_argument("--hls-url", default="http://127.0.0.1:8080/live/dog_ZSL-1A-07_front/hls.m3u8")
    parser.add_argument("--battery", type=int, default=78)
    parser.add_argument("--signal", type=int, default=92)
    parser.add_argument("--network-type", default="5G")
    parser.add_argument("--confidence", type=float, default=0.88)
    parser.add_argument("--image", type=Path, help="optional local image to upload as event snapshot")
    parser.add_argument("--event-time", default=now, help="ISO datetime, default now")
    parser.add_argument("--timeout", type=int, default=10)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base_url = args.base_url.rstrip("/")
    sequence_id = f"sim-{datetime.now(timezone.utc).strftime('%Y%m%d%H%M%S')}-{uuid.uuid4().hex[:8]}"
    snapshot_url = ""

    try:
        if args.image:
            image_path = args.image.expanduser().resolve()
            if not image_path.exists():
                print(f"image not found: {image_path}", file=sys.stderr)
                return 2
            snapshot_url = upload_snapshot(base_url, image_path, sequence_id, args)
            print(f"snapshot uploaded: {snapshot_url}")

        payload = build_payload(args, sequence_id, snapshot_url)
        status_code, response_payload = post_json(f"{base_url}/api/telemetry/ingest/", payload, args.timeout)
    except HTTPError as exc:
        print(exc.read().decode("utf-8", errors="replace"), file=sys.stderr)
        return 1
    except URLError as exc:
        print(f"request failed: {exc}", file=sys.stderr)
        return 1

    print(f"telemetry status: {status_code}")
    print(json.dumps(response_payload, ensure_ascii=False, indent=2))
    print("frontend should receive SSE toast: 新告警：自行车违停")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
