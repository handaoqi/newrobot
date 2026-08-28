#!/usr/bin/env python3
"""Verify a foxglove_bridge is serving, read-only, and advertising topics.

Written against raw sockets on purpose: neither `websockets` nor
`websocket-client` is installed on the NX, and this has to run on the robot
without adding a dependency just to smoke-test a debug bridge.

Exit codes: 0 probe passed, 1 probe failed, 2 bad usage.

    foxglove_ws_probe.py --host 127.0.0.1 --port 8765 --expect /tf /odom
    foxglove_ws_probe.py --port 8080 --path /ws --expect /tf   # through the web proxy
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import socket
import struct
import sys
import time

# foxglove_bridge 3.4.3 (the version installed here) negotiates
# "foxglove.sdk.v1" and answers 400 Bad Request to the older
# "foxglove.websocket.v1" that most documentation still quotes. Both are offered
# so the probe keeps working across bridge versions; the server picks one.
SUBPROTOCOLS = ("foxglove.sdk.v1", "foxglove.websocket.v1")
# Capabilities that let a browser tab write to the robot. Their presence is a
# failure, not a warning: this probe is the gate that keeps a misconfigured
# bridge from being declared ready.
WRITE_CAPABILITIES = frozenset({"clientPublish", "services", "parameters", "parametersSubscribe"})


class ProbeError(RuntimeError):
    pass


def _handshake(sock: socket.socket, host: str, port: int, path: str = "/") -> bytes:
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    request = (
        f"GET {path} HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        f"Upgrade: websocket\r\n"
        f"Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        f"Sec-WebSocket-Version: 13\r\n"
        f"Sec-WebSocket-Protocol: {', '.join(SUBPROTOCOLS)}\r\n"
        f"\r\n"
    )
    sock.sendall(request.encode("ascii"))
    buffer = b""
    while b"\r\n\r\n" not in buffer:
        chunk = sock.recv(4096)
        if not chunk:
            raise ProbeError("connection closed during the HTTP upgrade")
        buffer += chunk
    header, _, rest = buffer.partition(b"\r\n\r\n")
    status = header.split(b"\r\n", 1)[0].decode("latin-1")
    if "101" not in status:
        raise ProbeError(f"server refused the websocket upgrade: {status}")
    if not any(name.encode() in header for name in SUBPROTOCOLS):
        raise ProbeError(f"server accepted none of the subprotocols {list(SUBPROTOCOLS)}")
    return rest


def _recv_exactly(sock: socket.socket, buffer: bytearray, count: int) -> bytes:
    while len(buffer) < count:
        chunk = sock.recv(65536)
        if not chunk:
            raise ProbeError("connection closed mid-frame")
        buffer += chunk
    result = bytes(buffer[:count])
    del buffer[:count]
    return result


def _read_frame(sock: socket.socket, buffer: bytearray) -> tuple[int, bytes]:
    first, second = _recv_exactly(sock, buffer, 2)
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack(">H", _recv_exactly(sock, buffer, 2))[0]
    elif length == 127:
        length = struct.unpack(">Q", _recv_exactly(sock, buffer, 8))[0]
    # A server must not mask, but handle it rather than desynchronising if it does.
    mask = _recv_exactly(sock, buffer, 4) if masked else b""
    payload = _recv_exactly(sock, buffer, length)
    if masked:
        payload = bytes(byte ^ mask[i % 4] for i, byte in enumerate(payload))
    return opcode, payload


def probe(host: str, port: int, expect: list[str], timeout: float, path: str = "/") -> dict:
    deadline = time.monotonic() + timeout
    try:
        sock = socket.create_connection((host, port), timeout=timeout)
    except OSError as exc:
        raise ProbeError(f"cannot connect to {host}:{port}: {exc}") from exc

    server_info: dict | None = None
    channels: dict[str, str] = {}
    with sock:
        sock.settimeout(timeout)
        buffer = bytearray(_handshake(sock, host, port, path))
        while time.monotonic() < deadline:
            try:
                opcode, payload = _read_frame(sock, buffer)
            except socket.timeout:
                break
            if opcode == 0x8:  # close
                break
            if opcode != 0x1:  # only text frames carry the JSON control plane
                continue
            message = json.loads(payload.decode("utf-8"))
            kind = message.get("op")
            if kind == "serverInfo":
                server_info = message
            elif kind == "advertise":
                for channel in message.get("channels", []):
                    channels[channel["topic"]] = channel.get("schemaName", "")
            # Stop as soon as the question can be answered.
            if server_info is not None and (not expect or set(expect) <= channels.keys()):
                break

    if server_info is None:
        raise ProbeError("no serverInfo received; the endpoint is not a foxglove_bridge")

    capabilities = set(server_info.get("capabilities", []))
    offending = sorted(capabilities & WRITE_CAPABILITIES)
    if offending:
        raise ProbeError(
            f"bridge advertises write capabilities {offending}; a browser could publish to the robot. "
            "Check the capabilities list in config/bridge.yaml."
        )
    missing = [topic for topic in expect if topic not in channels]
    if missing:
        raise ProbeError(f"bridge is up but never advertised: {missing}. Advertised: {sorted(channels)}")

    return {"name": server_info.get("name", ""), "capabilities": sorted(capabilities), "channels": channels}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument(
        "--path",
        default="/",
        help="request path; use /ws to probe through the foxglove_web_serve.py reverse proxy",
    )
    parser.add_argument("--expect", nargs="*", default=[], help="Topics that must be advertised")
    options = parser.parse_args(argv)

    try:
        result = probe(options.host, options.port, options.expect, options.timeout, options.path)
    except ProbeError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    print(f"OK: {result['name']}")
    print(f"    capabilities: {result['capabilities'] or ['(none: read-only)']}")
    print(f"    {len(result['channels'])} channels advertised")
    for topic in sorted(result["channels"]):
        print(f"      {topic}  [{result['channels'][topic]}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
