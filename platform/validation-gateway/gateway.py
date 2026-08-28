#!/usr/bin/env python3
"""Authenticated, read-only WebSocket proxy for ephemeral validation bridges."""

from __future__ import annotations

import asyncio
import ipaddress
import json
import os
import socket
import urllib.parse

import requests
import websockets
from websockets.exceptions import ConnectionClosed


PLATFORM_URL = os.environ.get("ROAMERX_PLATFORM_INTERNAL_URL", "http://backend:8000").rstrip("/")
GATEWAY_TOKEN = os.environ.get("ROAMERX_VALIDATION_GATEWAY_TOKEN", "")
LISTEN_HOST = os.environ.get("ROAMERX_VALIDATION_GATEWAY_HOST", "0.0.0.0")
LISTEN_PORT = int(os.environ.get("ROAMERX_VALIDATION_GATEWAY_PORT", "8765"))
ACTIVE_STATES = {"staging", "running", "analyzing", "uploading", "cancelling"}
FOXGLOVE_PROTOCOLS = ["foxglove.sdk.v1", "foxglove.websocket.v1"]


def _authorized_job(ticket: str) -> dict:
    if not GATEWAY_TOKEN:
        raise RuntimeError("gateway token is not configured")
    response = requests.post(
        f"{PLATFORM_URL}/api/internal/validation-live-ticket/verify/",
        json={"ticket": ticket},
        headers={"X-Validation-Gateway-Token": GATEWAY_TOKEN},
        timeout=10,
    )
    response.raise_for_status()
    return response.json()


def _validate_private_upstream(value: str) -> str:
    parsed = urllib.parse.urlsplit(value)
    if parsed.scheme not in {"ws", "wss"} or not parsed.hostname or parsed.path not in {"", "/"}:
        raise ValueError("invalid upstream URL")
    if parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise ValueError("invalid upstream URL")
    port = parsed.port or (443 if parsed.scheme == "wss" else 80)
    if port < 8800 or port > 9499:
        raise ValueError("upstream port is outside the validation range")
    addresses = socket.getaddrinfo(parsed.hostname, port, type=socket.SOCK_STREAM)
    if not addresses:
        raise ValueError("upstream does not resolve")
    for address in addresses:
        ip = ipaddress.ip_address(address[4][0])
        if not (ip.is_private or ip.is_loopback):
            raise ValueError("upstream must resolve to a private address")
    return value


async def _pump(source, destination) -> None:
    try:
        async for message in source:
            await destination.send(message)
    except ConnectionClosed:
        return


async def proxy(websocket, path: str) -> None:
    query = urllib.parse.parse_qs(urllib.parse.urlsplit(path).query)
    ticket = (query.get("ticket") or [""])[0]
    try:
        if not ticket or len(ticket) > 2048:
            raise ValueError("missing ticket")
        job = await asyncio.to_thread(_authorized_job, ticket)
        if job.get("state") not in ACTIVE_STATES:
            raise ValueError("job is not active")
        upstream_url = await asyncio.to_thread(_validate_private_upstream, job.get("live_bridge_url", ""))
    except Exception:
        await websocket.close(code=1008, reason="validation bridge authorization failed")
        return

    protocols = [websocket.subprotocol] if websocket.subprotocol else FOXGLOVE_PROTOCOLS
    try:
        async with websockets.connect(
            upstream_url,
            subprotocols=protocols,
            open_timeout=10,
            max_size=None,
            ping_interval=20,
        ) as upstream:
            first = asyncio.create_task(_pump(websocket, upstream))
            second = asyncio.create_task(_pump(upstream, websocket))
            done, pending = await asyncio.wait({first, second}, return_when=asyncio.FIRST_COMPLETED)
            for task in pending:
                task.cancel()
            await asyncio.gather(*done, *pending, return_exceptions=True)
    except Exception:
        await websocket.close(code=1011, reason="validation bridge unavailable")


async def main() -> None:
    async with websockets.serve(
        proxy,
        LISTEN_HOST,
        LISTEN_PORT,
        subprotocols=FOXGLOVE_PROTOCOLS,
        max_size=None,
        ping_interval=20,
    ):
        print(json.dumps({"event": "validation_gateway_ready", "port": LISTEN_PORT}), flush=True)
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
