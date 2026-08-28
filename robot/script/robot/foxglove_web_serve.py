#!/usr/bin/env python3
"""Serve the vendored Lichtblick web bundle and proxy /ws to foxglove_bridge.

Upstream's own container image is `caddy file-server` over these same files, so
nothing more than a static server is required. This one adds two things that
matter here:

  * a /ws reverse proxy, so the read-only bridge can stay bound to 127.0.0.1 and
    only one port is ever exposed on the robot LAN;
  * --default-layout, which reproduces upstream's entrypoint trick of replacing
    the LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER comment inside index.html, so
    a layout is already loaded when the page opens instead of being imported by
    hand through the UI.

Standard library only: the NX has no web server installed and this is a debug
tool, not a reason to add a dependency to a robot.

    foxglove_web_serve.py --dist <dir> --port 8080 \
        --default-layout docs/yuwang/demo_patrol_layout.json
"""

from __future__ import annotations

import argparse
import errno
import json
import mimetypes
import os
import posixpath
import selectors
import socket
import sys
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# Python's mimetypes database predates wasm on many systems. Serving a .wasm as
# application/octet-stream makes the browser refuse to instantiate it and the app
# comes up blank, which reads exactly like a broken bundle. Pin the ones the
# bundle actually contains rather than trusting the system table.
EXPLICIT_TYPES = {
    ".wasm": "application/wasm",
    ".js": "text/javascript; charset=utf-8",
    ".mjs": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".html": "text/html; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".map": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".ttf": "font/ttf",
    ".woff": "font/woff",
    ".woff2": "font/woff2",
}

# Mirrors upstream's vercel.json. Cross-origin isolation (COOP/COEP) is
# deliberately not here: upstream's own caddy image sets neither and works, while
# COEP breaks any resource that lacks CORP. --cross-origin-isolated turns it on
# for anyone who needs SharedArrayBuffer.
BASE_HEADERS = {
    "X-Frame-Options": "DENY",
    "X-Content-Type-Options": "nosniff",
    "Referrer-Policy": "origin",
}
ISOLATION_HEADERS = {
    "Cross-Origin-Opener-Policy": "same-origin",
    "Cross-Origin-Embedder-Policy": "credentialless",
}

LAYOUT_PLACEHOLDER = "/*LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER*/"

# Headers that describe this specific hop and must not be forwarded verbatim.
HOP_BY_HOP = frozenset(
    {"connection", "keep-alive", "proxy-authenticate", "proxy-authorization", "te", "trailer", "upgrade"}
)


class Config:
    def __init__(
        self, dist: Path, index: bytes, bridge: tuple[str, int], ws_path: str, isolated: bool, verbose: bool
    ):
        self.dist = dist
        self.index = index
        self.bridge = bridge
        self.ws_path = ws_path
        self.isolated = isolated
        self.verbose = verbose


def build_index(dist: Path, layout_file: Path | None) -> bytes:
    """Read index.html, optionally injecting a default layout.

    The injection happens in memory. Upstream's entrypoint rewrites the file on
    disk, which would mean the vendored bundle no longer matches its checksum and
    a second run would inject into already-injected HTML.
    """
    source = dist / "index.html"
    if not source.is_file():
        raise SystemExit(f"no index.html in {dist}; run fetch_lichtblick_web.sh first")
    html = source.read_text(encoding="utf-8")
    if layout_file is None:
        return html.encode("utf-8")

    raw = layout_file.read_text(encoding="utf-8")
    try:
        layout = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise SystemExit(f"{layout_file} is not valid JSON: {exc}") from exc
    if LAYOUT_PLACEHOLDER not in html:
        raise SystemExit(
            f"index.html has no {LAYOUT_PLACEHOLDER}; this bundle does not support "
            "default-layout injection, import the layout through the UI instead"
        )
    # Re-serialised rather than pasted: this lands inside a <script> block, and
    # compact JSON with no stray newlines is what the placeholder expects.
    return html.replace(LAYOUT_PLACEHOLDER, json.dumps(layout, separators=(",", ":"))).encode("utf-8")


class Handler(BaseHTTPRequestHandler):
    server_version = "roamerx-foxglove-web"
    protocol_version = "HTTP/1.1"
    config: Config

    def log_message(self, fmt: str, *args) -> None:  # noqa: A002
        sys.stderr.write("[foxglove-web] %s %s\n" % (self.address_string(), fmt % args))

    def log_request(self, code="-", size="-") -> None:
        # Loading the app is ~400 requests. Logging each one buries the startup
        # banner and the proxy errors that actually need reading.
        if self.config.verbose:
            super().log_request(code, size)

    # --- static files ---------------------------------------------------------

    def _resolve(self, path: str) -> Path | None:
        """Map a URL path to a file inside dist, or None for the SPA fallback."""
        path = path.split("?", 1)[0].split("#", 1)[0]
        path = posixpath.normpath(posixpath.join("/", path))
        if path in ("/", "/index.html"):
            return None
        candidate = self.config.dist / path.lstrip("/")
        try:
            resolved = candidate.resolve(strict=True)
        except (OSError, RuntimeError):
            return None
        # Containment check against the resolved root: normpath alone does not
        # stop a symlink inside dist from pointing out of it.
        try:
            resolved.relative_to(self.config.dist)
        except ValueError:
            return None
        if not resolved.is_file():
            return None
        return resolved

    def _send_common_headers(self) -> None:
        for name, value in BASE_HEADERS.items():
            self.send_header(name, value)
        if self.config.isolated:
            for name, value in ISOLATION_HEADERS.items():
                self.send_header(name, value)

    def _serve_bytes(self, body: bytes, content_type: str, cacheable: bool) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        # Content-hashed chunk names make long caching safe; index.html carries
        # the injected layout and must never be cached.
        self.send_header("Cache-Control", "public, max-age=31536000, immutable" if cacheable else "no-store")
        self._send_common_headers()
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        if self._maybe_proxy():
            return
        target = self._resolve(self.path)
        if target is None:
            # SPA fallback. Everything the bundle needs is a real file, so a miss
            # is a client-side route, not a 404.
            self._serve_bytes(self.config.index, "text/html; charset=utf-8", cacheable=False)
            return
        suffix = target.suffix.lower()
        content_type = EXPLICIT_TYPES.get(suffix) or mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        try:
            body = target.read_bytes()
        except OSError as exc:
            self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, f"cannot read asset: {exc}")
            return
        self._serve_bytes(body, content_type, cacheable=True)

    def do_HEAD(self) -> None:  # noqa: N802
        self.do_GET()

    # --- websocket reverse proxy ---------------------------------------------

    def _maybe_proxy(self) -> bool:
        if self.path.split("?", 1)[0].rstrip("/") != self.config.ws_path.rstrip("/"):
            return False
        if self.headers.get("Upgrade", "").lower() != "websocket":
            self.send_error(HTTPStatus.BAD_REQUEST, "this endpoint only accepts websocket upgrades")
            return True
        self._proxy_websocket()
        return True

    def _proxy_websocket(self) -> None:
        host, port = self.config.bridge
        try:
            upstream = socket.create_connection((host, port), timeout=10)
        except OSError as exc:
            self.log_message("bridge unreachable at %s:%s: %s", host, port, exc)
            self.send_error(HTTPStatus.BAD_GATEWAY, f"foxglove_bridge is not reachable at {host}:{port}")
            return

        # Sec-WebSocket-Protocol has to survive untouched: foxglove_bridge 3.4.3
        # answers 400 to a handshake that does not offer foxglove.sdk.v1, so a
        # proxy that drops the header breaks every client.
        lines = [f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\n"]
        for name, value in self.headers.items():
            if name.lower() in HOP_BY_HOP or name.lower() == "host":
                continue
            lines.append(f"{name}: {value}\r\n")
        lines.append("Connection: Upgrade\r\nUpgrade: websocket\r\n\r\n")
        try:
            upstream.sendall("".join(lines).encode("latin-1"))
        except OSError as exc:
            upstream.close()
            self.send_error(HTTPStatus.BAD_GATEWAY, f"cannot forward the upgrade: {exc}")
            return

        # Relay the upstream response verbatim, then stop speaking HTTP. After a
        # 101 both directions are an opaque byte stream, so there is no reason to
        # parse websocket frames here -- and no risk of mangling them.
        try:
            head = b""
            while b"\r\n\r\n" not in head:
                chunk = upstream.recv(4096)
                if not chunk:
                    raise OSError("bridge closed during the upgrade")
                head += chunk
        except OSError as exc:
            upstream.close()
            self.send_error(HTTPStatus.BAD_GATEWAY, f"bridge refused the upgrade: {exc}")
            return

        header_blob, _, leftover = head.partition(b"\r\n\r\n")
        status_line = header_blob.split(b"\r\n", 1)[0]
        try:
            self.wfile.write(header_blob + b"\r\n\r\n")
            self.wfile.flush()
            if leftover:
                self.wfile.write(leftover)
                self.wfile.flush()
        except OSError:
            upstream.close()
            return

        if b" 101 " not in status_line:
            # Not an upgrade; the response is already relayed, so just hang up.
            upstream.close()
            self.close_connection = True
            return

        self._splice(self.connection, upstream)
        self.close_connection = True

    @staticmethod
    def _splice(client: socket.socket, upstream: socket.socket) -> None:
        """Pump bytes both ways until either side closes.

        Nothing here inspects self.rfile for buffered bytes. RFC 6455 forbids a
        client from sending frames before it has seen the 101, so by the time the
        upgrade is relayed the read buffer is empty by construction.

        Both sockets stay blocking. selectors tells us which one has data, so a
        recv after a readiness event returns promptly, and a blocking sendall
        applies natural backpressure instead of silently dropping a frame.
        """
        selector = selectors.DefaultSelector()
        selector.register(client, selectors.EVENT_READ, upstream)
        selector.register(upstream, selectors.EVENT_READ, client)
        try:
            while True:
                for key, _ in selector.select(timeout=300):
                    source: socket.socket = key.fileobj  # type: ignore[assignment]
                    sink: socket.socket = key.data
                    try:
                        data = source.recv(65536)
                    except OSError:
                        return
                    if not data:  # peer closed its side
                        return
                    try:
                        sink.sendall(data)
                    except OSError:
                        return
        finally:
            selector.close()
            try:
                upstream.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            upstream.close()


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request, client_address) -> None:
        # A browser closing a tab mid-transfer is normal, not a stack trace.
        exc = sys.exc_info()[1]
        if isinstance(exc, (BrokenPipeError, ConnectionResetError)):
            return
        if isinstance(exc, OSError) and exc.errno in (errno.EPIPE, errno.ECONNRESET):
            return
        super().handle_error(request, client_address)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--dist",
        default="/home/dogrobot/runtime/nx-edge/install/lichtblick-web/dist",
        help="unpacked Lichtblick web bundle",
    )
    parser.add_argument("--address", default="127.0.0.1", help="bind address; use 0.0.0.0 to expose on the LAN")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--bridge-host", default="127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=8765)
    parser.add_argument("--ws-path", default="/ws", help="path proxied to foxglove_bridge")
    parser.add_argument("--default-layout", default=None, help="layout JSON injected into index.html")
    parser.add_argument(
        "--cross-origin-isolated",
        action="store_true",
        help="send COOP/COEP headers (needed only for SharedArrayBuffer)",
    )
    parser.add_argument("--verbose", action="store_true", help="log every request")
    options = parser.parse_args(argv)

    if not 1 <= options.port <= 65535:
        parser.error(f"--port out of range: {options.port}")
    dist = Path(options.dist).resolve()
    if not dist.is_dir():
        print(
            f"bundle not found at {dist}\nRun: robot/script/robot/fetch_lichtblick_web.sh",
            file=sys.stderr,
        )
        return 2

    layout_file = Path(options.default_layout).resolve() if options.default_layout else None
    if layout_file is not None and not layout_file.is_file():
        print(f"layout not found: {layout_file}", file=sys.stderr)
        return 2

    config = Config(
        dist=dist,
        index=build_index(dist, layout_file),
        bridge=(options.bridge_host, options.bridge_port),
        ws_path=options.ws_path,
        isolated=options.cross_origin_isolated,
        verbose=options.verbose,
    )
    handler = type("BoundHandler", (Handler,), {"config": config})

    try:
        server = Server((options.address, options.port), handler)
    except OSError as exc:
        print(f"cannot bind {options.address}:{options.port}: {exc}", file=sys.stderr)
        return 2

    shown = "127.0.0.1" if options.address in ("0.0.0.0", "::") else options.address
    print(f"[foxglove-web] serving {dist} on http://{options.address}:{options.port}", file=sys.stderr)
    print(f"[foxglove-web] open  http://{shown}:{options.port}/", file=sys.stderr)
    print(
        f"[foxglove-web] websocket ws://{shown}:{options.port}{options.ws_path}"
        f" -> {options.bridge_host}:{options.bridge_port}",
        file=sys.stderr,
    )
    if layout_file is not None:
        print(f"[foxglove-web] default layout: {layout_file.name}", file=sys.stderr)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
