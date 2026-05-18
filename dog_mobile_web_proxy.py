#!/usr/bin/env python3
import base64
import http.server
import os
import urllib.error
import urllib.request


TARGET = os.environ.get("DOG_WEB_TARGET", "http://127.0.0.1:8765")
USER = os.environ.get("DOG_WEB_USER", "demo")
PASSWORD = os.environ.get("DOG_WEB_PASSWORD")
PORT = int(os.environ.get("DOG_WEB_PROXY_PORT", "8876"))


class Proxy(http.server.BaseHTTPRequestHandler):
    def _authorized(self):
        header = self.headers.get("Authorization", "")
        expected = "Basic " + base64.b64encode(f"{USER}:{PASSWORD}".encode()).decode()
        return header == expected

    def _require_auth(self):
        self.send_response(401)
        self.send_header("WWW-Authenticate", 'Basic realm="Dog Demo"')
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.end_headers()
        self.wfile.write("Authentication required".encode("utf-8"))

    def _proxy(self):
        if not self._authorized():
            self._require_auth()
            return

        body = None
        if self.command in {"POST", "PUT", "PATCH"}:
            length = int(self.headers.get("Content-Length", "0") or "0")
            body = self.rfile.read(length) if length else None

        url = TARGET + self.path
        headers = {
            key: value
            for key, value in self.headers.items()
            if key.lower() not in {"host", "authorization", "connection", "content-length"}
        }
        request = urllib.request.Request(url, data=body, headers=headers, method=self.command)

        try:
            with urllib.request.urlopen(request, timeout=120) as response:
                payload = response.read()
                self.send_response(response.status)
                for key, value in response.headers.items():
                    if key.lower() not in {"connection", "transfer-encoding"}:
                        self.send_header(key, value)
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(payload)
        except urllib.error.HTTPError as exc:
            payload = exc.read()
            self.send_response(exc.code)
            self.send_header("Content-Type", exc.headers.get("Content-Type", "text/plain; charset=utf-8"))
            self.end_headers()
            self.wfile.write(payload)
        except Exception as exc:
            payload = f"Proxy error: {exc}".encode("utf-8")
            self.send_response(502)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

    def do_GET(self):
        self._proxy()

    def do_POST(self):
        self._proxy()

    def log_message(self, fmt, *args):
        print("%s - %s" % (self.address_string(), fmt % args), flush=True)


def main():
    if not PASSWORD:
        raise SystemExit("DOG_WEB_PASSWORD must be set before starting the proxy")
    server = http.server.ThreadingHTTPServer(("127.0.0.1", PORT), Proxy)
    print(f"Dog mobile web proxy on http://127.0.0.1:{PORT} -> {TARGET}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
