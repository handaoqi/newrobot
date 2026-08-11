import argparse
import http.client
import os
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit


HOP_BY_HOP_HEADERS = {
    "connection",
    "keep-alive",
    "proxy-authenticate",
    "proxy-authorization",
    "te",
    "trailer",
    "transfer-encoding",
    "upgrade",
}


class AppHandler(SimpleHTTPRequestHandler):
    backend_url = urlsplit("http://127.0.0.1:8000")

    def do_GET(self):
        if self.path.startswith(("/api/", "/media/")):
            self.proxy_to_backend()
            return
        super().do_GET()

    def do_POST(self):
        self.proxy_to_backend()

    def do_PUT(self):
        self.proxy_to_backend()

    def do_PATCH(self):
        self.proxy_to_backend()

    def do_DELETE(self):
        self.proxy_to_backend()

    def send_head(self):
        path = self.translate_path(self.path)
        if os.path.isdir(path):
            path = os.path.join(path, "index.html")
        if not os.path.exists(path):
            self.path = "/index.html"
        return super().send_head()

    def proxy_to_backend(self):
        body = None
        content_length = self.headers.get("Content-Length")
        if content_length:
            body = self.rfile.read(int(content_length))

        conn = http.client.HTTPConnection(
            self.backend_url.hostname,
            self.backend_url.port or 80,
            timeout=120,
        )
        headers = {
            key: value
            for key, value in self.headers.items()
            if key.lower() not in HOP_BY_HOP_HEADERS and key.lower() != "host"
        }
        headers["Host"] = self.backend_url.netloc
        headers["X-Forwarded-Proto"] = "http"
        headers["X-Forwarded-Host"] = self.headers.get("Host", "")

        try:
            conn.request(self.command, self.path, body=body, headers=headers)
            response = conn.getresponse()
            self.send_response(response.status, response.reason)
            for key, value in response.getheaders():
                if key.lower() not in HOP_BY_HOP_HEADERS:
                    self.send_header(key, value)
            self.end_headers()
            while True:
                chunk = response.read(64 * 1024)
                if not chunk:
                    break
                self.wfile.write(chunk)
                self.wfile.flush()
        finally:
            conn.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=3000)
    parser.add_argument("--directory", default="dist")
    parser.add_argument("--backend", default="http://127.0.0.1:8000")
    args = parser.parse_args()

    handler = lambda *handler_args, **handler_kwargs: AppHandler(
        *handler_args,
        directory=args.directory,
        **handler_kwargs,
    )
    AppHandler.backend_url = urlsplit(args.backend)

    with ThreadingHTTPServer((args.host, args.port), handler) as server:
        print(f"Serving {args.directory} on http://{args.host}:{args.port}")
        print(f"Proxying /api and /media to {args.backend}")
        server.serve_forever()


if __name__ == "__main__":
    main()
