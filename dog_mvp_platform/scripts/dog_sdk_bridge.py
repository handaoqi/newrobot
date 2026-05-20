#!/usr/bin/env python3
import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class RobotBridge:
    def __init__(self, sdk_lib, local_ip, local_port, dog_ip):
        sys.path.insert(0, sdk_lib)
        from mc_sdk_zsl_1_py import HighLevel

        self.app = HighLevel()
        self.app.initRobot(local_ip, local_port, dog_ip)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if self.app.checkConnect():
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("SDK did not connect within 3s")

        self.lock = threading.Lock()
        self.running = True
        self.standing = False
        self.mode = "idle"
        self.linear = 0.0
        self.angular = 0.0
        self.deadline = 0.0
        self.pending_move = None
        self.stand_ready_at = 0.0
        self.last_stand_command = 0.0
        self.last_error = ""
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            now = time.time()
            try:
                with self.lock:
                    mode = self.mode
                    standing = self.standing
                    pending_move = self.pending_move
                    stand_ready_at = self.stand_ready_at
                    linear = self.linear
                    angular = self.angular
                    deadline = self.deadline

                if pending_move and not standing:
                    if now - self.last_stand_command > 0.7:
                        self.app.standUp()
                        self.last_stand_command = now
                    with self.lock:
                        self.standing = True
                        self.mode = "standing"
                        self.stand_ready_at = max(self.stand_ready_at, now + 1.6)
                    time.sleep(0.05)
                    continue

                if pending_move and now >= stand_ready_at:
                    with self.lock:
                        linear, angular, duration = self.pending_move
                        self.pending_move = None
                        self.linear = linear
                        self.angular = angular
                        self.deadline = time.time() + duration
                        self.mode = "move"
                    continue

                if mode == "stand":
                    if now - self.last_stand_command > 0.7:
                        self.app.standUp()
                        self.last_stand_command = now
                    with self.lock:
                        self.standing = True
                        self.mode = "standing"
                        self.stand_ready_at = max(self.stand_ready_at, now + 1.6)
                    time.sleep(0.05)
                    continue

                if standing and now < stand_ready_at:
                    if now - self.last_stand_command > 0.7:
                        self.app.standUp()
                        self.last_stand_command = now
                    time.sleep(0.05)
                    continue

                if mode == "move" and now < deadline:
                    self.app.move(linear, 0.0, angular)
                elif standing:
                    self.app.move(0.0, 0.0, 0.0)
                    with self.lock:
                        if self.mode == "move":
                            self.mode = "standing"
                        self.linear = 0.0
                        self.angular = 0.0
                time.sleep(0.05)
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc)
                time.sleep(0.2)

    def stand(self):
        with self.lock:
            self.mode = "stand"
            self.pending_move = None
        return self.status()

    def move(self, linear, angular, duration):
        linear = max(-0.2, min(float(linear), 0.2))
        angular = max(-0.6, min(float(angular), 0.6))
        duration = max(0.1, min(float(duration), 2.0))
        with self.lock:
            if self.standing and time.time() >= self.stand_ready_at:
                self.linear = linear
                self.angular = angular
                self.deadline = time.time() + duration
                self.mode = "move"
            else:
                self.pending_move = (linear, angular, duration)
                self.mode = "standing"
        return self.status(extra={"accepted_move": [linear, angular, duration]})

    def stop(self):
        with self.lock:
            self.pending_move = None
            self.linear = 0.0
            self.angular = 0.0
            self.deadline = 0.0
            if self.standing:
                self.mode = "standing"
        return self.status(extra={"stopped": True})

    def status(self, extra=None):
        with self.lock:
            data = {
                "connected": bool(self.app.checkConnect()),
                "battery": self.app.getBatteryPower(),
                "ctrlmode": self.app.getCurrentCtrlmode(),
                "standing": self.standing,
                "mode": self.mode,
                "linear": self.linear,
                "angular": self.angular,
                "deadline": self.deadline,
                "pending_move": bool(self.pending_move),
                "stand_ready_in": max(0.0, self.stand_ready_at - time.time()),
                "rpy": list(self.app.getRPY()),
                "world_velocity": list(self.app.getWorldVelocity()),
                "last_error": self.last_error,
            }
        if extra:
            data.update(extra)
        return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-lib", required=True)
    parser.add_argument("--local-ip", default="192.168.234.1")
    parser.add_argument("--local-port", type=int, default=43988)
    parser.add_argument("--dog-ip", default="192.168.234.1")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9095)
    args = parser.parse_args()

    bridge = RobotBridge(args.sdk_lib, args.local_ip, args.local_port, args.dog_ip)

    class Handler(BaseHTTPRequestHandler):
        def _send(self, payload, status=200):
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _body(self):
            length = int(self.headers.get("Content-Length", "0") or "0")
            if length <= 0:
                return {}
            return json.loads(self.rfile.read(length).decode("utf-8"))

        def do_GET(self):
            if self.path == "/status":
                self._send({"ok": True, "sdk": bridge.status()})
                return
            self._send({"ok": False, "error": "not found"}, 404)

        def do_POST(self):
            try:
                if self.path == "/stand":
                    self._send({"ok": True, "sdk": bridge.stand()})
                    return
                if self.path == "/move":
                    body = self._body()
                    self._send({
                        "ok": True,
                        "sdk": bridge.move(
                            body.get("linear", 0.0),
                            body.get("angular", 0.0),
                            body.get("duration", 0.5),
                        ),
                    })
                    return
                if self.path == "/stop":
                    self._send({"ok": True, "sdk": bridge.stop()})
                    return
                self._send({"ok": False, "error": "not found"}, 404)
            except Exception as exc:
                self._send({"ok": False, "error": str(exc), "sdk": bridge.status()}, 500)

        def log_message(self, fmt, *args):
            return

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"dog sdk bridge listening on {args.host}:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
