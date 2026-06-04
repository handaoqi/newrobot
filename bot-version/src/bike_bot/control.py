from __future__ import annotations

import argparse
import json
import logging
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock
from typing import Any

from .config import AppConfig

LOGGER = logging.getLogger(__name__)


class RobotActionController:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self._lock = Lock()
        self._sdk_app: Any | None = None

    def execute(self, action: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        if action not in self._action_map():
            raise ValueError(f"unsupported action: {action}")

        if self.config.control.dry_run:
            LOGGER.info("dry-run robot action action=%s payload=%s", action, payload or {})
            return {"ok": True, "action": action, "dry_run": True}

        app = self._get_sdk_app()
        sdk_method = self._action_map()[action]
        with self._lock:
            result = sdk_method(app, payload or {})
        LOGGER.info("robot action executed action=%s result=%s", action, result)
        return {"ok": True, "action": action, "dry_run": False, "sdk_result": result}

    def _get_sdk_app(self):
        if self._sdk_app is not None:
            return self._sdk_app
        if not self.config.control.sdk_enabled:
            raise RuntimeError("robot SDK is disabled")

        if self.config.control.sdk_lib_path:
            sdk_path = str(Path(self.config.control.sdk_lib_path).resolve())
            if sdk_path not in sys.path:
                sys.path.insert(0, sdk_path)

        import mc_sdk_zsl_1_py  # type: ignore

        app = mc_sdk_zsl_1_py.HighLevel()
        app.initRobot(
            self.config.control.local_ip,
            self.config.control.local_port,
            self.config.control.robot_ip,
        )
        self._sdk_app = app
        return app

    @staticmethod
    def _action_map():
        return {
            "shake_hand": lambda app, _payload: app.shakeHand(),
            "stand_up": lambda app, _payload: app.standUp(),
            "lie_down": lambda app, _payload: app.lieDown(),
            "passive": lambda app, _payload: app.passive(),
            "move_forward": lambda app, payload: app.move(float(payload.get("vx", 0.35)), 0.0, 0.0),
            "move_backward": lambda app, payload: app.move(float(payload.get("vx", -0.35)), 0.0, 0.0),
            "move_left": lambda app, payload: app.move(0.0, float(payload.get("vy", 0.25)), 0.0),
            "move_right": lambda app, payload: app.move(0.0, float(payload.get("vy", -0.25)), 0.0),
            "turn_left": lambda app, payload: app.move(0.0, 0.0, float(payload.get("yaw_rate", 0.45))),
            "turn_right": lambda app, payload: app.move(0.0, 0.0, float(payload.get("yaw_rate", -0.45))),
            "move_stop": lambda app, _payload: app.move(0.0, 0.0, 0.0),
            "jump": lambda app, _payload: app.jump(),
            "front_jump": lambda app, _payload: app.frontJump(),
            "backflip": lambda app, _payload: app.backflip(),
            "two_leg_stand": lambda app, payload: app.twoLegStand(
                float(payload.get("vx", 0.0)),
                float(payload.get("yaw_rate", 0.0)),
            ),
            "cancel_two_leg_stand": lambda app, _payload: app.cancelTwoLegStand(),
            "attitude_control": lambda app, payload: app.attitudeControl(
                float(payload.get("roll_vel", 0.0)),
                float(payload.get("pitch_vel", 0.0)),
                float(payload.get("yaw_vel", 0.0)),
                float(payload.get("height_vel", 0.0)),
            ),
        }


class CommandRequestHandler(BaseHTTPRequestHandler):
    server_version = "BikeBotCommandServer/1.0"

    def do_POST(self) -> None:
        if self.path.rstrip("/") != "/commands":
            self._send_json(404, {"detail": "not found"})
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            raw_body = self.rfile.read(content_length)
            payload = json.loads(raw_body.decode("utf-8")) if raw_body else {}
            self._handle_command(payload)
        except json.JSONDecodeError:
            self._send_json(400, {"detail": "invalid JSON"})
        except ValueError as exc:
            self._send_json(400, {"detail": str(exc)})
        except Exception as exc:
            LOGGER.exception("command execution failed")
            self._send_json(500, {"detail": str(exc)})

    def log_message(self, format: str, *args: Any) -> None:
        LOGGER.info("command_server " + format, *args)

    def _handle_command(self, payload: dict[str, Any]) -> None:
        config: AppConfig = self.server.config  # type: ignore[attr-defined]
        controller: RobotActionController = self.server.controller  # type: ignore[attr-defined]
        robot_code = payload.get("robot_code")
        if robot_code != config.robot.code:
            raise ValueError(f"robot_code mismatch: expected {config.robot.code}, got {robot_code}")

        action = payload.get("action")
        if not action:
            raise ValueError("action is required")

        result = controller.execute(action, payload.get("payload") or {})
        self._send_json(
            200,
            {
                "detail": "command accepted",
                "command_id": payload.get("command_id"),
                "robot_code": config.robot.code,
                **result,
            },
        )

    def _send_json(self, status_code: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class CommandServer:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.controller = RobotActionController(config)
        self.httpd = ThreadingHTTPServer((config.control.host, config.control.port), CommandRequestHandler)
        self.httpd.config = config
        self.httpd.controller = self.controller

    def serve_forever(self) -> None:
        LOGGER.info(
            "starting command server host=%s port=%s dry_run=%s",
            self.config.control.host,
            self.config.control.port,
            self.config.control.dry_run,
        )
        self.httpd.serve_forever(poll_interval=0.5)

    def shutdown(self) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bike bot command server.")
    parser.add_argument("--config", default="config.yaml", help="path to yaml config")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s [%(name)s] %(message)s")
    config = AppConfig.from_file(args.config)
    server = CommandServer(config)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        LOGGER.info("command server shutdown requested")
    finally:
        server.shutdown()
