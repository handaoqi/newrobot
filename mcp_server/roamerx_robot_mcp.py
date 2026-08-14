#!/usr/bin/env python3
"""RoamerX remote-control MCP server."""
from __future__ import annotations

import argparse
import os
import time
from typing import Any

import requests
from mcp.server.fastmcp import FastMCP


PLATFORM_URL = os.environ.get("ROAMERX_PLATFORM_URL", "http://39.107.250.69:8088").rstrip("/")
PLATFORM_TOKEN = os.environ.get("ROAMERX_PLATFORM_TOKEN", "")


class PlatformClient:
    def __init__(self, base_url: str = PLATFORM_URL, token: str = PLATFORM_TOKEN) -> None:
        self.base_url = base_url.rstrip("/")
        self.token = token

    def command(self, robot_id: int, category: str, payload: dict[str, Any]) -> dict[str, Any]:
        if not self.token:
            raise RuntimeError("ROAMERX_PLATFORM_TOKEN is required")
        response = requests.post(
            f"{self.base_url}/api/mcp/robots/{robot_id}/{category}/",
            json=payload,
            headers={"Authorization": f"Token {self.token}"},
            timeout=15,
        )
        try:
            response.raise_for_status()
        except requests.HTTPError as exc:
            raise RuntimeError(f"platform rejected command: {response.text}") from exc
        return response.json()

    def wait_for_command(self, robot_id: int, command: dict[str, Any], wait_seconds: float) -> dict[str, Any]:
        deadline = time.monotonic() + max(0.0, min(float(wait_seconds), 10.0))
        terminal = {"succeeded", "failed", "cancelled", "timed_out", "expired", "rejected"}
        latest = command
        while latest.get("status") not in terminal and time.monotonic() < deadline:
            time.sleep(0.3)
            response = requests.get(
                f"{self.base_url}/api/robots/{robot_id}/commands/{command['id']}/",
                headers={"Authorization": f"Token {self.token}"}, timeout=10,
            )
            response.raise_for_status()
            latest = response.json()
        return {"command": latest, "completed": latest.get("status") in terminal}


client = PlatformClient()
mcp = FastMCP(
    "roamerx-robot-control",
    host=os.environ.get("ROAMERX_MCP_HOST", "127.0.0.1"),
    port=int(os.environ.get("ROAMERX_MCP_PORT", "8095")),
)


@mcp.tool(name="robot_direction")
def robot_direction(robot_id: int, direction: str, command: dict[str, float] | None = None) -> dict:
    """方向类：forward/backward/left/right/turn_left/turn_right/stop/velocity。"""
    return client.command(robot_id, "direction", {"direction": direction, "command": command or {}})


@mcp.tool(name="robot_speed")
def robot_speed(robot_id: int, level: str) -> dict:
    """速度类：micro、low、medium、high。"""
    return client.command(robot_id, "speed", {"level": level})


@mcp.tool(name="robot_action")
def robot_action(robot_id: int, action: str) -> dict:
    """行动类：stand_up、prone、passive、motion_start、motion_stop。"""
    return client.command(robot_id, "action", {"action": action})


@mcp.tool(name="robot_skill_run")
def robot_skill_run(robot_id: int, preset: str = "", steps: list[dict[str, Any]] | None = None, description: str = "", wait_seconds: float = 10) -> dict:
    """高层技能：预置名称或 AI 翻译后的步骤；description 仅记录自然语言意图。"""
    payload: dict[str, Any] = {"description": description}
    if preset:
        payload["preset"] = preset
    if steps is not None:
        payload["steps"] = steps
    command = client.command(robot_id, "skill", payload)
    return client.wait_for_command(robot_id, command, wait_seconds)


@mcp.tool(name="robot_skill_status")
def robot_skill_status(robot_id: int, command_id: str) -> dict:
    """读取异步技能执行状态。"""
    return client.command(robot_id, "skill-status", {"command_id": command_id})


@mcp.tool(name="robot_skill_cancel")
def robot_skill_cancel(robot_id: int, command_id: str) -> dict:
    """取消异步技能；Edge Agent 会立即下发停止速度。"""
    return client.command(robot_id, "skill-cancel", {"command_id": command_id})


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--transport", choices=("stdio", "streamable-http"), default="stdio")
    args = parser.parse_args()
    mcp.run(transport=args.transport)


if __name__ == "__main__":
    main()
