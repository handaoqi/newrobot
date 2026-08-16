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

# Public MCP values are always converted to the platform's canonical command
# vocabulary before the request leaves this process.  The Chinese and legacy
# spellings keep natural-language callers from accidentally receiving a vague
# "unsupported MCP parameter" response from the platform.
CONTROL_VALUE_ALIASES = {
    "direction": {
        "forward": "forward", "前进": "forward", "前行": "forward",
        "backward": "backward", "后退": "backward", "后移": "backward",
        "left": "left", "左移": "left",
        "right": "right", "右移": "right",
        "turn_left": "turn_left", "左转": "turn_left", "向左转": "turn_left",
        "turn_right": "turn_right", "右转": "turn_right", "向右转": "turn_right",
        "stop": "stop", "停止": "stop", "停下": "stop",
        "velocity": "velocity", "速度": "velocity", "速度控制": "velocity",
    },
    "speed": {
        "micro": "micro", "微速": "micro", "微速档": "micro",
        "low": "low", "低速": "low", "低速档": "low",
        "medium": "medium", "中速": "medium", "中速档": "medium", "normal": "medium",
        "high": "high", "高速": "high", "高速档": "high",
    },
    "action": {
        "stand_up": "stand_up", "stand": "stand_up", "起立": "stand_up", "站立": "stand_up",
        "prone": "prone", "lie_down": "prone", "趴下": "prone", "匍匐": "prone",
        "passive": "passive", "damping": "passive", "阻尼": "passive", "软急停": "passive",
        "motion_start": "motion_start", "启动运控": "motion_start", "启动运动": "motion_start",
        "motion_stop": "motion_stop", "停止运控": "motion_stop", "停止运动": "motion_stop",
    },
}


def canonical_control_value(category: str, value: str) -> str:
    """Map a released MCP value to its single platform representation."""
    raw = str(value or "").strip().lower()
    resolved = CONTROL_VALUE_ALIASES[category].get(raw)
    if resolved:
        return resolved
    supported = ", ".join(sorted(set(CONTROL_VALUE_ALIASES[category].values())))
    raise ValueError(f"不支持的 {category} 参数 {value!r}；可用值：{supported}")


def require_command_id(command_id: str, operation: str) -> str:
    resolved = str(command_id or "").strip()
    if not resolved:
        raise ValueError(f"{operation} 需要 command_id")
    return resolved


def resolve_robot_id(robot_id: int | None) -> int:
    """Return an explicit robot ID or the installation-wide default.

    The MCP schema keeps ``robot_id`` optional so a locally installed skill can
    operate the robot selected by its deployment.  An explicit ID always wins,
    which keeps multi-robot use possible without duplicating the server.
    """
    configured_id: int | str | None = robot_id
    if configured_id is None:
        configured_id = os.environ.get("ROAMERX_DEFAULT_ROBOT_ID", "").strip()
    if configured_id in (None, ""):
        raise ValueError(
            "robot_id is required; set ROAMERX_DEFAULT_ROBOT_ID for this MCP installation "
            "or provide robot_id explicitly"
        )
    if isinstance(configured_id, bool):
        raise ValueError("robot_id must be a positive platform robot ID")
    try:
        resolved_id = int(configured_id)
    except (TypeError, ValueError) as exc:
        raise ValueError("robot_id must be a positive platform robot ID") from exc
    if resolved_id <= 0:
        raise ValueError("robot_id must be a positive platform robot ID")
    return resolved_id


def configured_default_robot_id() -> int | None:
    """Return a valid optional default without blocking inventory discovery."""
    configured_id = os.environ.get("ROAMERX_DEFAULT_ROBOT_ID", "").strip()
    if not configured_id:
        return None
    try:
        return resolve_robot_id(None)
    except ValueError:
        return None


# The registry is the single customer-facing inventory of released MCP skills.
# Planned features deliberately do not appear here until their end-to-end path
# has been deployed and verified.
MCP_SKILL_REGISTRY = {
    "customer_summary": (
        "我可以通过云端安全链路远程操控机器狗：选择设备后，可起立或匍匐、启动或停止运控、"
        "切换微速/低速/中速/高速档、前后左右移动和左右转向，并可停止或进入阻尼。"
        "也可以查看平台已登记的设备及其平台 ID，以便明确选择目标机器狗。"
        "还可以开启人员识别、查看识别到的人员，针对指定 track_id 或画面中央人员执行持续本地视觉跟随，"
        "也支持执行预设组合动作；两者均可查询或停止。"
    ),
    "safety": [
        "回答功能咨询时只说明能力，不发送控制命令。",
        "所有控制均须由用户明确提出对应操作命令；不需要现场通道或行进路线的二次确认。",
        "组合动作须使用明确的预设或步骤；人员跟随须使用明确 track_id，或明确要求画面中央人员，不能根据含糊表述猜测目标。",
    ],
    "groups": [
        {
            "name": "平台设备",
            "tool": "robot_list_platform_devices",
            "buttons": [
                {"label": "查看设备列表", "read_only": True},
            ],
        },
        {
            "name": "基础遥控",
            "tool": "robot_direction",
            "buttons": [
                {"label": "前进", "direction": "forward"},
                {"label": "后退", "direction": "backward"},
                {"label": "左移", "direction": "left"},
                {"label": "右移", "direction": "right"},
                {"label": "左转", "direction": "turn_left"},
                {"label": "右转", "direction": "turn_right"},
                {"label": "停止", "direction": "stop"},
                {"label": "按速度控制", "direction": "velocity"},
            ],
        },
        {
            "name": "速度档位",
            "tool": "robot_speed",
            "buttons": [
                {"label": "微速", "level": "micro"},
                {"label": "低速", "level": "low"},
                {"label": "中速", "level": "medium"},
                {"label": "高速", "level": "high"},
            ],
        },
        {
            "name": "姿态与运控",
            "tool": "robot_action",
            "buttons": [
                {"label": "起立", "action": "stand_up"},
                {"label": "匍匐", "action": "prone"},
                {"label": "阻尼", "action": "passive"},
                {"label": "启动运控", "action": "motion_start"},
                {"label": "停止运控", "action": "motion_stop"},
            ],
        },
        {
            "name": "人员识别与跟随",
            "tools": [
                "robot_person_detection",
                "robot_person_detection_status",
                "robot_person_follow",
                "robot_person_follow_status",
                "robot_person_follow_stop",
            ],
            "buttons": [
                {"label": "开启/关闭跟踪识别", "enabled": True},
                {"label": "查看识别人员", "read_only": True},
                {"label": "跟随画面中央人员", "target": "center"},
                {"label": "按 track_id 开始持续跟随"},
                {"label": "查询/停止跟随"},
            ],
        },
        {
            "name": "组合动作",
            "tools": ["robot_skill_run", "robot_skill_status", "robot_skill_cancel"],
            "buttons": [
                {"label": "执行预设或步骤组合"},
                {"label": "查询组合动作状态", "read_only": True},
                {"label": "取消组合动作"},
            ],
        },
    ],
}

SERVER_INSTRUCTIONS = (
    "当用户询问‘你能做什么’、功能或按钮清单时，先调用 robot_remote_control_capabilities。"
    "当用户询问平台有哪些设备、某个 robot_id 对应哪台机器狗或要选择设备时，调用 "
    "robot_list_platform_devices；它是只读操作。"
    "回答咨询时不得发送控制命令。用户明确要求控制时直接调用对应工具，无需现场通道或行进路线的二次确认。"
    "人员跟随的视觉闭环仅在 Edge Agent 本地运行。"
    "所有控制均经云平台、MQTT 和 Edge Agent。"
)


class PlatformClient:
    def __init__(self, base_url: str = PLATFORM_URL, token: str = PLATFORM_TOKEN) -> None:
        self.base_url = base_url.rstrip("/")
        self.token = token

    def _headers(self) -> dict[str, str]:
        if not self.token:
            raise RuntimeError("ROAMERX_PLATFORM_TOKEN is required")
        return {"Authorization": f"Token {self.token}"}

    def command(self, robot_id: int | None, category: str, payload: dict[str, Any]) -> dict[str, Any]:
        resolved_id = resolve_robot_id(robot_id)
        response = requests.post(
            f"{self.base_url}/api/mcp/robots/{resolved_id}/{category}/",
            json=payload,
            headers=self._headers(),
            timeout=15,
        )
        try:
            response.raise_for_status()
        except requests.HTTPError as exc:
            raise RuntimeError(f"platform rejected command: {response.text}") from exc
        return response.json()

    def list_platform_devices(self) -> dict[str, Any]:
        """Return the safe device-selection fields from the platform inventory."""
        response = requests.get(
            f"{self.base_url}/api/robots/",
            headers=self._headers(),
            timeout=15,
        )
        try:
            response.raise_for_status()
        except requests.HTTPError as exc:
            raise RuntimeError(f"platform rejected device list request: {response.text}") from exc
        payload = response.json()
        if isinstance(payload, dict):
            payload = payload.get("results")
        if not isinstance(payload, list):
            raise RuntimeError("platform returned an invalid device list")

        devices = []
        for device in payload:
            if not isinstance(device, dict):
                continue
            robot_id = device.get("id")
            if isinstance(robot_id, bool) or not isinstance(robot_id, int) or robot_id <= 0:
                continue
            devices.append(
                {
                    "id": robot_id,
                    "code": str(device.get("code") or ""),
                    "name": str(device.get("name") or ""),
                    "location": str(device.get("location") or ""),
                    "area": str(device.get("area") or ""),
                    "connection_status": str(device.get("connection_status") or "unknown"),
                    "status": str(device.get("status") or "unknown"),
                    "battery_level": device.get("battery_level"),
                }
            )
        return {
            "devices": devices,
            "count": len(devices),
            "default_robot_id": configured_default_robot_id(),
        }

    def person_detection_status(self, robot_id: int | None) -> dict[str, Any]:
        return self.command(robot_id, "person-detection-status", {})

    def wait_for_command(self, robot_id: int | None, command: dict[str, Any], wait_seconds: float) -> dict[str, Any]:
        resolved_id = resolve_robot_id(robot_id)
        deadline = time.monotonic() + max(0.0, min(float(wait_seconds), 10.0))
        terminal = {"succeeded", "failed", "cancelled", "timed_out", "expired", "rejected"}
        latest = command
        while latest.get("status") not in terminal and time.monotonic() < deadline:
            time.sleep(0.3)
            response = requests.get(
                f"{self.base_url}/api/robots/{resolved_id}/commands/{command['id']}/",
                headers={"Authorization": f"Token {self.token}"}, timeout=10,
            )
            response.raise_for_status()
            latest = response.json()
        return {"command": latest, "completed": latest.get("status") in terminal}


client = PlatformClient()
mcp = FastMCP(
    "roamerx-robot-control",
    instructions=SERVER_INSTRUCTIONS,
    host=os.environ.get("ROAMERX_MCP_HOST", "127.0.0.1"),
    port=int(os.environ.get("ROAMERX_MCP_PORT", "8095")),
)


@mcp.tool(name="robot_direction")
def robot_direction(
    robot_id: int | None = None,
    direction: str = "",
    command: dict[str, float] | None = None,
) -> dict:
    """方向类；支持前进、后退、左右移动、左右转向、停止及速度控制。"""
    return client.command(
        robot_id,
        "direction",
        {"direction": canonical_control_value("direction", direction), "command": command or {}},
    )


@mcp.tool(name="robot_list_platform_devices")
def robot_list_platform_devices() -> dict[str, Any]:
    """读取平台设备列表和平台 robot_id；只读，不发送任何控制命令。"""
    return client.list_platform_devices()


@mcp.tool(name="robot_speed")
def robot_speed(robot_id: int | None = None, level: str = "") -> dict:
    """速度类：micro、low、medium、high；也接受中文档位名称。"""
    return client.command(robot_id, "speed", {"level": canonical_control_value("speed", level)})


@mcp.tool(name="robot_action")
def robot_action(robot_id: int | None = None, action: str = "") -> dict:
    """行动类；支持起立、匍匐/趴下、阻尼、启动或停止运控。"""
    return client.command(robot_id, "action", {"action": canonical_control_value("action", action)})


@mcp.tool(name="robot_skill_run")
def robot_skill_run(
    robot_id: int | None = None,
    preset: str = "",
    steps: list[dict[str, Any]] | None = None,
    description: str = "",
    wait_seconds: float = 10,
) -> dict:
    """高层技能：预置名称或 AI 翻译后的步骤；用户明确指定后直接执行。"""
    if not preset.strip() and not steps:
        raise ValueError("请提供 preset 或至少一个 steps 动作")
    payload: dict[str, Any] = {"description": description}
    if preset:
        payload["preset"] = preset
    if steps is not None:
        payload["steps"] = steps
    command = client.command(robot_id, "skill", payload)
    return client.wait_for_command(robot_id, command, wait_seconds)


@mcp.tool(name="robot_skill_status")
def robot_skill_status(robot_id: int | None = None, command_id: str = "") -> dict:
    """读取异步技能执行状态。"""
    return client.command(robot_id, "skill-status", {"command_id": require_command_id(command_id, "查询组合动作")})


@mcp.tool(name="robot_skill_cancel")
def robot_skill_cancel(robot_id: int | None = None, command_id: str = "") -> dict:
    """取消异步技能；Edge Agent 会立即下发停止速度。"""
    return client.command(robot_id, "skill-cancel", {"command_id": require_command_id(command_id, "取消组合动作")})


@mcp.tool(name="robot_person_detection")
def robot_person_detection(robot_id: int | None = None, enabled: bool = False) -> dict:
    """开启或关闭远控页的人员跟踪识别；此操作不移动机器狗。"""
    if not isinstance(enabled, bool):
        raise ValueError("enabled 必须是布尔值")
    return client.command(robot_id, "person-detection", {"enabled": enabled})


@mcp.tool(name="robot_person_detection_status")
def robot_person_detection_status(robot_id: int | None = None) -> dict:
    """读取人员识别开关、实时性及可选择跟随的人员 track_id；只读。"""
    return client.person_detection_status(robot_id)


@mcp.tool(name="robot_person_follow")
def robot_person_follow(
    robot_id: int | None = None,
    track_id: str = "",
    target: str = "",
    wait_seconds: float = 10,
) -> dict:
    """按 track_id 跟随，或以 target='center' 跟随最新画面中央的人员框。"""
    resolved_track_id = track_id.strip()
    resolved_target = target.strip().lower()
    if resolved_target:
        if resolved_target != "center" or resolved_track_id:
            raise ValueError("target 仅支持 center，且不能与 track_id 同时使用")
        payload = {"target": "center"}
    elif resolved_track_id:
        payload = {"track_id": resolved_track_id}
    else:
        raise ValueError("请提供 track_id，或将 target 设为 center 以跟随画面中央的人员")
    command = client.command(robot_id, "person-follow-start", payload)
    return client.wait_for_command(robot_id, command, wait_seconds)


@mcp.tool(name="robot_person_follow_status")
def robot_person_follow_status(robot_id: int | None = None, wait_seconds: float = 5) -> dict:
    """读取 Edge Agent 本地持续跟随状态与感知健康度。"""
    command = client.command(robot_id, "person-follow-status", {})
    return client.wait_for_command(robot_id, command, wait_seconds)


@mcp.tool(name="robot_person_follow_stop")
def robot_person_follow_stop(robot_id: int | None = None, wait_seconds: float = 5) -> dict:
    """停止持续人员跟随；狗端先发布零速度再返回结果。"""
    command = client.command(robot_id, "person-follow-stop", {})
    return client.wait_for_command(robot_id, command, wait_seconds)


@mcp.tool(name="robot_remote_control_capabilities")
def robot_remote_control_capabilities() -> dict[str, Any]:
    """客户询问“你能做什么”或遥控页有哪些功能时，返回完整且安全的功能清单；只读。"""
    return MCP_SKILL_REGISTRY


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--transport", choices=("stdio", "streamable-http"), default="stdio")
    args = parser.parse_args()
    mcp.run(transport=args.transport)


if __name__ == "__main__":
    main()
