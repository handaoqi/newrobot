from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class MqttConfig:
    host: str
    port: int
    username: str = ""
    password: str = ""
    ca_file: str = ""
    cert_file: str = ""
    key_file: str = ""
    keepalive_seconds: int = 20


@dataclass(frozen=True)
class CodexConfig:
    binary: str
    home: str
    yolo: bool = True
    timeout_seconds: int = 7200


@dataclass(frozen=True)
class DevAgentConfig:
    robot_id: str
    mqtt: MqttConfig
    codex: CodexConfig
    workspaces: dict[str, str]
    log_dir: str
    conversation_file: str

    @classmethod
    def load(cls, edge_config_path: str, dev_config_path: str) -> "DevAgentConfig":
        edge = yaml.safe_load(Path(edge_config_path).read_text(encoding="utf-8")) or {}
        dev = yaml.safe_load(Path(dev_config_path).read_text(encoding="utf-8")) or {}
        mqtt = edge.get("mqtt") or {}
        codex = dev.get("codex") or {}
        workspaces = dev.get("workspaces") or {}
        if not edge.get("robot", {}).get("id"):
            raise ValueError("robot.id is missing from edge config")
        if not mqtt.get("host"):
            raise ValueError("mqtt.host is missing from edge config")
        if not workspaces:
            raise ValueError("at least one workspace must be configured")
        resolved_workspaces = {}
        for name, raw_path in workspaces.items():
            path = Path(raw_path).expanduser().resolve()
            if not path.is_dir():
                raise ValueError(f"workspace does not exist: {name}={path}")
            resolved_workspaces[str(name)] = str(path)
        return cls(
            robot_id=str(edge["robot"]["id"]),
            mqtt=MqttConfig(
                host=str(mqtt["host"]),
                port=int(mqtt.get("port", 8883)),
                username=str(mqtt.get("username") or ""),
                password=str(mqtt.get("password") or ""),
                ca_file=str(mqtt.get("ca_file") or ""),
                cert_file=str(mqtt.get("cert_file") or ""),
                key_file=str(mqtt.get("key_file") or ""),
                keepalive_seconds=int(mqtt.get("keepalive_seconds", 20)),
            ),
            codex=CodexConfig(
                binary=str(codex.get("binary") or "/usr/local/bin/codex"),
                home=str(codex.get("home") or "/home/robot/.codex"),
                yolo=bool(codex.get("yolo", True)),
                timeout_seconds=max(30, int(codex.get("timeout_seconds", 7200))),
            ),
            workspaces=resolved_workspaces,
            log_dir=str(Path((dev.get("storage") or {}).get("log_dir") or "data/tasks").expanduser()),
            conversation_file=str(
                Path(
                    (dev.get("storage") or {}).get("conversation_file")
                    or "data/conversation.json"
                ).expanduser()
            ),
        )
