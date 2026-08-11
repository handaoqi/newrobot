from __future__ import annotations

import argparse
import json
import logging
import ssl
import threading
from datetime import datetime, timezone
from pathlib import Path

import paho.mqtt.client as mqtt

from . import __version__
from .codex_runner import CodexRunner
from .config import DevAgentConfig
from .conversation_store import ConversationStore
from .protocol import DevTaskRequest, TaskMessageError

LOGGER = logging.getLogger(__name__)


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class DevAgentApplication:
    def __init__(self, config: DevAgentConfig) -> None:
        self.config = config
        self.runner = CodexRunner(config.codex)
        self.conversation = ConversationStore(config.conversation_file)
        self._sequence: dict[str, int] = {}
        self._task_lock = threading.Lock()
        self._known_tasks: set[str] = set()
        self._heartbeat_started = False
        self.log_dir = Path(config.log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"roamerx-dev-{config.robot_id}",
            protocol=mqtt.MQTTv5,
        )
        if config.mqtt.username:
            self.client.username_pw_set(config.mqtt.username, config.mqtt.password)
        if config.mqtt.ca_file:
            self.client.tls_set(
                ca_certs=config.mqtt.ca_file,
                certfile=config.mqtt.cert_file or None,
                keyfile=config.mqtt.key_file or None,
                tls_version=ssl.PROTOCOL_TLS_CLIENT,
            )
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.will_set(
            self._topic("dev/presence"),
            json.dumps({"status": "offline", "agent_version": __version__, "timestamp": now_iso()}),
            qos=1,
            retain=True,
        )

    def run(self) -> None:
        LOGGER.info("starting dev agent robot=%s", self.config.robot_id)
        self.client.connect(
            self.config.mqtt.host,
            self.config.mqtt.port,
            keepalive=self.config.mqtt.keepalive_seconds,
        )
        self.client.loop_forever(retry_first_connection=True)

    def _on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        if reason_code != 0:
            LOGGER.error("MQTT connection failed: %s", reason_code)
            return
        client.subscribe(self._topic("dev/tasks"), qos=1)
        client.subscribe(self._topic("dev/tasks/+/control"), qos=1)
        self._publish(
            self._topic("dev/presence"),
            {
                "status": "online",
                "agent_version": __version__,
                "codex_binary": self.config.codex.binary,
                "workspaces": sorted(self.config.workspaces),
                "timestamp": now_iso(),
            },
            retain=True,
        )
        if not self._heartbeat_started:
            self._heartbeat_started = True
            threading.Thread(target=self._heartbeat_loop, daemon=True, name="dev-heartbeat").start()
        LOGGER.info("connected to MQTT broker")

    def _on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties) -> None:
        LOGGER.warning("MQTT disconnected: %s", reason_code)

    def _heartbeat_loop(self) -> None:
        while True:
            threading.Event().wait(10)
            self._publish(
                self._topic("dev/presence"),
                {
                    "status": "online",
                    "agent_version": __version__,
                    "codex_binary": self.config.codex.binary,
                    "workspaces": sorted(self.config.workspaces),
                    "timestamp": now_iso(),
                },
                retain=True,
            )

    def _on_message(self, client, userdata, message) -> None:
        try:
            payload = json.loads(message.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            LOGGER.exception("invalid MQTT JSON topic=%s", message.topic)
            return
        if message.topic.endswith("/control"):
            self._handle_control(payload)
            return
        threading.Thread(
            target=self._handle_task,
            args=(payload,),
            daemon=True,
            name=f"dev-task-{payload.get('task_id', 'invalid')}",
        ).start()

    def _handle_task(self, payload: dict) -> None:
        try:
            task = DevTaskRequest.parse(payload)
        except TaskMessageError as exc:
            task_id = str(payload.get("task_id") or "invalid")
            self._publish_result(task_id, "rejected", error=str(exc))
            return
        if task.task_id in self._known_tasks:
            self._emit(task.task_id, "status", status="duplicate", text="任务已接收，忽略重复下发")
            return
        if task.workspace not in self.config.workspaces:
            self._publish_result(task.task_id, "rejected", error=f"unknown workspace: {task.workspace}")
            return
        if not self._task_lock.acquire(blocking=False):
            self._publish_result(task.task_id, "rejected", error="ROBOT_DEV_BUSY")
            return
        self._known_tasks.add(task.task_id)
        self._sequence[task.task_id] = 0
        try:
            self._emit(task.task_id, "status", status="running", text="Codex 已启动")
            session_id = self.conversation.thread_id()
            workspace_path = self.config.workspaces[task.workspace]
            effective_prompt = (
                f"本轮远程开发指定工作区：{workspace_path}\n"
                "执行前请进入该目录，并读取该目录适用的 AGENTS.md 等项目说明。\n\n"
                f"用户指令：\n{task.prompt}"
            )
            self._emit(
                task.task_id,
                "conversation",
                status="resuming" if session_id else "creating",
                text="继续统一 Codex 对话" if session_id else "创建统一 Codex 对话",
                codex_thread_id=session_id,
            )
            exit_code, last_message, cancelled = self.runner.run(
                task_id=task.task_id,
                prompt=effective_prompt,
                workspace=workspace_path,
                session_id=session_id,
                on_output=lambda stream, text, parsed: self._on_codex_output(
                    task.task_id, stream, text, parsed
                ),
            )
            status = "cancelled" if cancelled else ("succeeded" if exit_code == 0 else "failed")
            self._publish_result(task.task_id, status, exit_code=exit_code, last_message=last_message)
        except TimeoutError as exc:
            self._publish_result(task.task_id, "timed_out", error=str(exc))
        except Exception as exc:
            LOGGER.exception("Codex task failed id=%s", task.task_id)
            self._publish_result(task.task_id, "failed", error=str(exc))
        finally:
            self._task_lock.release()

    def _handle_control(self, payload: dict) -> None:
        task_id = str(payload.get("task_id") or "")
        if payload.get("action") != "cancel" or not task_id:
            return
        cancelled = self.runner.cancel(task_id)
        self._emit(
            task_id,
            "status",
            status="cancelling" if cancelled else "not_running",
            text="正在取消 Codex 任务" if cancelled else "任务当前未运行",
        )

    def _on_codex_output(self, task_id: str, stream: str, text: str, parsed: dict | None) -> None:
        if parsed and parsed.get("type") == "thread.started" and parsed.get("thread_id"):
            self.conversation.save(str(parsed["thread_id"]))
        event = {
            "task_id": task_id,
            "timestamp": now_iso(),
            "stream": stream,
            "text": text,
            "codex_event": parsed,
        }
        self._emit(
            task_id,
            "output",
            stream=stream,
            text=text,
            codex_event=parsed,
            codex_thread_id=self.conversation.thread_id(),
        )

    def _emit(self, task_id: str, event_type: str, **payload) -> None:
        sequence = self._sequence.get(task_id, 0) + 1
        self._sequence[task_id] = sequence
        event = {
            "task_id": task_id,
            "sequence": sequence,
            "type": event_type,
            "timestamp": now_iso(),
            **payload,
        }
        self._write_log(task_id, event)
        self._publish(self._topic(f"dev/tasks/{task_id}/events"), event)

    def _publish_result(self, task_id: str, status: str, **payload) -> None:
        result = {
            "task_id": task_id,
            "status": status,
            "timestamp": now_iso(),
            "codex_thread_id": self.conversation.thread_id(),
            **payload,
        }
        self._write_log(task_id, {"type": "result", **result})
        self._publish(self._topic(f"dev/tasks/{task_id}/result"), result)

    def _write_log(self, task_id: str, event: dict) -> None:
        safe_id = task_id if all(char.isalnum() or char in "-_" for char in task_id) else "invalid"
        path = self.log_dir / f"{safe_id}.jsonl"
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(event, ensure_ascii=False, default=str) + "\n")

    def _publish(self, topic: str, payload: dict, retain: bool = False) -> None:
        info = self.client.publish(
            topic,
            json.dumps(payload, ensure_ascii=False, default=str),
            qos=1,
            retain=retain,
        )
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            LOGGER.error("MQTT publish failed rc=%s topic=%s", info.rc, topic)

    def _topic(self, suffix: str) -> str:
        return f"robots/{self.config.robot_id}/{suffix}"


def main() -> None:
    parser = argparse.ArgumentParser(description="RoamerX remote Codex development agent")
    parser.add_argument("--edge-config", required=True)
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    config = DevAgentConfig.load(args.edge_config, args.config)
    DevAgentApplication(config).run()
