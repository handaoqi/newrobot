from __future__ import annotations

import uuid
from dataclasses import dataclass


DEFAULT_CODEX_MODEL = "gpt-5.6-terra"
ALLOWED_CODEX_MODELS = frozenset({
    "gpt-5.6-sol",
    "gpt-5.6-terra",
    "gpt-5.6-luna",
})
ALLOWED_EXECUTION_MODES = frozenset({"plan", "execute"})
_WAKE_COMMAND_PREFIX_PUNCTUATION = "，,。！？!?：:；;"


class TaskMessageError(ValueError):
    pass


def extract_wake_command(transcript: str, wake_name: str = "小太阳") -> str | None:
    """Return the operation after an explicit wake name, otherwise ``None``.

    Speech recognition commonly inserts spaces or Chinese punctuation after the
    wake name, so both are ignored. The wake name must be the first spoken
    token; phrases merely mentioning it are not commands.
    """
    name = str(wake_name or "").strip()
    text = str(transcript or "").strip()
    if not name or not text.startswith(name):
        return None
    command = text[len(name):].lstrip().lstrip(_WAKE_COMMAND_PREFIX_PUNCTUATION).strip()
    return command or None


@dataclass(frozen=True)
class DevTaskRequest:
    task_id: str
    prompt: str
    workspace: str
    conversation_id: str = "main"
    model: str = DEFAULT_CODEX_MODEL
    execution_mode: str = "execute"

    @classmethod
    def parse(cls, payload: dict) -> "DevTaskRequest":
        try:
            task_id = str(uuid.UUID(str(payload["task_id"])))
        except (KeyError, TypeError, ValueError) as exc:
            raise TaskMessageError("task_id must be a UUID") from exc
        prompt = str(payload.get("prompt") or "").strip()
        workspace = str(payload.get("workspace") or "").strip()
        conversation_id = str(payload.get("conversation_id") or "main").strip()
        model = str(payload.get("model") or DEFAULT_CODEX_MODEL).strip()
        execution_mode = str(payload.get("execution_mode") or "execute").strip()
        if not prompt:
            raise TaskMessageError("prompt is required")
        if len(prompt) > 50000:
            raise TaskMessageError("prompt is too long")
        if not workspace:
            raise TaskMessageError("workspace is required")
        if not conversation_id.replace("_", "").replace("-", "").isalnum() or len(conversation_id) > 64:
            raise TaskMessageError("conversation_id is invalid")
        if model not in ALLOWED_CODEX_MODELS:
            raise TaskMessageError("model is not allowed")
        if execution_mode not in ALLOWED_EXECUTION_MODES:
            raise TaskMessageError("execution_mode is not allowed")
        return cls(
            task_id=task_id,
            prompt=prompt,
            workspace=workspace,
            conversation_id=conversation_id,
            model=model,
            execution_mode=execution_mode,
        )
