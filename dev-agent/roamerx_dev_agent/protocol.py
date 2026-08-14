from __future__ import annotations

import uuid
from dataclasses import dataclass


DEFAULT_CODEX_MODEL = "gpt-5.6-terra"
ALLOWED_CODEX_MODELS = frozenset({
    "gpt-5.6-sol",
    "gpt-5.6-terra",
    "gpt-5.6-luna",
})


class TaskMessageError(ValueError):
    pass


@dataclass(frozen=True)
class DevTaskRequest:
    task_id: str
    prompt: str
    workspace: str
    conversation_id: str = "main"
    model: str = DEFAULT_CODEX_MODEL

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
        return cls(
            task_id=task_id,
            prompt=prompt,
            workspace=workspace,
            conversation_id=conversation_id,
            model=model,
        )
