from __future__ import annotations

import uuid
from dataclasses import dataclass


class TaskMessageError(ValueError):
    pass


@dataclass(frozen=True)
class DevTaskRequest:
    task_id: str
    prompt: str
    workspace: str

    @classmethod
    def parse(cls, payload: dict) -> "DevTaskRequest":
        try:
            task_id = str(uuid.UUID(str(payload["task_id"])))
        except (KeyError, TypeError, ValueError) as exc:
            raise TaskMessageError("task_id must be a UUID") from exc
        prompt = str(payload.get("prompt") or "").strip()
        workspace = str(payload.get("workspace") or "").strip()
        if not prompt:
            raise TaskMessageError("prompt is required")
        if len(prompt) > 50000:
            raise TaskMessageError("prompt is too long")
        if not workspace:
            raise TaskMessageError("workspace is required")
        return cls(task_id=task_id, prompt=prompt, workspace=workspace)
