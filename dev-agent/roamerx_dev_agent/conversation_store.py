from __future__ import annotations

import json
import threading
from pathlib import Path


class ConversationStore:
    DEFAULT_CONVERSATION_ID = "main"
    DEFAULT_CONVERSATION_NAME = "主会话"

    def __init__(self, path: str) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def _path_for(self, conversation_id: str) -> Path:
        if conversation_id == self.DEFAULT_CONVERSATION_ID:
            return self.path
        return self.path.with_name(f"{self.path.stem}.{conversation_id}{self.path.suffix}")

    def thread_id(self, conversation_id: str = DEFAULT_CONVERSATION_ID) -> str | None:
        target = self._path_for(conversation_id)
        with self._lock:
            try:
                payload = json.loads(target.read_text(encoding="utf-8"))
            except (FileNotFoundError, json.JSONDecodeError, OSError):
                return None
            value = str(payload.get("thread_id") or "").strip()
            return value or None

    def sessions(self) -> list[dict[str, str]]:
        items = [{"id": self.DEFAULT_CONVERSATION_ID, "name": self.DEFAULT_CONVERSATION_NAME}]
        for path in sorted(self.path.parent.glob(f"{self.path.stem}.*{self.path.suffix}")):
            conversation_id = path.stem[len(self.path.stem) + 1:]
            if conversation_id:
                items.append({"id": conversation_id, "name": conversation_id})
        return items

    def save(self, thread_id: str, conversation_id: str = DEFAULT_CONVERSATION_ID) -> None:
        target = self._path_for(conversation_id)
        payload = json.dumps({"thread_id": thread_id}, ensure_ascii=False, indent=2)
        temporary = target.with_suffix(f"{target.suffix}.tmp")
        with self._lock:
            temporary.write_text(payload + "\n", encoding="utf-8")
            temporary.replace(target)

    def clear(self, conversation_id: str = DEFAULT_CONVERSATION_ID) -> None:
        """Forget a broken Codex thread; its immutable rollout file is retained."""
        target = self._path_for(conversation_id)
        with self._lock:
            try:
                target.unlink()
            except FileNotFoundError:
                pass
