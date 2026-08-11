from __future__ import annotations

import json
import threading
from pathlib import Path


class ConversationStore:
    def __init__(self, path: str) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def thread_id(self) -> str | None:
        with self._lock:
            try:
                payload = json.loads(self.path.read_text(encoding="utf-8"))
            except (FileNotFoundError, json.JSONDecodeError, OSError):
                return None
            value = str(payload.get("thread_id") or "").strip()
            return value or None

    def save(self, thread_id: str) -> None:
        payload = json.dumps({"thread_id": thread_id}, ensure_ascii=False, indent=2)
        temporary = self.path.with_suffix(f"{self.path.suffix}.tmp")
        with self._lock:
            temporary.write_text(payload + "\n", encoding="utf-8")
            temporary.replace(self.path)
