from __future__ import annotations

import json
import queue
import threading
from collections.abc import Iterator
from typing import Any


class EventBroker:
    def __init__(self) -> None:
        self._subscribers: set[queue.Queue[dict[str, Any]]] = set()
        self._lock = threading.Lock()

    def publish(self, event_type: str, payload: dict[str, Any]) -> None:
        message = {"type": event_type, **payload}
        with self._lock:
            subscribers = list(self._subscribers)

        for subscriber in subscribers:
            try:
                subscriber.put_nowait(message)
            except queue.Full:
                self.unsubscribe(subscriber)

    def subscribe(self) -> queue.Queue[dict[str, Any]]:
        subscriber: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=20)
        with self._lock:
            self._subscribers.add(subscriber)
        return subscriber

    def unsubscribe(self, subscriber: queue.Queue[dict[str, Any]]) -> None:
        with self._lock:
            self._subscribers.discard(subscriber)


event_broker = EventBroker()


def sse_stream(subscriber: queue.Queue[dict[str, Any]]) -> Iterator[str]:
    yield "event: connected\ndata: {}\n\n"
    while True:
        try:
            message = subscriber.get(timeout=25)
        except queue.Empty:
            yield ": keepalive\n\n"
            continue

        event_type = message.get("type", "message")
        data = json.dumps(message, ensure_ascii=False, default=str)
        yield f"event: {event_type}\ndata: {data}\n\n"
