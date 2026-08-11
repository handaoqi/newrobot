from __future__ import annotations

from .realtime import event_broker


class RealtimePublisher:
    """Single-process P0 gateway; replace backend with Redis for multi-process deployment."""

    @staticmethod
    def publish_robot_status(robot_id: int, data: dict) -> None:
        event_broker.publish("robot_status", {"robot_id": robot_id, "data": data})

    @staticmethod
    def publish_task_event(execution_id: str, data: dict) -> None:
        event_broker.publish("task_event", {"execution_id": execution_id, "data": data})

    @staticmethod
    def publish_alert(data: dict) -> None:
        event_broker.publish("inspection_event_created", data)


realtime_publisher = RealtimePublisher()
