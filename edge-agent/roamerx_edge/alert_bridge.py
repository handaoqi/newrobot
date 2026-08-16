from __future__ import annotations

import uuid

from .protocol import now_iso


class AlertBridge:
    def __init__(self, telemetry, task_executor, publish_alert) -> None:
        self.telemetry = telemetry
        self.task_executor = task_executor
        self.publish_alert = publish_alert

    def emit_system_alert(self, event_type: str, severity: str, source_code: str, attributes=None) -> str:
        return self._emit(event_type, severity, {"component": "edge_agent", "code": source_code}, attributes=attributes)

    def emit_detection_alert(self, detection: dict, media: dict | None = None) -> str:
        return self._emit(
            detection["event_type"],
            detection.get("severity", "medium"),
            {
                "component": detection.get("component", "detector"),
                "model_version": detection.get("model_version", ""),
            },
            detection=detection,
            media=media,
        )

    def _emit(self, event_type, severity, source, detection=None, media=None, attributes=None) -> str:
        event_id = str(uuid.uuid4())
        pose = self.telemetry.latest_pose()
        context = self.task_executor.context
        payload = {
            "event_id": event_id,
            "event_type": event_type,
            "severity": severity,
            "occurred_at": now_iso(),
            "task_execution_id": context.task_execution_id if context else None,
            "map_id": self.telemetry.robot.current_map_id,
            "map_version": self.telemetry.robot.current_map_version,
            "pose": {
                "frame_id": "map",
                "x": pose.x if pose else None,
                "y": pose.y if pose else None,
                "yaw": pose.yaw if pose else None,
            },
            "source": source,
            "attributes": attributes or {},
        }
        if detection:
            payload["detection"] = detection
        if media:
            payload["media"] = media
        self.publish_alert(payload)
        return event_id
