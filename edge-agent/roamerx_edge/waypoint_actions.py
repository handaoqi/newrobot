"""Idempotent waypoint action registry executed after arrival confirmation."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Callable


LOGGER = logging.getLogger(__name__)

SUPPORTED_ACTIONS = {"snapshot"}


@dataclass(frozen=True)
class ActionResult:
    action: str
    status: str
    message: str = ""
    duration_seconds: float = 0.0


class WaypointActionRegistry:
    """Execute registered waypoint actions with timeout and idempotent keys."""

    def __init__(self) -> None:
        self._handlers: dict[str, Callable[[dict], None]] = {
            "snapshot": self._handle_snapshot,
        }
        self._completed: set[str] = set()

    def reset(self) -> None:
        self._completed.clear()

    def validate(self, actions: list) -> None:
        unknown = [action for action in actions if action not in self._handlers]
        if unknown:
            raise ValueError(f"unsupported waypoint actions: {unknown}")

    def execute(
        self,
        *,
        actions: list,
        idempotency_key: str,
        timeout_seconds: float = 5.0,
        context: dict | None = None,
    ) -> list[ActionResult]:
        if not actions:
            return []
        if idempotency_key in self._completed:
            return [
                ActionResult(action=str(action), status="skipped_duplicate")
                for action in actions
            ]
        results: list[ActionResult] = []
        for action in actions:
            handler = self._handlers.get(str(action))
            if handler is None:
                results.append(ActionResult(action=str(action), status="rejected", message="unknown action"))
                continue
            started = time.monotonic()
            try:
                handler(dict(context or {}))
                elapsed = time.monotonic() - started
                if elapsed > timeout_seconds:
                    results.append(
                        ActionResult(
                            action=str(action),
                            status="timeout",
                            message=f"exceeded {timeout_seconds:.1f}s",
                            duration_seconds=elapsed,
                        )
                    )
                else:
                    results.append(
                        ActionResult(
                            action=str(action),
                            status="succeeded",
                            duration_seconds=elapsed,
                        )
                    )
            except Exception as exc:
                LOGGER.warning("waypoint action %s failed: %s", action, exc, exc_info=True)
                results.append(
                    ActionResult(
                        action=str(action),
                        status="failed",
                        message=str(exc),
                        duration_seconds=time.monotonic() - started,
                    )
                )
        self._completed.add(idempotency_key)
        return results

    def _handle_snapshot(self, context: dict) -> None:
        # Snapshot is recorded by the platform/camera pipeline when the arrival
        # event is published. Edge only validates and marks the action complete.
        LOGGER.info(
            "waypoint snapshot action acknowledged waypoint_id=%s",
            context.get("waypoint_id"),
        )
