"""Safety-gated motion-search state machine.

The module is ROS independent on purpose: it can be unit-tested without a
robot.  A ROS adapter supplies the callbacks and must keep motion disabled
until ``motion_search_enabled`` is explicitly enabled.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Any


class SearchState(str, Enum):
    IDLE = "idle"
    SAFETY_CHECK = "safety_check"
    ROTATE = "rotate"
    SHORT_MOVE = "short_move"
    STOP_CONFIRM = "stop_confirm"
    VERIFY = "verify"
    ACCEPTED = "accepted"
    FAILED_STOP = "failed_stop"


@dataclass
class SearchConfig:
    enabled: bool = False
    rotate_steps_deg: tuple[float, ...] = (0.0, 45.0, -45.0, 90.0, -90.0, 135.0, -135.0, 180.0)
    move_distances_m: tuple[float, ...] = (0.3, 0.6, 1.0)
    stop_speed_mps: float = 0.03
    stop_yaw_rate_rps: float = 0.03


@dataclass
class SearchContext:
    state: SearchState = SearchState.IDLE
    phase: str = "rotate"
    candidate_index: int = 0
    attempts: list[dict[str, Any]] = field(default_factory=list)
    failure_reason: str | None = None


class RelocalizationMotionSearch:
    def __init__(self, config: SearchConfig, safety_check: Callable[[], tuple[bool, str]],
                 stop_motion: Callable[[], None], verify_candidate: Callable[[dict[str, Any]], dict[str, Any]]):
        self.config = config
        self.safety_check = safety_check
        self.stop_motion = stop_motion
        self.verify_candidate = verify_candidate
        self.context = SearchContext()

    def start(self) -> SearchContext:
        if not self.config.enabled:
            raise RuntimeError("motion search is disabled")
        self.context = SearchContext(state=SearchState.SAFETY_CHECK)
        ok, reason = self.safety_check()
        if not ok:
            return self.fail(reason)
        self.context.state = SearchState.ROTATE
        return self.context

    def submit_stop_sample(self, speed_mps: float, yaw_rate_rps: float) -> SearchContext:
        if self.context.state not in (SearchState.ROTATE, SearchState.SHORT_MOVE, SearchState.STOP_CONFIRM):
            return self.context
        if abs(speed_mps) > self.config.stop_speed_mps or abs(yaw_rate_rps) > self.config.stop_yaw_rate_rps:
            self.context.state = SearchState.STOP_CONFIRM
            return self.context
        self.context.state = SearchState.VERIFY
        return self.context

    def verify(self, candidate: dict[str, Any]) -> SearchContext:
        if self.context.state != SearchState.VERIFY:
            return self.context
        result = dict(self.verify_candidate(candidate) or {})
        result.setdefault("candidate", candidate)
        self.context.attempts.append(result)
        if bool(result.get("accepted")):
            self.context.state = SearchState.ACCEPTED
            return self.context
        return self._next_candidate()

    def fail(self, reason: str) -> SearchContext:
        self.stop_motion()
        self.context.failure_reason = reason
        self.context.state = SearchState.FAILED_STOP
        return self.context

    def _next_candidate(self) -> SearchContext:
        self.context.candidate_index += 1
        total = len(self.config.rotate_steps_deg)
        if self.context.phase == "rotate" and self.context.candidate_index < total:
            self.context.state = SearchState.ROTATE
            return self.context
        if self.context.phase == "rotate":
            self.context.phase = "short_move"
            self.context.candidate_index = 0
        if self.context.candidate_index < len(self.config.move_distances_m):
            self.context.state = SearchState.SHORT_MOVE
            return self.context
        return self.fail("all_motion_search_candidates_rejected")
