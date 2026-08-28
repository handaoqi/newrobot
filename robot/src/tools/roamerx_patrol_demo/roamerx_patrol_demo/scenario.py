"""Pure deterministic indoor-patrol model.

The model deliberately has no ROS dependency.  Keeping trajectory generation
pure makes the demo reproducible and lets the contract tests run on machines
where ROS is not installed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import re
from typing import Iterable, Sequence


DEFAULT_WAYPOINTS: tuple[tuple[float, float], ...] = (
    (7.0, 1.0),
    (7.0, 6.0),
    (2.0, 6.0),
    (2.0, 3.0),
)
START = (1.0, 1.0)
MAP_WIDTH_M = 10.0
MAP_HEIGHT_M = 8.0
EPOCH_NS = 1_700_000_000_000_000_000
_NAMESPACE_RE = re.compile(r"^(/[A-Za-z0-9_.-]+)*$")


@dataclass(frozen=True)
class PatrolConfig:
    duration_sec: float = 120.0
    publish_rate_hz: float = 50.0
    linear_speed_mps: float = 0.35
    angular_speed_rps: float = 0.6
    topic_namespace: str = ""
    waypoints: Sequence[tuple[float, float]] = field(default_factory=lambda: DEFAULT_WAYPOINTS)
    seed: int = 20260828
    publish_camera: bool = True
    publish_scan: bool = True
    publish_imu: bool = True

    def __post_init__(self) -> None:
        if self.duration_sec <= 0:
            raise ValueError("duration_sec must be greater than zero")
        if self.publish_rate_hz <= 0:
            raise ValueError("publish_rate_hz must be greater than zero")
        if self.linear_speed_mps <= 0:
            raise ValueError("linear_speed_mps must be greater than zero")
        if self.angular_speed_rps <= 0:
            raise ValueError("angular_speed_rps must be greater than zero")
        if not _NAMESPACE_RE.fullmatch(self.topic_namespace):
            raise ValueError("topic_namespace must contain only ROS namespace characters")
        if len(self.waypoints) < 4:
            raise ValueError("waypoints must contain at least four patrol points")
        for point in self.waypoints:
            if len(point) != 2 or not all(math.isfinite(float(value)) for value in point):
                raise ValueError("each waypoint must be a finite x,y pair")
            x, y = (float(value) for value in point)
            if not 0.0 <= x <= MAP_WIDTH_M or not 0.0 <= y <= MAP_HEIGHT_M:
                raise ValueError(
                    f"waypoint ({x:g}, {y:g}) is outside the {MAP_WIDTH_M:g}x{MAP_HEIGHT_M:g} m demo map"
                )
        if int(self.seed) != self.seed:
            raise ValueError("seed must be an integer")


@dataclass(frozen=True)
class RobotSample:
    time_ns: int
    x: float
    y: float
    yaw: float
    speed_mps: float
    angular_speed_rps: float
    phase: str
    waypoint_index: int
    diagnostic_level: int
    frame_id: str = "base_link"
    odom_frame: str = "odom"


@dataclass(frozen=True)
class _Segment:
    start: tuple[float, float]
    end: tuple[float, float]
    start_sec: float
    end_sec: float
    index: int
    phase: str = "TRAVEL"


class PatrolScenario:
    """A fixed route with a P2 pause and deterministic obstacle detour."""

    def __init__(self, config: PatrolConfig):
        self.config = config
        self._segments = self._make_segments()

    @property
    def epoch_ns(self) -> int:
        return EPOCH_NS

    def _make_segments(self) -> tuple[_Segment, ...]:
        points = (START, *tuple((float(x), float(y)) for x, y in self.config.waypoints), START)
        # Route timing is normalized to the requested duration.  The explicit
        # pause and detour are fixed fractions so changing duration preserves
        # the event order and remains deterministic.
        pause = min(5.0, self.config.duration_sec * 0.08)
        detour = min(10.0, self.config.duration_sec * 0.12)
        travel_budget = self.config.duration_sec - pause
        distances = [math.dist(points[i], points[i + 1]) for i in range(len(points) - 1)]
        total = sum(distances)
        available = max(travel_budget - detour, 1.0)
        cursor = 0.0
        result: list[_Segment] = []
        for i, ((start, end), distance) in enumerate(zip(zip(points, points[1:]), distances)):
            duration = available * distance / total
            phase = "OBSTACLE_AVOIDANCE" if i == 2 else "TRAVEL"
            result.append(_Segment(start, end, cursor, cursor + duration, i, phase))
            cursor += duration
            if i == 1:
                result.append(_Segment(end, end, cursor, cursor + pause, 2, "PAUSE_P2"))
                cursor += pause
            if i == 2:
                # Insert a triangular detour.  The exact detour is represented
                # as two sub-segments while retaining the same phase label.
                midpoint = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0 + 0.7)
                first = result.pop()
                half = detour / 2.0
                result.append(_Segment(first.start, midpoint, first.start_sec, first.end_sec, i, phase))
                result.append(_Segment(midpoint, first.end, first.end_sec, first.end_sec + detour, i, phase))
                cursor += detour
        # Stretch the final segment endpoint to exactly duration to prevent
        # floating point drift from changing the completion boundary.
        if result:
            last = result[-1]
            result[-1] = _Segment(last.start, last.end, last.start_sec, self.config.duration_sec, last.index, last.phase)
        return tuple(result)

    def _segment_at(self, elapsed_sec: float) -> _Segment | None:
        for segment in self._segments:
            if elapsed_sec <= segment.end_sec:
                return segment
        return None

    def sample(self, elapsed_ns: int) -> RobotSample:
        if elapsed_ns < 0:
            raise ValueError("elapsed_ns must not be negative")
        elapsed_sec = min(float(elapsed_ns) / 1_000_000_000.0, self.config.duration_sec)
        if elapsed_sec >= self.config.duration_sec:
            return RobotSample(
                EPOCH_NS + int(self.config.duration_sec * 1_000_000_000),
                START[0], START[1], 0.0, 0.0, 0.0, "COMPLETED", len(self.config.waypoints), 0
            )
        segment = self._segment_at(elapsed_sec)
        if segment is None:
            return RobotSample(EPOCH_NS + elapsed_ns, START[0], START[1], 0.0, 0.0, 0.0, "COMPLETED", len(self.config.waypoints), 0)
        span = max(segment.end_sec - segment.start_sec, 1e-9)
        ratio = min(max((elapsed_sec - segment.start_sec) / span, 0.0), 1.0)
        x = segment.start[0] + (segment.end[0] - segment.start[0]) * ratio
        y = segment.start[1] + (segment.end[1] - segment.start[1]) * ratio
        dx = segment.end[0] - segment.start[0]
        dy = segment.end[1] - segment.start[1]
        yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-9 else 0.0
        speed = 0.0 if segment.phase == "PAUSE_P2" else self.config.linear_speed_mps
        diagnostic = 1 if segment.phase == "OBSTACLE_AVOIDANCE" else 0
        return RobotSample(
            EPOCH_NS + int(elapsed_sec * 1_000_000_000),
            x,
            y,
            yaw,
            speed,
            0.0,
            segment.phase,
            min(segment.index + 1, len(self.config.waypoints)),
            diagnostic,
        )

    def trajectory_until(self, elapsed_ns: int, max_points: int = 5000) -> tuple[RobotSample, ...]:
        if max_points <= 0:
            return ()
        end_ns = max(0, min(elapsed_ns, int(self.config.duration_sec * 1_000_000_000)))
        step_ns = max(100_000_000, end_ns // max_points if end_ns else 100_000_000)
        times = range(0, end_ns + 1, step_ns)
        samples = [self.sample(value) for value in times]
        if samples and samples[-1].time_ns != self.sample(end_ns).time_ns:
            samples.append(self.sample(end_ns))
        return tuple(samples[-max_points:])
