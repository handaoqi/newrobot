from __future__ import annotations

import math
import uuid
from decimal import Decimal, InvalidOperation

from ..models import PatrolLoopSession, TaskExecution, TrajectoryPoint


DISTANCE_QUANTUM = Decimal("0.000001")
ZERO_DISTANCE = Decimal("0.000000")


def trajectory_distance(points) -> float:
    distance = 0.0
    for previous, current in zip(points, points[1:]):
        if (
            previous["map_id"]
            and current["map_id"]
            and str(previous["map_id"]) != str(current["map_id"])
        ):
            continue
        segment = math.hypot(
            float(current["x"]) - float(previous["x"]),
            float(current["y"]) - float(previous["y"]),
        )
        if math.isfinite(segment) and 0 <= segment <= 10:
            distance += segment
    return distance


def normalize_distance(value) -> Decimal | None:
    try:
        distance = Decimal(str(value))
    except (InvalidOperation, TypeError, ValueError):
        return None
    if not distance.is_finite() or distance < 0:
        return None
    return distance.quantize(DISTANCE_QUANTUM)


def stored_loop_total_distance(session: PatrolLoopSession) -> Decimal | None:
    return normalize_distance((session.metadata or {}).get("total_distance_m"))


def calculate_loop_total_distance(session_id) -> Decimal:
    points = (
        TrajectoryPoint.objects.filter(task_execution__loop_session_id=session_id)
        .order_by("task_execution_id", "seq")
        .values("task_execution_id", "map_id", "x", "y")
    )
    total = 0.0
    previous = None
    for current in points.iterator():
        if previous is None or previous["task_execution_id"] != current["task_execution_id"]:
            previous = current
            continue
        total += trajectory_distance((previous, current))
        previous = current
    return normalize_distance(total) or ZERO_DISTANCE


def update_patrol_loop_distance(
    session: PatrolLoopSession | None,
    execution: TaskExecution,
    batch_id: uuid.UUID,
) -> None:
    if session is None:
        return

    stored_total = stored_loop_total_distance(session)
    if stored_total is None:
        updated = calculate_loop_total_distance(session.id)
    else:
        point_fields = ("seq", "map_id", "x", "y", "batch_id")
        execution_points = TrajectoryPoint.objects.filter(task_execution=execution)
        inserted = list(
            execution_points.filter(batch_id=batch_id).order_by("seq").values(*point_fields)
        )
        if not inserted:
            return

        first_seq = inserted[0]["seq"]
        last_seq = inserted[-1]["seq"]
        previous = (
            execution_points.filter(seq__lt=first_seq)
            .order_by("-seq")
            .values(*point_fields)
            .first()
        )
        following = (
            execution_points.filter(seq__gt=last_seq)
            .order_by("seq")
            .values(*point_fields)
            .first()
        )
        lower_seq = previous["seq"] if previous else first_seq
        upper_seq = following["seq"] if following else last_seq
        points = list(
            execution_points.filter(seq__gte=lower_seq, seq__lte=upper_seq)
            .order_by("seq")
            .values(*point_fields)
        )

        # Rebuild only the affected batch-sized slice. This remains correct
        # when offline batches arrive late/out of order or overlap stored seqs.
        before = [point for point in points if point["batch_id"] != batch_id]
        delta = trajectory_distance(points) - trajectory_distance(before)
        updated = normalize_distance(max(Decimal("0"), stored_total + Decimal(str(delta))))
        updated = updated or ZERO_DISTANCE

    metadata = dict(session.metadata or {})
    metadata["total_distance_m"] = format(updated, "f")
    session.metadata = metadata
    session.save(update_fields=["metadata", "updated_at"])
