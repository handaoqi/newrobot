"""Frame-agnostic comparison of the two gyroscopes on the robot.

The robot carries two usable IMUs: the one built into the Livox Mid-360
(published on ``/front_lidar/imu`` in ``livox_frame``) and the BMI088 on the
3588 motion controller (carried inside ``/highlevel_robotstate`` in the body
frame). The localization stack only ever uses the first one.

Comparing them properly would need the ``base_link -> livox_frame`` extrinsic,
which is exactly the kind of dependency that turns a monitoring feature into a
calibration project. So this compares ``||omega||`` instead: the norm of the
angular velocity is invariant under the rotation between the two frames, so any
difference in it is a real disagreement between the sensors rather than a
mounting artefact. A lever arm does not affect angular velocity at all.

Two regimes are distinguished because they mean different things:

* **Stationary** (both norms tiny): whatever each sensor still reports is its
  gyro bias. The measured gap here - 0.740 deg/s on the Livox against
  0.285 deg/s on the BMI088 - is the thing that drives the yaw drift, so it is
  worth surfacing on its own rather than averaging into the moving case.
* **Moving**: both should track the same body rotation. A sustained gap means
  one of them is scaling wrong or dropping samples.

This module is deliberately free of ROS imports so it can be unit tested.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

_RAD_TO_DEG = 180.0 / math.pi


@dataclass
class ImuCrossCheckConfig:
    """Thresholds for the dual-IMU comparison.

    Defaults are deliberately loose. They exist to catch a sensor that has
    genuinely failed or come loose, not to grade calibration quality - real
    values should be set from field data before anyone tightens them.
    """

    enabled: bool = True
    robot_state_topic: str = "/highlevel_robotstate"
    lidar_imu_topic: str = "/front_lidar/imu"
    window_seconds: float = 5.0
    min_samples: int = 20
    evaluate_interval_seconds: float = 5.0
    #: A body stream older than this counts as absent rather than disagreeing.
    body_stale_seconds: float = 3.0
    #: Below this norm both sensors are treated as standing still.
    stationary_rate_dps: float = 2.0
    #: Sustained disagreement while moving.
    max_rate_delta_dps: float = 3.0
    #: Disagreement between the two resting biases.
    max_static_bias_gap_dps: float = 1.0
    #: Consecutive bad evaluations before anything is reported.
    mismatch_samples: int = 3


class ImuCrossCheck:
    """Rolling-window comparison of two gyro streams.

    Not thread safe on its own; the caller is expected to hold a lock, which is
    cheap because both feeds arrive on the same ROS executor.
    """

    def __init__(self, config: ImuCrossCheckConfig | None = None):
        self.config = config or ImuCrossCheckConfig()
        # Timestamps are arrival times from a monotonic clock, not header
        # stamps: the 3588 bridge does not share a clock discipline with the
        # NX, and a window this wide does not need stamp-level alignment.
        self._lidar: deque[tuple[float, float]] = deque()
        self._body: deque[tuple[float, float]] = deque()
        self._last_evaluated_at: float | None = None
        self._mismatch_streak = 0

    @staticmethod
    def magnitude_dps(x: float, y: float, z: float) -> float:
        """Angular rate norm in deg/s. Both feeds publish rad/s."""
        return math.sqrt(float(x) ** 2 + float(y) ** 2 + float(z) ** 2) * _RAD_TO_DEG

    def add_lidar_gyro(self, at: float, x: float, y: float, z: float) -> None:
        self._append(self._lidar, at, x, y, z)

    def add_body_gyro(self, at: float, x: float, y: float, z: float) -> None:
        self._append(self._body, at, x, y, z)

    def _append(self, window: deque, at: float, x: float, y: float, z: float) -> None:
        window.append((at, self.magnitude_dps(x, y, z)))
        self._trim(window, at)

    def _trim(self, window: deque, now: float) -> None:
        horizon = now - self.config.window_seconds
        while window and window[0][0] < horizon:
            window.popleft()

    def due(self, now: float) -> bool:
        """Rate limit: the comparison is a monitor, not a control loop."""
        if self._last_evaluated_at is None:
            return True
        return now - self._last_evaluated_at >= self.config.evaluate_interval_seconds

    def evaluate(self, now: float) -> dict:
        """Compare the two windows. Always returns a report, never raises."""
        self._last_evaluated_at = now
        self._trim(self._lidar, now)
        self._trim(self._body, now)

        lidar_mean = _mean(self._lidar)
        body_mean = _mean(self._body)
        report = {
            "lidar_topic": self.config.lidar_imu_topic,
            "body_topic": self.config.robot_state_topic,
            "lidar_samples": len(self._lidar),
            "body_samples": len(self._body),
            "lidar_rate_dps": _round(lidar_mean),
            "body_rate_dps": _round(body_mean),
            "window_seconds": self.config.window_seconds,
        }

        body_age = now - self._body[-1][0] if self._body else None
        if not self._body or (body_age is not None and body_age > self.config.body_stale_seconds):
            # The expected state today: the ecal2ros bridge does not publish
            # this topic yet. Absent is not the same as mismatched, and must
            # never latch an alert.
            report["status"] = "body_stream_absent"
            report["body_age_seconds"] = _round(body_age) if body_age is not None else None
            self._mismatch_streak = 0
            return report

        if len(self._lidar) < self.config.min_samples or len(self._body) < self.config.min_samples:
            report["status"] = "insufficient_samples"
            self._mismatch_streak = 0
            return report

        delta = lidar_mean - body_mean
        stationary = (
            lidar_mean <= self.config.stationary_rate_dps
            and body_mean <= self.config.stationary_rate_dps
        )
        limit = (
            self.config.max_static_bias_gap_dps
            if stationary
            else self.config.max_rate_delta_dps
        )
        report["delta_dps"] = _round(delta)
        report["stationary"] = stationary
        report["threshold_dps"] = limit
        if stationary:
            # Resting norms are the biases themselves - the number the drift
            # investigation actually cares about.
            report["lidar_static_bias_dps"] = _round(lidar_mean)
            report["body_static_bias_dps"] = _round(body_mean)

        if abs(delta) > limit:
            self._mismatch_streak += 1
        else:
            self._mismatch_streak = 0
        report["mismatch_streak"] = self._mismatch_streak
        report["status"] = (
            "mismatch"
            if self._mismatch_streak >= max(1, self.config.mismatch_samples)
            else "ok"
        )
        return report


def _mean(window: deque) -> float:
    if not window:
        return 0.0
    return sum(value for _at, value in window) / len(window)


def _round(value: float | None) -> float | None:
    return None if value is None else round(float(value), 4)
