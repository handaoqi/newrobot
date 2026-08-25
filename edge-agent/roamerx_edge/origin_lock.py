from __future__ import annotations

import math
import logging
import os
import tempfile
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import yaml

LOGGER = logging.getLogger(__name__)


class OriginSignalUnavailable(RuntimeError):
    """A required RTK stream is missing or stale for a known duration."""

    def __init__(self, message: str, unavailable_seconds: float = 0.0) -> None:
        super().__init__(message)
        self.unavailable_seconds = max(0.0, float(unavailable_seconds))


@dataclass
class OriginSample:
    latitude: float
    longitude: float
    altitude: float
    position_fixed: bool
    heading_fixed: bool
    baseline_m: float
    heading_deg: float
    heading_std_deg: float
    horizontal_std_m: float
    age_seconds: float
    ntrip_quality: str = ""
    fix_status: int = -1
    solution_status: int = -1
    position_type: int = 0
    solution_satellites: int = 0
    vertical_std_m: float | None = None
    differential_age_seconds: float | None = None
    message_time_offset_seconds: float | None = None
    measurement_time_source: str = ""
    measurement_stamp: float | None = None
    ntrip_state: str = ""
    tracking_satellites: int = 0
    heading_status: int = -1
    heading_type: int = 0
    heading_satellites: int = 0
    heading_solution_satellites: int = 0
    pitch_deg: float = 0.0
    pitch_std_deg: float = 0.0
    sampled_at: float = 0.0

    def __post_init__(self) -> None:
        if not self.sampled_at:
            self.sampled_at = time.time()


class OriginLockMonitor:
    """Continuously quality-gates an RTK anchor and publishes a stable snapshot."""

    def __init__(
        self,
        sample_provider: Callable[[], OriginSample],
        origin_file: str,
        *,
        duration_seconds: float = 10.0,
        max_spread_m: float = 0.02,
        sample_interval_seconds: float = 1.0,
        min_baseline_m: float = 0.20,
        max_heading_std_deg: float = 5.0,
        max_age_seconds: float = 1.5,
        no_signal_timeout_seconds: float = 3.0,
        heading_offset_deg: float = 0.0,
    ) -> None:
        self.sample_provider = sample_provider
        self.origin_file = Path(origin_file).expanduser()
        self.duration_seconds = max(1.0, float(duration_seconds))
        self.max_spread_m = float(max_spread_m)
        self.sample_interval_seconds = max(0.1, float(sample_interval_seconds))
        self.min_baseline_m = float(min_baseline_m)
        self.max_heading_std_deg = float(max_heading_std_deg)
        self.max_age_seconds = float(max_age_seconds)
        self.no_signal_timeout_seconds = max(0.1, float(no_signal_timeout_seconds))
        self.heading_offset_deg = float(heading_offset_deg)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._generation = 0
        self._samples: deque[OriginSample] = deque()
        self._fix_wait_started_monotonic = 0.0
        self._position_fix_seen = False
        self._status: dict = self._empty_status()

    def _empty_status(self) -> dict:
        return {
            "origin_lock_session_id": None,
            "origin_status": "idle",
            "quality_started_at": None,
            "continuous_seconds": 0.0,
            "required_seconds": self.duration_seconds,
            "lock_wait_seconds": 0.0,
            "lock_timeout_seconds": self.no_signal_timeout_seconds,
            "origin_position_ready": False,
            "origin_heading_ready": False,
            "origin_spread_ready": False,
            "origin_quality_required_seconds": self.duration_seconds,
            "rtk_enu_x_m": None,
            "rtk_enu_y_m": None,
            "rtk_yaw_deg": None,
            "heading_review_status": "idle",
            "heading_review_started_at": None,
            "heading_review_elapsed_seconds": 0.0,
            "heading_review_remaining_seconds": self.duration_seconds,
            "heading_review_required_seconds": self.duration_seconds,
            "heading_review_reset_count": 0,
            "heading_quality_current": False,
            "heading_min_baseline_m": self.min_baseline_m,
            "heading_max_std_deg": self.max_heading_std_deg,
            "heading_max_age_seconds": self.max_age_seconds,
            "heading_offset_deg": self.heading_offset_deg,
            "position_fixed": False,
            "heading_fixed": False,
            "fix_status": -1,
            "solution_status": -1,
            "position_type": 0,
            "solution_satellites": 0,
            "position_spread_m": None,
            "max_spread_m": 0.02,
            "sample_count": 0,
            "reset_count": 0,
            "no_signal_seconds": 0.0,
            "error_code": "",
            "message": "尚未开始锁定 ENU 原点",
        }

    def start(self) -> dict:
        self.stop()
        try:
            self.origin_file.unlink(missing_ok=True)
        except OSError:
            pass
        with self._lock:
            self._samples.clear()
            self._position_fix_seen = False
            self._status = self._empty_status()
            self._status.update(
                origin_lock_session_id=str(uuid.uuid4()),
                origin_status="waiting_fix",
                max_spread_m=self.max_spread_m,
                message=(
                    f"最多 {self.no_signal_timeout_seconds:.0f} 秒等待位置 FIX；"
                    f"三项条件连续稳定 {self.duration_seconds:.0f} 秒后锁定原点"
                ),
            )
            self._fix_wait_started_monotonic = time.monotonic()
        self._stop.clear()
        self._generation += 1
        generation = self._generation
        self._thread = threading.Thread(target=self._run, args=(generation,), name="origin-lock-monitor", daemon=True)
        self._thread.start()
        return self.status()

    def prepare(self) -> dict:
        """Start live RTK preview without starting the quality timer."""
        self.stop()
        try:
            self.origin_file.unlink(missing_ok=True)
        except OSError:
            pass
        with self._lock:
            self._samples.clear()
            self._status = self._empty_status()
            self._status.update(
                origin_status="ready",
                max_spread_m=self.max_spread_m,
                message=(
                    f"传感器检查完成；锁定时检查位置 FIX、双天线航向基线 FIX和坐标波动，"
                    f"连续稳定 {self.duration_seconds:.0f} 秒"
                ),
            )
        self._stop.clear()
        self._generation += 1
        generation = self._generation
        self._thread = threading.Thread(
            target=self._run,
            args=(generation,),
            name="origin-preview-monitor",
            daemon=True,
        )
        self._thread.start()
        return self.status()

    def stop(self) -> None:
        self._stop.set()
        self._generation += 1
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None

    def resume(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._generation += 1
        generation = self._generation
        self._thread = threading.Thread(target=self._run, args=(generation,), name="origin-lock-monitor", daemon=True)
        self._thread.start()

    def restore(self, ttl_seconds: float) -> bool:
        if not self.origin_file.is_file():
            return False
        try:
            origin = yaml.safe_load(self.origin_file.read_text(encoding="utf-8")) or {}
            locked_at = float(origin.get("locked_at_unix") or 0.0)
        except (OSError, TypeError, ValueError, yaml.YAMLError):
            return False
        if not origin.get("alignment_locked") or locked_at <= 0 or time.time() - locked_at > ttl_seconds:
            return False
        with self._lock:
            self._status = self._empty_status()
            self._status.update(
                origin_lock_session_id=str(origin.get("origin_lock_session_id") or uuid.uuid4()),
                origin_status="locked",
                continuous_seconds=0.0,
                position_fixed=True,
                heading_fixed=bool(origin.get("heading_fixed", False)),
                heading_stable=False,
                position_spread_m=origin.get("position_spread_m"),
                sample_count=int(origin.get("sample_count") or 0),
                baseline_m=origin.get("baseline_m"),
                heading_deg=origin.get("heading_deg"),
                heading_std_deg=origin.get("heading_std_deg"),
                heading_confirmed=bool(origin.get("heading_confirmed", False)),
                heading_confirmation_source=origin.get("heading_confirmation_source"),
                heading_confirmed_at_unix=origin.get("heading_confirmed_at_unix"),
                confirmed_heading_deg=origin.get("confirmed_heading_deg"),
                confirmed_receiver_heading_deg=origin.get("confirmed_receiver_heading_deg"),
                confirmed_enu_yaw_deg=origin.get("confirmed_enu_yaw_deg"),
                confirmed_heading_std_deg=origin.get("confirmed_heading_std_deg"),
                confirmed_baseline_m=origin.get("confirmed_baseline_m"),
                origin_position_ready=True,
                origin_heading_ready=True,
                origin_spread_ready=True,
                latitude=origin.get("origin_latitude"),
                longitude=origin.get("origin_longitude"),
                altitude=origin.get("origin_altitude"),
                origin_file=str(self.origin_file),
                origin=origin,
                message="已恢复有效 ENU 原点；启动 SLAM 后将实时复核航向",
            )
        return True

    def begin_heading_review(self) -> dict:
        """Expose live heading checks and wait for explicit operator confirmation."""
        with self._lock:
            if self._status.get("origin_status") != "locked":
                return dict(self._status)
            if self._status.get("heading_review_status") in {"manual_confirmation", "ready", "confirmed"}:
                return dict(self._status)
            self._status.update(
                heading_review_status="manual_confirmation",
                heading_review_started_at=None,
                heading_review_elapsed_seconds=0.0,
                heading_review_remaining_seconds=self.duration_seconds,
                continuous_seconds=0.0,
                heading_stable=False,
                message="请原地小范围转动，确认双天线航向稳定后点击“确认航向稳定，开始建图”",
            )
            return dict(self._status)

    def confirm_heading_review(self) -> dict:
        with self._lock:
            if self._status.get("heading_review_status") not in {"manual_confirmation", "ready"}:
                raise RuntimeError("双天线航向尚未进入人工复核步骤")
            confirmed_at = time.time()
            confirmed_heading = self._status.get("heading_deg")
            confirmed_heading_std = self._status.get("heading_std_deg")
            confirmed_baseline = self._status.get("baseline_m")
            confirmation = {
                "heading_confirmed": True,
                "heading_confirmation_source": "operator",
                "heading_confirmed_at_unix": confirmed_at,
                "heading_offset_deg": self.heading_offset_deg,
            }
            try:
                if math.isfinite(float(confirmed_heading)):
                    receiver_heading = float(confirmed_heading)
                    confirmation["confirmed_receiver_heading_deg"] = receiver_heading
                    confirmation["confirmed_heading_deg"] = self._base_heading_deg(receiver_heading)
                    confirmation["confirmed_enu_yaw_deg"] = self._enu_yaw_deg(receiver_heading)
            except (TypeError, ValueError):
                pass
            try:
                if math.isfinite(float(confirmed_heading_std)):
                    confirmation["confirmed_heading_std_deg"] = float(confirmed_heading_std)
            except (TypeError, ValueError):
                pass
            try:
                if math.isfinite(float(confirmed_baseline)):
                    confirmation["confirmed_baseline_m"] = float(confirmed_baseline)
            except (TypeError, ValueError):
                pass

            origin = dict(self._status.get("origin") or {})
            if origin:
                origin.update(confirmation)
                self._write_origin_file_locked(origin)
                confirmation["origin"] = origin
            self._status.update(
                heading_review_status="confirmed",
                heading_stable=True,
                **confirmation,
                message=(
                    f"已记录人工确认原点航向 {confirmation['confirmed_heading_deg']:.2f}°；"
                    "SLAM 将用当前雷达航向锁定 ENU-地图航向后再融合 GNSS"
                    if "confirmed_heading_deg" in confirmation
                    else "已记录人工航向确认；SLAM 将锁定 ENU-地图航向后再融合 GNSS"
                ),
            )
            return dict(self._status)

    def cancel(self) -> dict:
        self.stop()
        try:
            self.origin_file.unlink(missing_ok=True)
        except OSError:
            pass
        with self._lock:
            self._samples.clear()
            self._status.update(origin_status="cancelled", message="原点锁定已取消", continuous_seconds=0.0)
        return self.status()

    def status(self) -> dict:
        with self._lock:
            return dict(self._status)

    def _run(self, generation: int) -> None:
        last_signal_at = time.monotonic()
        has_seen_signal = False
        while not self._stop.is_set() and generation == self._generation:
            try:
                sample = self.sample_provider()
                if self._stop.is_set() or generation != self._generation:
                    return
                last_signal_at = time.monotonic()
                has_seen_signal = True
                self.ingest(sample)
            except Exception as exc:  # telemetry failure must remain visible, not kill the worker
                if self._stop.is_set() or generation != self._generation:
                    return
                with self._lock:
                    now = time.monotonic()
                    elapsed = now - last_signal_at
                    if has_seen_signal:
                        elapsed = max(
                            elapsed,
                            float(getattr(exc, "unavailable_seconds", 0.0) or 0.0),
                        )
                    if elapsed >= self.no_signal_timeout_seconds:
                        self._fail_locked(
                            f"RTK 连续 {self.no_signal_timeout_seconds:.1f} 秒无信号，当前步骤已停止",
                            "RTK_SIGNAL_TIMEOUT",
                        )
                        return
                    self._status["no_signal_seconds"] = elapsed
                    self._status["message"] = f"RTK 话题读取失败：{exc}"
                # Do not add the normal one-second sampling delay while RTK is
                # absent. Retrying promptly keeps the wall-clock failure bound
                # aligned with no_signal_timeout_seconds instead of extending
                # it by one or more full sampling periods.
                remaining = self.no_signal_timeout_seconds - elapsed
                self._stop.wait(min(0.1, max(0.0, remaining)))
                continue
            self._stop.wait(self.sample_interval_seconds)

    def ingest(self, sample: OriginSample) -> dict:
        with self._lock:
            position_valid, position_reason = self._position_sample_valid(sample)
            heading_valid, heading_reason = self._heading_sample_valid(sample)
            heading_baseline_valid, heading_baseline_reason = self._heading_baseline_valid(sample)
            LOGGER.info(
                "RTK origin quality position_fix=%s heading_fix=%s baseline=%.3fm heading=%.3fdeg "
                "heading_std=%.3fdeg horizontal_std=%.3fm age=%.2fs quality=%s",
                sample.position_fixed,
                sample.heading_fixed,
                sample.baseline_m,
                sample.heading_deg,
                sample.heading_std_deg,
                sample.horizontal_std_m,
                sample.age_seconds,
                sample.ntrip_quality or "unknown",
            )
            self._status.update(
                position_fixed=sample.position_fixed,
                heading_fixed=sample.heading_fixed,
                latitude=sample.latitude,
                longitude=sample.longitude,
                altitude=sample.altitude,
                baseline_m=sample.baseline_m,
                heading_deg=sample.heading_deg,
                heading_std_deg=sample.heading_std_deg,
                horizontal_std_m=sample.horizontal_std_m,
                vertical_std_m=sample.vertical_std_m,
                age_seconds=sample.age_seconds,
                data_age_seconds=sample.age_seconds,
                ntrip_quality=sample.ntrip_quality,
                fix_status=sample.fix_status,
                solution_status=sample.solution_status,
                position_type=sample.position_type,
                solution_satellites=sample.solution_satellites,
                differential_age_seconds=sample.differential_age_seconds,
                message_time_offset_seconds=sample.message_time_offset_seconds,
                measurement_time_source=sample.measurement_time_source,
                measurement_stamp=sample.measurement_stamp,
                ntrip_state=sample.ntrip_state,
                tracking_satellites=sample.tracking_satellites,
                heading_status=sample.heading_status,
                heading_type=sample.heading_type,
                heading_satellites=sample.heading_satellites,
                heading_solution_satellites=sample.heading_solution_satellites,
                pitch_deg=sample.pitch_deg,
                pitch_std_deg=sample.pitch_std_deg,
                last_sample_at=sample.sampled_at,
                no_signal_seconds=0.0,
                error_code="",
                heading_quality_current=heading_valid,
            )
            self._update_live_enu_locked(sample)
            if self._status.get("origin_status") == "ready":
                self._update_preview_checks_locked(sample, position_valid, heading_baseline_valid)
                checks_ready = all(
                    self._status.get(key)
                    for key in ("origin_position_ready", "origin_heading_ready", "origin_spread_ready")
                )
                self._status.update(
                    heading_stable=False,
                    message=(
                        f"三项原点条件当前满足；点击锁定后连续检查 {self.duration_seconds:.0f} 秒"
                        if checks_ready
                        else "实时检查原点三项条件，未满足项请见状态卡片"
                    ),
                )
                return dict(self._status)
            if self._status.get("origin_status") == "locked":
                self._update_heading_review_locked(sample, heading_valid, heading_reason)
                return dict(self._status)
            if self._status.get("origin_status") not in {"waiting_fix", "quality_holding"}:
                return dict(self._status)
            elapsed = max(0.0, time.monotonic() - self._fix_wait_started_monotonic)
            self._status["lock_wait_seconds"] = min(elapsed, self.no_signal_timeout_seconds)
            if position_valid:
                self._position_fix_seen = True
                self._samples.append(sample)
            else:
                self._samples.clear()
            spread = self._spread_m(self._samples)
            if spread > self.max_spread_m:
                self._samples.clear()
                if position_valid:
                    self._samples.append(sample)
                spread = 0.0
                if self._status.get("quality_started_at") is not None:
                    self._status["reset_count"] = int(self._status.get("reset_count") or 0) + 1
                self._status["quality_started_at"] = None
                self._status["continuous_seconds"] = 0.0
            spread_valid = len(self._samples) >= 2 and spread <= self.max_spread_m
            self._status.update(
                origin_position_ready=position_valid,
                origin_heading_ready=heading_baseline_valid,
                origin_spread_ready=spread_valid,
                position_spread_m=spread if self._samples else None,
                sample_count=len(self._samples),
            )
            checks_ready = position_valid and heading_baseline_valid and spread_valid
            if checks_ready:
                if self._status.get("quality_started_at") is None:
                    self._status["quality_started_at"] = sample.sampled_at
                quality_elapsed = max(
                    0.0,
                    sample.sampled_at - float(self._status.get("quality_started_at") or sample.sampled_at),
                )
                self._status.update(
                    origin_status="quality_holding",
                    continuous_seconds=min(quality_elapsed, self.duration_seconds),
                    message=(
                        f"原点三项条件连续稳定 {quality_elapsed:.0f}/{self.duration_seconds:.0f} 秒，"
                        "请保持机器狗静止"
                    ),
                )
                if quality_elapsed >= self.duration_seconds:
                    self._lock_origin_locked()
            elif not self._position_fix_seen and elapsed >= self.no_signal_timeout_seconds:
                self._fail_locked(
                    f"{self.no_signal_timeout_seconds:.0f} 秒内未获得 RTK 位置固定解，原点锁定已停止",
                    "RTK_FIX_TIMEOUT",
                )
            else:
                if self._status.get("quality_started_at") is not None:
                    self._status["reset_count"] = int(self._status.get("reset_count") or 0) + 1
                self._status.update(
                    origin_status="waiting_fix",
                    quality_started_at=None,
                    continuous_seconds=0.0,
                )
                missing = []
                if not position_valid:
                    missing.append(position_reason)
                if not heading_baseline_valid:
                    missing.append(heading_baseline_reason)
                if not spread_valid:
                    missing.append("等待经纬度波动小于 2 cm（至少 2 个样本）")
                self._status["message"] = "；".join(missing) or "等待原点三项条件"
            return dict(self._status)

    def _position_sample_valid(self, sample: OriginSample) -> tuple[bool, str]:
        if not sample.position_fixed:
            return False, "等待 RTK 位置 FIX"
        if sample.age_seconds > self.max_age_seconds:
            return False, f"RTK 位置数据延迟 {sample.age_seconds:.1f}s，超过 {self.max_age_seconds:.1f}s"
        if not all(math.isfinite(value) for value in (sample.latitude, sample.longitude, sample.altitude)):
            return False, "RTK 坐标包含无效数值"
        return True, ""

    def _heading_sample_valid(self, sample: OriginSample) -> tuple[bool, str]:
        if not sample.heading_fixed:
            return False, "等待双天线航向基线 FIX"
        if sample.baseline_m < self.min_baseline_m:
            return False, f"双天线基线 {sample.baseline_m:.2f} m，小于 {self.min_baseline_m:.2f} m"
        if sample.heading_std_deg > self.max_heading_std_deg:
            return False, f"航向标准差 {sample.heading_std_deg:.2f}°，超过 {self.max_heading_std_deg:.2f}°"
        if sample.age_seconds > self.max_age_seconds:
            return False, f"RTK 航向数据延迟 {sample.age_seconds:.1f}s，超过 {self.max_age_seconds:.1f}s"
        return True, ""

    def _heading_baseline_valid(self, sample: OriginSample) -> tuple[bool, str]:
        if not sample.heading_fixed:
            return False, "等待双天线航向基线 FIX"
        if sample.baseline_m < self.min_baseline_m:
            return False, f"双天线基线 {sample.baseline_m:.2f} m，小于 {self.min_baseline_m:.2f} m"
        return True, ""

    def _update_preview_checks_locked(
        self,
        sample: OriginSample,
        position_valid: bool,
        heading_baseline_valid: bool,
    ) -> None:
        if position_valid:
            self._samples.append(sample)
            cutoff = sample.sampled_at - self.duration_seconds
            while self._samples and self._samples[0].sampled_at < cutoff:
                self._samples.popleft()
        else:
            self._samples.clear()
        spread = self._spread_m(self._samples)
        self._status.update(
            origin_position_ready=position_valid,
            origin_heading_ready=heading_baseline_valid,
            origin_spread_ready=len(self._samples) >= 2 and spread <= self.max_spread_m,
            position_spread_m=spread if self._samples else None,
            sample_count=len(self._samples),
        )

    def _update_live_enu_locked(self, sample: OriginSample) -> None:
        origin = self._status.get("origin") or {}
        try:
            lat0 = float(origin["origin_latitude"])
            lon0 = float(origin["origin_longitude"])
        except (KeyError, TypeError, ValueError):
            self._status.update(
                rtk_enu_x_m=None,
                rtk_enu_y_m=None,
                rtk_yaw_deg=self._enu_yaw_deg(sample.heading_deg) if sample.heading_fixed else None,
            )
            return
        earth = 6378137.0
        x_m = math.radians(sample.longitude - lon0) * math.cos(math.radians(lat0)) * earth
        y_m = math.radians(sample.latitude - lat0) * earth
        yaw_deg = self._enu_yaw_deg(sample.heading_deg) if sample.heading_fixed else None
        self._status.update(rtk_enu_x_m=x_m, rtk_enu_y_m=y_m, rtk_yaw_deg=yaw_deg)

    def _base_heading_deg(self, receiver_heading_deg: float) -> float:
        """Return robot compass heading (north-zero, clockwise-positive)."""
        return (float(receiver_heading_deg) - self.heading_offset_deg) % 360.0

    def _enu_yaw_deg(self, receiver_heading_deg: float) -> float:
        """Return robot yaw in ENU convention (east-zero, CCW-positive)."""
        return (90.0 - float(receiver_heading_deg) + self.heading_offset_deg) % 360.0

    def _update_heading_review_locked(
        self,
        sample: OriginSample,
        heading_valid: bool,
        reason: str,
    ) -> None:
        state = self._status.get("heading_review_status")
        if state == "manual_confirmation":
            self._status.update(
                heading_stable=heading_valid,
                message="请原地小范围转动，确认双天线航向稳定后点击“确认航向稳定，开始建图”",
            )
            return
        if state == "confirmed":
            self._status.update(heading_stable=True)
            return
        if state not in {"manual_confirmation", "confirmed"}:
            self._status.update(
                heading_stable=False,
                message="ENU 原点已锁定（只锁经纬高）；启动 SLAM 预热后进入人工航向复核",
            )

    def _fail_locked(self, message: str, error_code: str = "RTK_SIGNAL_TIMEOUT") -> None:
        self._samples.clear()
        self._status.update(
            origin_status="failed",
            quality_started_at=None,
            continuous_seconds=0.0,
            position_spread_m=None,
            sample_count=0,
            heading_stable=False,
            error_code=error_code,
            no_signal_seconds=self.no_signal_timeout_seconds,
            message=message,
        )
        self._stop.set()

    @staticmethod
    def _spread_m(samples: deque[OriginSample]) -> float:
        if len(samples) < 2:
            return 0.0
        lat0 = math.radians(samples[0].latitude)
        lon0 = math.radians(samples[0].longitude)
        earth = 6378137.0
        points = [
            (
                (math.radians(item.longitude) - lon0) * math.cos(lat0) * earth,
                (math.radians(item.latitude) - math.radians(samples[0].latitude)) * earth,
            )
            for item in samples
        ]
        return max(math.hypot(ax - bx, ay - by) for ax, ay in points for bx, by in points)

    def _lock_origin_locked(self) -> None:
        samples = list(self._samples)
        origin = {
            "schema": "roamerx.gnss-origin.v2",
            "datum": "CGCS2000",
            "origin_latitude": sum(item.latitude for item in samples) / len(samples),
            "origin_longitude": sum(item.longitude for item in samples) / len(samples),
            "origin_altitude": sum(item.altitude for item in samples) / len(samples),
            "enu_axis": "x=east,y=north,z=up",
            "alignment_locked": True,
            "alignment_source": "rtk_fixed_anchor",
            "lock_duration_seconds": float(self._status.get("lock_wait_seconds") or 0.0),
            "position_spread_m": self._spread_m(self._samples),
            "sample_count": len(samples),
            "heading_deg": samples[-1].heading_deg,
            "heading_std_deg": max(item.heading_std_deg for item in samples),
            "baseline_m": min(item.baseline_m for item in samples),
            "heading_fixed": bool(samples[-1].heading_fixed),
            "heading_offset_deg": self.heading_offset_deg,
            "locked_at_unix": time.time(),
            "origin_lock_session_id": self._status.get("origin_lock_session_id"),
        }
        self._write_origin_file_locked(origin)
        self._status.update(
            origin_status="locked",
            continuous_seconds=0.0,
            origin_file=str(self.origin_file),
            origin=origin,
            heading_stable=False,
            heading_review_status="idle",
            origin_position_ready=True,
            origin_heading_ready=True,
            origin_spread_ready=True,
            message=f"ENU 原点三项条件连续稳定 {self.duration_seconds:.0f} 秒并锁定成功；可以启动 SLAM",
        )

    def _write_origin_file_locked(self, origin: dict) -> None:
        """Atomically persist origin metadata while the monitor lock is held."""
        self.origin_file.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp_name = tempfile.mkstemp(
            prefix=f".{self.origin_file.name}.",
            suffix=".tmp",
            dir=str(self.origin_file.parent),
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                handle.write(yaml.safe_dump(origin, allow_unicode=True, sort_keys=False))
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(tmp_name, self.origin_file)
        except Exception:
            try:
                os.unlink(tmp_name)
            except OSError:
                pass
            raise
