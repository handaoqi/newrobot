from __future__ import annotations

import math
import logging
import os
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import yaml

LOGGER = logging.getLogger(__name__)


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
        duration_seconds: float = 60.0,
        max_spread_m: float = 0.02,
        sample_interval_seconds: float = 1.0,
        min_baseline_m: float = 0.20,
        max_heading_std_deg: float = 5.0,
        max_age_seconds: float = 1.5,
        no_signal_timeout_seconds: float = 3.0,
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
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._generation = 0
        self._samples: deque[OriginSample] = deque()
        self._status: dict = self._empty_status()

    @staticmethod
    def _empty_status() -> dict:
        return {
            "origin_lock_session_id": None,
            "origin_status": "idle",
            "quality_started_at": None,
            "continuous_seconds": 0.0,
            "required_seconds": 60.0,
            "position_fixed": False,
            "heading_fixed": False,
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
            self._status = self._empty_status()
            self._status.update(
                origin_lock_session_id=str(uuid.uuid4()),
                origin_status="waiting_quality",
                required_seconds=self.duration_seconds,
                max_spread_m=self.max_spread_m,
                message="等待位置 FIX、双天线航向 FIX 和厘米级稳定窗口",
            )
        self._stop.clear()
        self._generation += 1
        generation = self._generation
        self._thread = threading.Thread(target=self._run, args=(generation,), name="origin-lock-monitor", daemon=True)
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
                continuous_seconds=float(origin.get("lock_duration_seconds") or self.duration_seconds),
                required_seconds=self.duration_seconds,
                position_fixed=True,
                heading_fixed=True,
                heading_stable=False,
                position_spread_m=origin.get("position_spread_m"),
                sample_count=int(origin.get("sample_count") or 0),
                baseline_m=origin.get("baseline_m"),
                heading_deg=origin.get("heading_deg"),
                heading_std_deg=origin.get("heading_std_deg"),
                latitude=origin.get("origin_latitude"),
                longitude=origin.get("origin_longitude"),
                altitude=origin.get("origin_altitude"),
                origin_file=str(self.origin_file),
                origin=origin,
                message="已恢复有效 ENU 原点；启动 SLAM 后将实时复核航向",
            )
        return True

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
        no_signal_since: float | None = None
        while not self._stop.is_set() and generation == self._generation:
            try:
                sample = self.sample_provider()
                if self._stop.is_set() or generation != self._generation:
                    return
                no_signal_since = None
                self.ingest(sample)
            except Exception as exc:  # telemetry failure must remain visible, not kill the worker
                if self._stop.is_set() or generation != self._generation:
                    return
                with self._lock:
                    now = time.monotonic()
                    if no_signal_since is None:
                        no_signal_since = now
                    elapsed = now - no_signal_since
                    if elapsed >= self.no_signal_timeout_seconds:
                        self._fail_locked(
                            f"RTK 连续 {self.no_signal_timeout_seconds:.1f} 秒无信号，原点锁定已停止"
                        )
                        return
                    self._status["no_signal_seconds"] = elapsed
                    self._reset_window_locked(f"RTK 话题读取失败：{exc}")
            self._stop.wait(self.sample_interval_seconds)

    def ingest(self, sample: OriginSample) -> dict:
        with self._lock:
            valid, reason = self._sample_valid(sample)
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
                age_seconds=sample.age_seconds,
                ntrip_quality=sample.ntrip_quality,
                last_sample_at=sample.sampled_at,
                no_signal_seconds=0.0,
                error_code="",
            )
            if self._status.get("origin_status") == "locked":
                self._status["heading_stable"] = valid
                self._status["message"] = "ENU 原点已锁定；请小范围转动并复核双天线航向"
                return dict(self._status)
            if not valid:
                self._reset_window_locked(reason)
                return dict(self._status)

            if not self._samples:
                self._status["quality_started_at"] = sample.sampled_at
            self._samples.append(sample)
            spread = self._spread_m(self._samples)
            if spread > self.max_spread_m:
                self._reset_window_locked(f"经纬度波动 {spread * 100:.1f} cm，超过 2 cm，连续计时已重置")
                return dict(self._status)
            elapsed = max(0.0, sample.sampled_at - float(self._status["quality_started_at"] or sample.sampled_at))
            self._status.update(
                origin_status="quality_holding",
                continuous_seconds=min(elapsed, self.duration_seconds),
                position_spread_m=spread,
                sample_count=len(self._samples),
                heading_stable=True,
                message=f"质量连续满足 {elapsed:.0f}/{self.duration_seconds:.0f} 秒，请保持静止",
            )
            if elapsed >= self.duration_seconds:
                self._lock_origin_locked()
            return dict(self._status)

    def _sample_valid(self, sample: OriginSample) -> tuple[bool, str]:
        if not sample.position_fixed:
            return False, "等待 RTK 位置 FIX，连续计时未开始"
        if not sample.heading_fixed:
            return False, "等待双天线航向基线 FIX，连续计时未开始"
        if sample.baseline_m < self.min_baseline_m:
            return False, f"双天线基线 {sample.baseline_m:.2f} m，小于 {self.min_baseline_m:.2f} m"
        if sample.heading_std_deg > self.max_heading_std_deg:
            return False, f"航向标准差 {sample.heading_std_deg:.2f}°，超过 {self.max_heading_std_deg:.2f}°"
        if sample.age_seconds > self.max_age_seconds:
            return False, f"RTK 航向数据延迟 {sample.age_seconds:.1f}s，超过 {self.max_age_seconds:.1f}s"
        if not all(math.isfinite(value) for value in (sample.latitude, sample.longitude, sample.altitude)):
            return False, "RTK 坐标包含无效数值"
        return True, ""

    def _reset_window_locked(self, message: str) -> None:
        if self._samples:
            self._status["reset_count"] = int(self._status.get("reset_count") or 0) + 1
        self._samples.clear()
        self._status.update(
            origin_status="waiting_quality",
            quality_started_at=None,
            continuous_seconds=0.0,
            position_spread_m=None,
            sample_count=0,
            heading_stable=False,
            message=message,
        )

    def _fail_locked(self, message: str) -> None:
        self._samples.clear()
        self._status.update(
            origin_status="failed",
            quality_started_at=None,
            continuous_seconds=0.0,
            position_spread_m=None,
            sample_count=0,
            heading_stable=False,
            error_code="RTK_SIGNAL_TIMEOUT",
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
            "alignment_source": "dual_antenna_anchor",
            "lock_duration_seconds": self.duration_seconds,
            "position_spread_m": self._spread_m(self._samples),
            "sample_count": len(samples),
            "heading_deg": samples[-1].heading_deg,
            "heading_std_deg": max(item.heading_std_deg for item in samples),
            "baseline_m": min(item.baseline_m for item in samples),
            "locked_at_unix": time.time(),
            "origin_lock_session_id": self._status.get("origin_lock_session_id"),
        }
        self.origin_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.origin_file.with_suffix(self.origin_file.suffix + ".tmp")
        temporary.write_text(yaml.safe_dump(origin, allow_unicode=True, sort_keys=False), encoding="utf-8")
        os.replace(temporary, self.origin_file)
        self._status.update(
            origin_status="locked",
            continuous_seconds=self.duration_seconds,
            origin_file=str(self.origin_file),
            origin=origin,
            heading_stable=True,
            message="ENU 原点锁定成功，可以启动 SLAM 预热",
        )
