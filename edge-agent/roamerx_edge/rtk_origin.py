"""RTK origin payload parsing and thread-safe live ROS message cache."""

from __future__ import annotations

import copy
import math
import threading
import time
from typing import Callable

import yaml

from .origin_lock import OriginSample, OriginSignalUnavailable


def _finite_number(*values: object, default: float) -> float:
    """Return the first present finite number, preserving legitimate zeroes."""
    for value in values:
        if value is None or value == "":
            continue
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(number):
            return number
    return float(default)


def build_origin_sample(
    fix: dict,
    pvh: dict,
    ntrip_message: dict,
    *,
    sampled_at: float | None = None,
    now: Callable[[], float] = time.time,
) -> OriginSample:
    """Normalize the three RTK topic payloads into the origin-lock contract."""
    ntrip_data = ntrip_message.get("data") or ""
    ntrip = yaml.safe_load(ntrip_data) if isinstance(ntrip_data, str) else ntrip_data
    if not isinstance(ntrip, dict):
        ntrip = {}
    bestnav = pvh.get("bestnav") or {}
    heading = pvh.get("heading") or {}
    fix_status_value = (fix.get("status") or {}).get("status")
    fix_status = int(fix_status_value) if fix_status_value is not None else -1
    solution_status = int(bestnav.get("p_sol_status", ntrip.get("solution_status", -1)))
    position_type = int(bestnav.get("pos_type") or ntrip.get("position_type") or 0)
    ntrip_quality = str(ntrip.get("quality") or "")
    position_fixed = (
        fix_status >= 2
        and solution_status == 0
        and position_type in {48, 49, 50}
        and ntrip_quality == "rtk_fixed"
    )
    heading_fixed = int(heading.get("sol_status", -1)) == 0 and int(heading.get("heading_type") or 0) > 0
    lat_std = _finite_number(
        bestnav.get("lat_std"), ntrip.get("horizontal_std_m"), default=9999.0
    )
    lon_std = _finite_number(
        bestnav.get("lon_std"), ntrip.get("horizontal_std_m"), default=9999.0
    )
    vertical_std = _finite_number(
        bestnav.get("hgt_std"), ntrip.get("vertical_std_m"), default=9999.0
    )
    differential_age = _finite_number(
        bestnav.get("diff_age_s"),
        ntrip.get("differential_age_s"),
        ntrip.get("differential_age_seconds"),
        default=0.0,
    )
    header_stamp = pvh.get("header", {}).get("stamp", {})
    stamp_seconds = _finite_number(header_stamp.get("sec"), default=0.0) + _finite_number(
        header_stamp.get("nanosec"), default=0.0
    ) / 1e9
    message_time_offset = now() - stamp_seconds if stamp_seconds > 1_000_000_000 else None
    header_age = max(0.0, message_time_offset) if message_time_offset is not None else 0.0
    data_age = max(_finite_number(ntrip.get("age_sec"), default=0.0), header_age)
    return OriginSample(
        latitude=_finite_number(bestnav.get("latitude_deg"), fix.get("latitude"), default=0.0),
        longitude=_finite_number(bestnav.get("longitude_deg"), fix.get("longitude"), default=0.0),
        altitude=_finite_number(bestnav.get("altitude_m"), fix.get("altitude"), default=0.0),
        position_fixed=position_fixed,
        heading_fixed=heading_fixed,
        baseline_m=_finite_number(heading.get("base_line"), default=0.0),
        heading_deg=_finite_number(heading.get("heading_deg"), default=0.0),
        heading_std_deg=_finite_number(heading.get("heading_std"), default=180.0),
        horizontal_std_m=max(lat_std, lon_std),
        vertical_std_m=vertical_std,
        age_seconds=data_age,
        ntrip_quality=ntrip_quality,
        fix_status=fix_status,
        solution_status=solution_status,
        position_type=position_type,
        solution_satellites=int(bestnav.get("soln_svs_num") or ntrip.get("solution_satellites") or 0),
        differential_age_seconds=differential_age,
        message_time_offset_seconds=message_time_offset,
        measurement_time_source=str(ntrip.get("measurement_time_source") or ""),
        measurement_stamp=stamp_seconds if stamp_seconds > 0 else None,
        ntrip_state=str(ntrip.get("state") or ""),
        tracking_satellites=int(bestnav.get("svs_num") or ntrip.get("satellites") or 0),
        heading_status=int(heading.get("sol_status", -1)),
        heading_type=int(heading.get("heading_type") or 0),
        heading_satellites=int(heading.get("svs_num") or 0),
        heading_solution_satellites=int(heading.get("soln_svs_num") or 0),
        pitch_deg=_finite_number(heading.get("pitch_deg"), default=0.0),
        pitch_std_deg=_finite_number(heading.get("pitch_std"), default=90.0),
        sampled_at=_finite_number(sampled_at, default=now()),
    )


class RtkOriginPayloadCache:
    """Keep the latest messages without spawning a ROS CLI process per sample."""

    REQUIRED_STREAMS = ("fix", "pvh", "ntrip")

    def __init__(self, *, stale_after_seconds: float = 1.5) -> None:
        self.stale_after_seconds = max(0.1, float(stale_after_seconds))
        self._lock = threading.Lock()
        self._created_monotonic = time.monotonic()
        self._payloads: dict[str, dict] = {}
        self._received_monotonic: dict[str, float] = {}
        self._received_wall: dict[str, float] = {}

    def update(self, stream: str, payload: dict) -> None:
        if stream not in self.REQUIRED_STREAMS:
            raise ValueError(f"unsupported RTK stream: {stream}")
        now_monotonic = time.monotonic()
        now_wall = time.time()
        with self._lock:
            self._payloads[stream] = copy.deepcopy(payload)
            self._received_monotonic[stream] = now_monotonic
            self._received_wall[stream] = now_wall

    def snapshot(self) -> tuple[dict, dict, dict, float]:
        now_monotonic = time.monotonic()
        with self._lock:
            missing = [name for name in self.REQUIRED_STREAMS if name not in self._payloads]
            if missing:
                elapsed = now_monotonic - self._created_monotonic
                raise OriginSignalUnavailable(
                    f"RTK 话题尚无数据：{', '.join(missing)}",
                    elapsed,
                )
            ages = {
                name: max(0.0, now_monotonic - self._received_monotonic[name])
                for name in self.REQUIRED_STREAMS
            }
            stale = [name for name, age in ages.items() if age > self.stale_after_seconds]
            if stale:
                elapsed = max(ages[name] for name in stale)
                detail = ", ".join(f"{name} {ages[name]:.1f}s" for name in stale)
                raise OriginSignalUnavailable(f"RTK 话题数据过期：{detail}", elapsed)
            return (
                copy.deepcopy(self._payloads["fix"]),
                copy.deepcopy(self._payloads["pvh"]),
                copy.deepcopy(self._payloads["ntrip"]),
                self._received_wall["pvh"],
            )
