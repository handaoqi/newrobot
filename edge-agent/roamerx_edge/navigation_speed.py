"""Shared route/teleoperation speed-tier definitions."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


NAVIGATION_SPEED_LEVELS = ("micro", "low", "medium", "high")


@dataclass(frozen=True)
class NavigationSpeedProfile:
    level: str
    vx_mps: float
    vy_mps: float
    wz_rps: float


# Current RemoteControlPage bases multiplied by its selected speed scale.
REMOTE_SPEED_PROFILES = {
    "micro": NavigationSpeedProfile("micro", 1.05, 0.7875, 1.8375),
    "low": NavigationSpeedProfile("low", 1.50, 1.125, 2.625),
    "medium": NavigationSpeedProfile("medium", 2.10, 1.575, 3.675),
    "high": NavigationSpeedProfile("high", 3.00, 2.25, 5.25),
}

# Automatic navigation intentionally keeps micro at the established 0.30 m/s
# cruise. Only explicit low/medium/high route selections use the corresponding
# remote-monitoring page values as their caps.
NAVIGATION_SPEED_PROFILES = {
    "micro": NavigationSpeedProfile("micro", 0.30, 0.225, 0.525),
    **{level: profile for level, profile in REMOTE_SPEED_PROFILES.items() if level != "micro"},
}


def normalize_navigation_speed_level(value: object) -> str:
    normalized = str(value or "micro").strip().lower()
    return normalized if normalized in NAVIGATION_SPEED_LEVELS else "micro"


def navigation_speed_profile(level: object, config: object | None = None) -> NavigationSpeedProfile:
    """Return a validated profile, optionally overridden by SafetyConfig."""
    normalized = normalize_navigation_speed_level(level)
    fallback = NAVIGATION_SPEED_PROFILES[normalized]
    if config is None:
        return fallback

    def value(axis: str, fallback_value: float) -> float:
        suffix = "rps" if axis == "turn" else "mps"
        raw = getattr(config, f"navigation_{axis}_{normalized}_{suffix}", fallback_value)
        try:
            parsed = float(raw)
        except (TypeError, ValueError):
            return fallback_value
        return parsed if isfinite(parsed) and parsed > 0.0 else fallback_value

    return NavigationSpeedProfile(
        normalized,
        value("speed", fallback.vx_mps),
        value("lateral", fallback.vy_mps),
        value("turn", fallback.wz_rps),
    )
