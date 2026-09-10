"""Immutable per-leg navigation configuration."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class LegProfile:
    localization_mode: str
    global_planner_id: str
    local_controller_id: str
    detour_enabled: bool
    collision_slowdown_enabled: bool = True
    collision_stop_enabled: bool = True
    speed_profile: str = "cruise"
    goal_checker_id: str = "general_goal_checker"
    arrival_policy: str = "stop_and_confirm"

