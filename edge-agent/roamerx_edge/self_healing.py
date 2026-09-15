"""Scene-aware diagnosis and level tracking for navigation self healing.

This module deliberately contains no ROS calls.  Edge is the policy owner;
ROS/BT adapters only transport a diagnosis or execute an allowed action.
"""

from __future__ import annotations

import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Callable


FAULT_LOCALIZATION_LOST = "localization_lost"
FAULT_NDT_DEGRADED = "ndt_degraded"
FAULT_RTK_TRANSIENT_LOSS = "rtk_transient_loss"
FAULT_NAV_STUCK = "nav_stuck"
FAULT_NAV_ACTION_FAILED = "nav_action_failed"
FAULT_SENSOR_STALE = "sensor_stale"
FAULT_COLLISION_STOP = "collision_stop"

LOCALIZATION_FAULTS = {
    FAULT_LOCALIZATION_LOST,
    FAULT_NDT_DEGRADED,
    FAULT_RTK_TRANSIENT_LOSS,
    FAULT_SENSOR_STALE,
}


class SelfHealingPolicyError(ValueError):
    """A stale, skipped-level, or unauthorized recovery request."""


@dataclass(frozen=True)
class FaultDiagnosis:
    episode_id: str
    fault_label: str
    scene_mode: str
    level: int
    action_type: str
    forbid_spin: bool
    wait_for_rtk: bool
    use_lio_hold: bool
    search_laser: bool
    recovered: bool
    reason: str


@dataclass
class SelfHealingEpisode:
    episode_id: str
    fault_label: str
    scene_mode: str
    started_wall_time: float
    started_monotonic: float
    evidence: dict = field(default_factory=dict)
    current_level: int = 0
    action_type: str = ""
    active_action_id: str = ""
    rtk_stable_samples: int = 0
    allowed_actions: tuple[str, ...] = ()
    finished: bool = False


def normalize_scene_mode(route_snapshot: dict | None) -> str:
    route = route_snapshot or {}
    map_info = route.get("map") or {}
    scene = str(map_info.get("scene_scope") or route.get("scene_scope") or "indoor").lower()
    return scene if scene in {"indoor", "outdoor", "transition"} else "indoor"


def _bool(value) -> bool:
    return value is True


def _finite_score(value) -> float | None:
    try:
        score = float(value)
    except (TypeError, ValueError):
        return None
    return score if score == score and abs(score) != float("inf") else None


def _localization_status_is_normal(value) -> bool:
    if isinstance(value, bool):
        return False
    if isinstance(value, (int, float)):
        return int(value) == 3
    return str(value or "").strip().lower() in {"3", "normal", "localized"}


def _rtk_float_within_ukf_gate(decision: dict) -> bool:
    if decision.get("rtk_float_usable_for_ukf") is True:
        return True
    drift = decision.get("rtk_drift")
    drift_xy = _finite_score(drift.get("xy_m")) if isinstance(drift, dict) else None
    threshold = (
        drift.get("ukf_float_max_residual_m")
        if isinstance(drift, dict)
        else decision.get("ukf_float_max_residual_m")
    )
    try:
        threshold = float(threshold) if threshold is not None else 0.40
    except (TypeError, ValueError):
        threshold = 0.40
    return (
        decision.get("rtk_usable") is True
        and str(decision.get("rtk_quality") or "").lower() == "float"
        and drift_xy is not None
        and drift_xy <= threshold
    )


def compact_self_healing_evidence(evidence: dict | None) -> dict:
    """Keep decision evidence useful without persisting plans or sample arrays."""

    def scalar_fields(values) -> dict:
        if not isinstance(values, dict):
            return {}
        compact = {}
        for key, value in values.items():
            if value is None or isinstance(value, (bool, int, float)):
                compact[str(key)] = value
            elif isinstance(value, str):
                compact[str(key)] = value[:256]
        return compact

    source = evidence if isinstance(evidence, dict) else {}
    compact = scalar_fields(source)
    decision = source.get("localization_decision")
    compact["localization_decision"] = scalar_fields(decision)
    if isinstance(decision, dict):
        for nested_key in ("rtk_drift", "ndt_drift", "one_shot_correction"):
            nested = scalar_fields(decision.get(nested_key))
            if nested:
                compact["localization_decision"][nested_key] = nested
    compact["obstacle"] = scalar_fields(source.get("obstacle"))
    global_plan = (source.get("obstacle") or {}).get("global_plan")
    if isinstance(global_plan, dict):
        points = global_plan.get("points")
        compact["obstacle"]["global_plan_updated"] = global_plan.get("updated") is True
        compact["obstacle"]["global_plan_point_count"] = (
            len(points) if isinstance(points, list) else 0
        )
    return compact


class FaultDiagnoser:
    """Pure decision matrix shared by automatic Edge and BT recovery paths."""

    def __init__(self, *, ndt_failure_score: float = 0.5) -> None:
        self.ndt_failure_score = float(ndt_failure_score)

    def normalize_fault(self, requested: str, evidence: dict, scene_mode: str) -> str:
        requested = str(requested or FAULT_NAV_ACTION_FAILED).strip().lower()
        decision = evidence.get("localization_decision") or {}
        obstacle = evidence.get("obstacle") or {}
        status = evidence.get("localization_status")
        rtk_fixed = _bool(decision.get("rtk_good_for_navigation")) or (
            _bool(decision.get("rtk_usable"))
            and str(decision.get("rtk_quality") or "").lower() == "fixed"
        )
        lio_healthy = _bool(decision.get("lio_healthy"))
        ndt_score = _finite_score(decision.get("ndt_score"))
        ndt_health = decision.get("ndt_healthy")
        ndt_healthy = (
            _bool(ndt_health)
            if isinstance(ndt_health, bool)
            else ndt_score is not None and ndt_score < self.ndt_failure_score
        )

        if obstacle.get("collision_limited") is True:
            return FAULT_COLLISION_STOP
        if obstacle.get("stale") is True or evidence.get("sensor_stale") is True:
            return FAULT_SENSOR_STALE
        if (
            status is not None
            and not _localization_status_is_normal(status)
            and not (rtk_fixed or lio_healthy)
        ):
            return FAULT_LOCALIZATION_LOST
        if requested.startswith("ndt") and not ndt_healthy:
            return FAULT_NDT_DEGRADED
        if (
            scene_mode in {"outdoor", "transition"}
            and not rtk_fixed
            and lio_healthy
            and requested in {
                FAULT_LOCALIZATION_LOST,
                FAULT_NDT_DEGRADED,
                FAULT_NAV_ACTION_FAILED,
                "navigation_failed",
            }
        ):
            return FAULT_RTK_TRANSIENT_LOSS
        aliases = {
            "navigation_failed": FAULT_NAV_ACTION_FAILED,
            "navigation_failure": FAULT_NAV_ACTION_FAILED,
            "obstacle_stuck": FAULT_NAV_STUCK,
        }
        normalized = aliases.get(requested, requested)
        supported = LOCALIZATION_FAULTS | {
            FAULT_NAV_STUCK,
            FAULT_NAV_ACTION_FAILED,
            FAULT_COLLISION_STOP,
        }
        return normalized if normalized in supported else FAULT_NAV_ACTION_FAILED

    def diagnose(
        self,
        *,
        episode_id: str,
        requested_fault: str,
        route_snapshot: dict | None,
        evidence: dict,
        level: int = 0,
        rtk_stable_samples: int = 0,
        required_rtk_samples: int = 3,
    ) -> FaultDiagnosis:
        scene_mode = normalize_scene_mode(route_snapshot)
        fault = self.normalize_fault(requested_fault, evidence, scene_mode)
        decision = evidence.get("localization_decision") or {}
        ndt_score = _finite_score(decision.get("ndt_score"))
        ndt_health = decision.get("ndt_healthy")
        ndt_healthy = (
            _bool(ndt_health)
            if isinstance(ndt_health, bool)
            else ndt_score is not None and ndt_score < self.ndt_failure_score
        )
        lio_healthy = _bool(decision.get("lio_healthy"))
        rtk_fixed = _bool(decision.get("rtk_good_for_navigation")) or (
            _bool(decision.get("rtk_usable"))
            and str(decision.get("rtk_quality") or "").lower() == "fixed"
        )
        rtk_float_within_gate = _rtk_float_within_ukf_gate(decision)
        localization_normal = _localization_status_is_normal(
            evidence.get("localization_status")
        )
        nav_progress = evidence.get("navigation_progress") is True

        recovered = False
        if fault == FAULT_RTK_TRANSIENT_LOSS:
            recovered = rtk_fixed and rtk_stable_samples >= max(1, required_rtk_samples)
        elif fault == FAULT_NDT_DEGRADED:
            recovered = localization_normal and (rtk_fixed or ndt_healthy)
        elif fault in LOCALIZATION_FAULTS:
            recovered = localization_normal and (rtk_fixed or ndt_healthy or lio_healthy)
        elif fault in {FAULT_NAV_STUCK, FAULT_NAV_ACTION_FAILED}:
            recovered = nav_progress and not _bool((evidence.get("obstacle") or {}).get("collision_limited"))
        elif fault == FAULT_COLLISION_STOP:
            recovered = not _bool((evidence.get("obstacle") or {}).get("collision_limited"))

        level = min(3, max(0, int(level)))
        outdoor_rtk_policy = scene_mode in {"outdoor", "transition"}
        forbid_spin = outdoor_rtk_policy or fault == FAULT_RTK_TRANSIENT_LOSS
        absolute_sources_poor = not rtk_fixed and not rtk_float_within_gate and not ndt_healthy
        # A navigation/controller failure must not silently alter localization
        # fusion merely because both absolute observers happen to be weak.
        # LIO hold is reserved for diagnosed localization-source failures.
        use_lio_hold = fault in LOCALIZATION_FAULTS and absolute_sources_poor and lio_healthy

        if recovered:
            action = "exit_recovery"
            reason = "health_condition_recovered"
        elif level == 0:
            action = "smart_rtk_wait" if fault == FAULT_RTK_TRANSIENT_LOSS else "observe_and_clear"
            reason = (
                "outdoor_rtk_wait_without_motion"
                if fault == FAULT_RTK_TRANSIENT_LOSS else "level_0_observation"
            )
        elif level == 1:
            action = "set_ukf_lio_hold" if use_lio_hold else (
                "local_relocalize" if fault in LOCALIZATION_FAULTS else "local_replan"
            )
            reason = "absolute_sources_poor_lio_healthy" if use_lio_hold else "level_1_low_cost"
        elif level == 2:
            if fault in LOCALIZATION_FAULTS:
                action = (
                    "continue_lio_hold"
                    if fault == FAULT_RTK_TRANSIENT_LOSS
                    else ("search_laser_feature" if forbid_spin else "adaptive_spin")
                )
            else:
                action = "bounded_reverse_or_bypass"
            reason = "level_2_bounded_motion"
        else:
            action = "global_relocalize" if fault in LOCALIZATION_FAULTS else "safe_hold"
            reason = "level_3_final_recovery"

        return FaultDiagnosis(
            episode_id=episode_id,
            fault_label=fault,
            scene_mode=scene_mode,
            level=level,
            action_type=action,
            forbid_spin=forbid_spin,
            wait_for_rtk=(
                fault == FAULT_RTK_TRANSIENT_LOSS and level == 0 and not recovered
            ),
            use_lio_hold=use_lio_hold and level >= 1 and not recovered,
            search_laser=(
                fault in LOCALIZATION_FAULTS
                and fault != FAULT_RTK_TRANSIENT_LOSS
                and level == 2
                and not recovered
            ),
            recovered=recovered,
            reason=reason,
        )


class SelfHealingCoordinator:
    """Tracks one bounded self-healing episode and persists its lifecycle."""

    def __init__(
        self,
        *,
        store,
        event_callback: Callable[[str, dict, str], None] | None = None,
        ndt_failure_score: float = 0.5,
        rtk_required_samples: int = 3,
    ) -> None:
        self.store = store
        self.event_callback = event_callback
        self.diagnoser = FaultDiagnoser(ndt_failure_score=ndt_failure_score)
        self.rtk_required_samples = max(1, int(rtk_required_samples))
        self._lock = threading.RLock()
        self._episode: SelfHealingEpisode | None = None
        self._last_diagnosis: FaultDiagnosis | None = None
        self._action_started_monotonic: dict[str, float] = {}

    @staticmethod
    def _allowed_actions(diagnosis: FaultDiagnosis) -> tuple[str, ...]:
        if diagnosis.recovered:
            return ()
        if diagnosis.level <= 1:
            return (diagnosis.action_type,)
        if diagnosis.level == 2 and diagnosis.fault_label in LOCALIZATION_FAULTS:
            if diagnosis.fault_label == FAULT_RTK_TRANSIENT_LOSS:
                return ()
            actions = ["search_laser_feature"]
            if not diagnosis.forbid_spin:
                actions.append("adaptive_spin")
            return tuple(actions)
        if diagnosis.level == 2:
            return (
                "bounded_reverse",
                "bounded_bypass",
                "bounded_lateral_left",
                "bounded_lateral_right",
            )
        return (diagnosis.action_type,)

    def diagnose(
        self,
        requested_fault: str,
        *,
        route_snapshot: dict | None,
        evidence: dict,
        episode_id: str = "",
        level: int | None = None,
    ) -> FaultDiagnosis:
        with self._lock:
            requested_level = 0 if level is None else int(level)
            if requested_level < 0 or requested_level > 3:
                raise SelfHealingPolicyError("recovery level must be between 0 and 3")
            episode = self._episode
            if episode is not None and episode.finished and episode_id:
                if episode.episode_id == episode_id and self._last_diagnosis is not None:
                    return self._last_diagnosis
                raise SelfHealingPolicyError("stale or unknown self-healing episode")
            if episode is not None and not episode.finished and episode_id and episode.episode_id != episode_id:
                raise SelfHealingPolicyError("self-healing episode does not match the active episode")
            if episode is None or episode.finished:
                if requested_level != 0:
                    raise SelfHealingPolicyError("a new self-healing episode must start at level 0")
                scene_mode = normalize_scene_mode(route_snapshot)
                normalized_fault = self.diagnoser.normalize_fault(
                    requested_fault, evidence, scene_mode
                )
                compact_evidence = compact_self_healing_evidence(evidence)
                episode = SelfHealingEpisode(
                    episode_id=str(uuid.uuid4()),
                    fault_label=normalized_fault,
                    scene_mode=scene_mode,
                    started_wall_time=time.time(),
                    started_monotonic=time.monotonic(),
                    evidence=compact_evidence,
                )
                self._episode = episode
                self.store.start_self_heal_episode(
                    episode.episode_id,
                    fault_label=episode.fault_label,
                    scene_mode=episode.scene_mode,
                    task_execution_id=str(evidence.get("task_execution_id") or ""),
                    map_id=str(evidence.get("map_id") or ""),
                    evidence=compact_evidence,
                )
                self._emit("self_healing.started", episode, {"evidence": compact_evidence})
            elif requested_level > episode.current_level + 1:
                raise SelfHealingPolicyError(
                    f"recovery level jump rejected: current={episode.current_level} requested={requested_level}"
                )
            if level is not None:
                episode.current_level = max(episode.current_level, requested_level)
            episode.evidence = compact_self_healing_evidence(evidence)
            decision = evidence.get("localization_decision") or {}
            rtk_fixed = _bool(decision.get("rtk_good_for_navigation")) or (
                _bool(decision.get("rtk_usable"))
                and str(decision.get("rtk_quality") or "").lower() == "fixed"
            )
            episode.rtk_stable_samples = episode.rtk_stable_samples + 1 if rtk_fixed else 0
            diagnosis = self.diagnoser.diagnose(
                episode_id=episode.episode_id,
                requested_fault=episode.fault_label,
                route_snapshot=route_snapshot,
                evidence=evidence,
                level=episode.current_level,
                rtk_stable_samples=episode.rtk_stable_samples,
                required_rtk_samples=self.rtk_required_samples,
            )
            episode.fault_label = diagnosis.fault_label
            episode.scene_mode = diagnosis.scene_mode
            episode.action_type = diagnosis.action_type
            episode.allowed_actions = self._allowed_actions(diagnosis)
            self._last_diagnosis = diagnosis
            if diagnosis.recovered:
                self.complete(success=True, reason=diagnosis.reason)
            return diagnosis

    def authorize_action(
        self,
        *,
        episode_id: str,
        level: int,
        action_type: str,
    ) -> tuple[bool, str]:
        """Authorize a concrete BT motion against the latest Edge decision."""
        with self._lock:
            episode = self._episode
            if episode is None or episode.finished:
                return False, "self-healing episode is not active"
            if not episode_id or episode.episode_id != episode_id:
                return False, "self-healing episode does not match the active episode"
            if int(level) != episode.current_level:
                return False, (
                    f"action level does not match current level: "
                    f"current={episode.current_level} requested={int(level)}"
                )
            normalized = str(action_type or "").strip()
            if normalized not in episode.allowed_actions:
                return False, (
                    f"action {normalized or '<empty>'} is not allowed; "
                    f"allowed={','.join(episode.allowed_actions) or '<none>'}"
                )
            return True, "action authorized by current diagnosis"

    def begin_action(self, *, level: int, action_type: str) -> str:
        with self._lock:
            if self._episode is None or self._episode.finished:
                return ""
            self._episode.current_level = min(3, max(self._episode.current_level, int(level)))
            self._episode.action_type = str(action_type)
            action_id = str(uuid.uuid4())
            self._episode.active_action_id = action_id
            self._action_started_monotonic[action_id] = time.monotonic()
            self.store.start_self_heal_action(
                action_id,
                episode_id=self._episode.episode_id,
                level=self._episode.current_level,
                action_type=self._episode.action_type,
            )
            self._emit(
                "self_healing.action_started",
                self._episode,
                {"action_id": action_id},
            )
            return action_id

    def finish_action(self, action_id: str = "", *, success: bool, reason: str = "") -> None:
        with self._lock:
            if not action_id and self._episode is not None:
                action_id = self._episode.active_action_id
            if not action_id:
                return
            started = self._action_started_monotonic.pop(action_id, None)
            duration_seconds = (
                max(0.0, time.monotonic() - started) if started is not None else None
            )
            self.store.finish_self_heal_action(
                action_id,
                success=success,
                reason=reason,
                duration_seconds=duration_seconds,
            )
            if self._episode is not None:
                self._emit(
                    "self_healing.action_finished",
                    self._episode,
                    {"action_id": action_id, "success": bool(success), "reason": reason},
                )
                if self._episode.active_action_id == action_id:
                    self._episode.active_action_id = ""

    def advance(self) -> int:
        with self._lock:
            if self._episode is None or self._episode.finished:
                return 0
            self._episode.current_level = min(3, self._episode.current_level + 1)
            return self._episode.current_level

    def complete(self, *, success: bool, reason: str) -> None:
        with self._lock:
            episode = self._episode
            if episode is None or episode.finished:
                return
            if episode.active_action_id:
                self.finish_action(
                    episode.active_action_id,
                    success=success,
                    reason=f"episode_completed:{reason}",
                )
            episode.finished = True
            elapsed = max(0.0, time.monotonic() - episode.started_monotonic)
            self.store.finish_self_heal_episode(
                episode.episode_id,
                success=success,
                terminal_level=episode.current_level,
                terminal_action=episode.action_type,
                duration_seconds=elapsed,
                reason=reason,
            )
            self._emit(
                "self_healing.completed",
                episode,
                {"success": bool(success), "duration_seconds": round(elapsed, 3), "reason": reason},
            )

    def snapshot(self) -> dict:
        with self._lock:
            episode = self._episode
            if episode is None:
                return {}
            return {
                "episode_id": episode.episode_id,
                "fault_label": episode.fault_label,
                "scene_mode": episode.scene_mode,
                "level": episode.current_level,
                "action_type": episode.action_type,
                "action_id": episode.active_action_id,
                "allowed_actions": list(episode.allowed_actions),
                "finished": episode.finished,
            }

    def _emit(self, event_type: str, episode: SelfHealingEpisode, extra: dict) -> None:
        if not self.event_callback:
            return
        payload = {
            "episode_id": episode.episode_id,
            "fault_label": episode.fault_label,
            "scene_mode": episode.scene_mode,
            "level": episode.current_level,
            "action_type": episode.action_type,
            **extra,
        }
        try:
            self.event_callback(event_type, payload, "")
        except Exception:
            # Persistence is the source of truth; telemetry failure must not
            # interrupt a safety recovery path.
            pass
