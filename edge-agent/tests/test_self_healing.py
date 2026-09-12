from types import SimpleNamespace

import pytest

from roamerx_edge.app import EdgeAgentApplication
from roamerx_edge.local_store import LocalStore
from roamerx_edge.self_healing import (
    FAULT_LOCALIZATION_LOST,
    FAULT_RTK_TRANSIENT_LOSS,
    FaultDiagnoser,
    SelfHealingCoordinator,
    SelfHealingPolicyError,
    compact_self_healing_evidence,
)


OUTDOOR_ROUTE = {
    "scene_scope": "outdoor",
    "map": {"scene_scope": "outdoor", "coordinate_mode": "rtk_fixed"},
}


def evidence(*, status=3, rtk_quality="invalid", rtk_usable=False, lio=True, ndt=False):
    return {
        "localization_status": status,
        "localization_decision": {
            "rtk_quality": rtk_quality,
            "rtk_usable": rtk_usable,
            "rtk_good_for_navigation": rtk_quality == "fixed" and rtk_usable,
            "lio_healthy": lio,
            "ndt_healthy": ndt,
            "ndt_score": 0.05 if ndt else 0.8,
        },
        "obstacle": {},
    }


def test_outdoor_rtk_transient_loss_never_allows_spin_or_laser_motion():
    diagnosis = FaultDiagnoser().diagnose(
        episode_id="episode",
        requested_fault="navigation_failed",
        route_snapshot=OUTDOOR_ROUTE,
        evidence=evidence(),
    )

    assert diagnosis.fault_label == FAULT_RTK_TRANSIENT_LOSS
    assert diagnosis.wait_for_rtk is True
    assert diagnosis.forbid_spin is True

    level_two = FaultDiagnoser().diagnose(
        episode_id="episode",
        requested_fault=diagnosis.fault_label,
        route_snapshot=OUTDOOR_ROUTE,
        evidence=evidence(),
        level=2,
    )
    assert level_two.search_laser is False
    assert level_two.action_type == "continue_lio_hold"


def test_outdoor_hard_loss_uses_linear_feature_search_but_forbids_spin():
    diagnosis = FaultDiagnoser().diagnose(
        episode_id="episode",
        requested_fault=FAULT_LOCALIZATION_LOST,
        route_snapshot=OUTDOOR_ROUTE,
        evidence=evidence(status=4, lio=False),
        level=2,
    )

    assert diagnosis.fault_label == FAULT_LOCALIZATION_LOST
    assert diagnosis.forbid_spin is True
    assert diagnosis.search_laser is True


def test_indoor_double_absolute_source_failure_selects_lio_hold_then_adaptive_spin():
    diagnoser = FaultDiagnoser()
    level_one = diagnoser.diagnose(
        episode_id="episode",
        requested_fault="ndt_degraded",
        route_snapshot={"scene_scope": "indoor"},
        evidence=evidence(),
        level=1,
    )
    level_two = diagnoser.diagnose(
        episode_id="episode",
        requested_fault="ndt_degraded",
        route_snapshot={"scene_scope": "indoor"},
        evidence=evidence(),
        level=2,
    )

    assert level_one.use_lio_hold is True
    assert level_one.action_type == "set_ukf_lio_hold"
    assert level_two.forbid_spin is False
    assert level_two.action_type == "adaptive_spin"


def test_explicit_unhealthy_ndt_flag_is_not_overridden_by_a_borderline_score():
    sample = evidence(status=3, lio=True, ndt=False)
    sample["localization_decision"]["ndt_score"] = 0.45

    diagnosis = FaultDiagnoser(ndt_failure_score=0.5).diagnose(
        episode_id="episode",
        requested_fault="ndt_degraded",
        route_snapshot={"scene_scope": "indoor"},
        evidence=sample,
        level=1,
    )

    assert diagnosis.use_lio_hold is True
    assert diagnosis.recovered is False


def test_initializing_localization_status_is_diagnosed_without_integer_conversion():
    diagnosis = FaultDiagnoser().diagnose(
        episode_id="episode",
        requested_fault="navigation_failed",
        route_snapshot={"scene_scope": "indoor"},
        evidence=evidence(status="initializing", lio=False),
    )

    assert diagnosis.fault_label == FAULT_LOCALIZATION_LOST
    assert diagnosis.recovered is False


def test_navigation_failure_does_not_change_ukf_profile_only_because_anchors_are_weak():
    diagnosis = FaultDiagnoser().diagnose(
        episode_id="episode",
        requested_fault="navigation_failed",
        route_snapshot={"scene_scope": "indoor"},
        evidence=evidence(),
        level=1,
    )

    assert diagnosis.fault_label == "nav_action_failed"
    assert diagnosis.use_lio_hold is False
    assert diagnosis.action_type == "local_replan"


def test_ukf_eligible_float_rtk_prevents_unnecessary_lio_hold():
    sample = evidence(rtk_quality="float", rtk_usable=True)
    sample["localization_decision"]["rtk_drift"] = {"xy_m": 0.19}

    diagnosis = FaultDiagnoser().diagnose(
        episode_id="episode",
        requested_fault="ndt_degraded",
        route_snapshot={"scene_scope": "indoor"},
        evidence=sample,
        level=1,
    )

    assert diagnosis.use_lio_hold is False
    assert diagnosis.action_type == "local_relocalize"


def test_persisted_evidence_summarizes_global_plan_instead_of_storing_points():
    sample = evidence()
    sample["obstacle"] = {
        "collision_limited": False,
        "global_plan": {
            "updated": True,
            "points": [{"x": float(index), "y": 0.0} for index in range(1000)],
        },
    }

    compact = compact_self_healing_evidence(sample)

    assert "global_plan" not in compact["obstacle"]
    assert compact["obstacle"]["global_plan_updated"] is True
    assert compact["obstacle"]["global_plan_point_count"] == 1000


def test_smart_rtk_wait_exits_after_three_consecutive_fixed_samples(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    coordinator = SelfHealingCoordinator(store=store, rtk_required_samples=3)
    first = coordinator.diagnose(
        "navigation_failed", route_snapshot=OUTDOOR_ROUTE, evidence=evidence()
    )
    assert first.fault_label == FAULT_RTK_TRANSIENT_LOSS

    results = [
        coordinator.diagnose(
            first.fault_label,
            episode_id=first.episode_id,
            route_snapshot=OUTDOOR_ROUTE,
            evidence=evidence(rtk_quality="fixed", rtk_usable=True),
        )
        for _ in range(3)
    ]

    assert [result.recovered for result in results] == [False, False, True]
    assert coordinator.snapshot()["finished"] is True
    store.close()


def test_episode_and_action_statistics_are_durable(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    coordinator = SelfHealingCoordinator(store=store)
    diagnosis = coordinator.diagnose(
        "ndt_degraded",
        route_snapshot={"scene_scope": "indoor"},
        evidence=evidence(status=4, lio=False),
    )
    action_id = coordinator.begin_action(level=1, action_type="local_relocalize")
    coordinator.finish_action(action_id, success=True, reason="accepted")
    coordinator.complete(success=True, reason="healthy")

    stats = store.self_heal_statistics()
    assert stats["faults"][0]["fault_label"] == diagnosis.fault_label
    assert stats["faults"][0]["succeeded"] == 1
    assert stats["actions"][0]["action_type"] == "local_relocalize"
    assert stats["actions"][0]["succeeded"] == 1
    store.close()


def test_bt_action_reports_are_persisted_and_progress_completes_episode(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    published = []
    application = object.__new__(EdgeAgentApplication)
    application.store = store
    application.config = SimpleNamespace(
        robot=SimpleNamespace(current_map_id="map-1"),
        safety=SimpleNamespace(ndt_failure_score=0.5),
    )
    application.safety_state = SimpleNamespace(localization_status="normal")
    application.navigation = SimpleNamespace(
        localization_diagnostics=lambda: {"quality": 0.1},
        localization_decision=lambda: evidence()["localization_decision"],
        obstacle_monitor_snapshot=lambda: {},
    )
    application.task_executor = SimpleNamespace(
        context=SimpleNamespace(
            route_snapshot={"scene_scope": "indoor"},
            task_execution_id="task-1",
            trace_id="trace-1",
        )
    )
    application.mqtt = SimpleNamespace(
        publish_task_event=lambda *args: published.append(args)
    )
    application._latest_task_state_event = {}
    application.self_healing = SelfHealingCoordinator(
        store=store,
        event_callback=application._publish_task_event,
    )

    diagnosed = application._handle_navigation_self_healing({
        "operation": 1,
        "fault_label": "navigation_failed",
        "level": 0,
    })
    application._handle_navigation_self_healing({
        "operation": 1,
        "episode_id": diagnosed["episode_id"],
        "fault_label": diagnosed["fault_label"],
        "level": 1,
    })
    application._handle_navigation_self_healing({
        "operation": 1,
        "episode_id": diagnosed["episode_id"],
        "fault_label": diagnosed["fault_label"],
        "level": 2,
    })
    concrete = application._handle_navigation_self_healing({
        "operation": 3,
        "episode_id": diagnosed["episode_id"],
        "fault_label": diagnosed["fault_label"],
        "level": 2,
        "action_type": "bounded_reverse",
    })
    application._handle_navigation_self_healing({
        "operation": 4,
        "episode_id": diagnosed["episode_id"],
        "action_id": concrete["action_id"],
        "fault_label": diagnosed["fault_label"],
        "level": 2,
        "action_type": "bounded_reverse",
        "success": True,
        "detail": "ndt_recovered",
    })
    application._publish_task_event("task.progress", {"state": "running"})

    stats = store.self_heal_statistics()
    assert stats["faults"][0]["succeeded"] == 1
    action_stats = {item["action_type"]: item for item in stats["actions"]}
    assert action_stats["bounded_reverse"]["succeeded"] == 1
    assert application.self_healing.snapshot()["finished"] is True
    assert any(event[0] == "self_healing.completed" for event in published)
    store.close()


def test_episode_rejects_level_jump_and_wrong_episode(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    coordinator = SelfHealingCoordinator(store=store)

    with pytest.raises(SelfHealingPolicyError, match="must start at level 0"):
        coordinator.diagnose(
            "navigation_failed",
            route_snapshot={"scene_scope": "indoor"},
            evidence=evidence(),
            level=2,
        )

    first = coordinator.diagnose(
        "navigation_failed",
        route_snapshot={"scene_scope": "indoor"},
        evidence=evidence(),
        level=0,
    )
    with pytest.raises(SelfHealingPolicyError, match="level jump rejected"):
        coordinator.diagnose(
            first.fault_label,
            episode_id=first.episode_id,
            route_snapshot={"scene_scope": "indoor"},
            evidence=evidence(),
            level=2,
        )
    with pytest.raises(SelfHealingPolicyError, match="does not match"):
        coordinator.diagnose(
            first.fault_label,
            episode_id="wrong-episode",
            route_snapshot={"scene_scope": "indoor"},
            evidence=evidence(),
            level=1,
        )
    store.close()


def test_concrete_motion_requires_current_edge_policy_authorization(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    coordinator = SelfHealingCoordinator(store=store)
    first = coordinator.diagnose(
        "navigation_failed",
        route_snapshot={"scene_scope": "indoor"},
        evidence=evidence(),
        level=0,
    )
    coordinator.diagnose(
        first.fault_label,
        episode_id=first.episode_id,
        route_snapshot={"scene_scope": "indoor"},
        evidence=evidence(),
        level=1,
    )
    coordinator.diagnose(
        first.fault_label,
        episode_id=first.episode_id,
        route_snapshot={"scene_scope": "indoor"},
        evidence=evidence(),
        level=2,
    )

    spin_allowed, _ = coordinator.authorize_action(
        episode_id=first.episode_id, level=2, action_type="adaptive_spin"
    )
    reverse_allowed, _ = coordinator.authorize_action(
        episode_id=first.episode_id, level=2, action_type="bounded_reverse"
    )
    wrong_episode_allowed, _ = coordinator.authorize_action(
        episode_id="wrong", level=2, action_type="bounded_reverse"
    )

    assert spin_allowed is False
    assert reverse_allowed is True
    assert wrong_episode_allowed is False
    store.close()


def test_action_started_requires_existing_episode_id(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    application = object.__new__(EdgeAgentApplication)
    application.self_healing = SelfHealingCoordinator(store=store)

    result = application._handle_navigation_self_healing({
        "operation": 3,
        "level": 0,
        "fault_label": "navigation_failed",
        "action_type": "observe_and_clear",
    })

    assert result == {
        "accepted": False,
        "reason": "active self-healing episode_id is required",
    }
    store.close()


def test_reconcile_and_retention_keep_long_term_summary(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    coordinator = SelfHealingCoordinator(store=store)
    diagnosis = coordinator.diagnose(
        "ndt_degraded",
        route_snapshot={"scene_scope": "indoor"},
        evidence=evidence(status=4, lio=False),
    )
    coordinator.begin_action(level=0, action_type=diagnosis.action_type)

    reconciled = store.reconcile_unfinished_self_healing()
    assert reconciled == {"episodes": 1, "actions": 1}
    action = store._connection.execute(
        "SELECT success, reason, duration_seconds FROM self_heal_actions"
    ).fetchone()
    assert dict(action)["success"] == 0
    assert dict(action)["reason"] == "interrupted_by_restart"
    assert dict(action)["duration_seconds"] is not None

    with store._connection:
        store._connection.execute(
            "UPDATE self_heal_episodes SET started_at='2020-01-01 00:00:00'"
        )
        store._connection.execute(
            "UPDATE self_heal_actions SET started_at='2020-01-01 00:00:00'"
        )
    pruned = store.prune_self_healing(retention_days=30)
    assert pruned["episodes"] == 1
    assert pruned["actions"] == 1
    assert pruned["foreign_key_violations"] == 0
    assert store._connection.execute(
        "SELECT COUNT(*) FROM self_heal_episodes"
    ).fetchone()[0] == 0
    assert store._connection.execute(
        "SELECT COUNT(*) FROM self_heal_actions"
    ).fetchone()[0] == 0
    stats = store.self_heal_statistics()
    assert stats["faults"][0]["total"] == 1
    assert stats["actions"][0]["total"] == 1
    assert stats["actions"][0]["average_duration_seconds"] is not None
    store.close()
