from roamerx_edge.recovery_arbiter import RecoveryArbiter
from roamerx_edge.waypoint_actions import WaypointActionRegistry
from types import SimpleNamespace

from roamerx_edge.leg_profile import LegProfile
from roamerx_edge.local_store import LocalStore
from roamerx_edge.task_executor import TaskExecutor


class FakeNavigation:
    def __init__(self):
        self.pose = SimpleNamespace(x=0.0, y=0.0, yaw=0.0)
        self.localization_state = {}
        self.stopped = False
        self.safety_profiles = []

    def latest_pose(self):
        return self.pose

    def localization_decision(self):
        return dict(self.localization_state)

    def stop_motion(self):
        self.stopped = True

    def set_safety_profile(self, **kwargs):
        self.safety_profiles.append(kwargs)


def test_recovery_budget_exhaustion_blocks_new_owners():
    arbiter = RecoveryArbiter()
    leases = []
    for index in range(6):
        lease = arbiter.acquire("EDGE_OBSTACLE", f"attempt-{index}", distance_m=0.3)
        assert lease is not None
        leases.append(lease)
        assert arbiter.release(lease) is True
    assert arbiter.budget_exhausted() is True
    assert arbiter.acquire("EDGE_LOCALIZATION", "blocked") is None
    arbiter.reset_budget()
    assert arbiter.acquire("EDGE_LOCALIZATION", "after-reset") is not None


def test_indoor_arrival_requires_click_proximity(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.0, y=1.0, yaw=0.0)
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = type(
        "Ctx",
        (),
        {
            "route_snapshot": {
                "scene_scope": "indoor",
                "map": {"coordinate_mode": "ndt", "scene_scope": "indoor"},
                "waypoints": [{"x": 1.0, "y": 1.0, "arrival_policy": "stop_and_confirm"}],
            },
            "task_type": "patrol",
            "docking": {},
        },
    )()
    waypoint = {"x": 1.0, "y": 1.0, "arrival_policy": "stop_and_confirm"}
    assert executor._arrival_within_tolerance(waypoint, 0) is True
    nav.pose = SimpleNamespace(
        x=1.0 + executor.final_waypoint_tolerance_m + 0.1,
        y=1.0,
        yaw=0.0,
    )
    assert executor._arrival_within_tolerance(waypoint, 0) is False
    store.close()


def test_leg_profile_includes_speed_profile_and_safety_layers():
    profile = LegProfile(
        "ndt",
        "theta_star",
        "mppi",
        True,
        collision_slowdown_enabled=False,
        speed_profile="final",
        goal_checker_id="precision_goal_checker",
        arrival_policy="precision",
    )
    assert profile.speed_profile == "final"
    assert profile.collision_stop_enabled is True
    assert profile.collision_slowdown_enabled is False


def test_waypoint_action_registry_is_idempotent():
    registry = WaypointActionRegistry()
    first = registry.execute(actions=["snapshot"], idempotency_key="k1", context={"waypoint_id": "wp-1"})
    second = registry.execute(actions=["snapshot"], idempotency_key="k1", context={"waypoint_id": "wp-1"})
    assert first[0].status == "succeeded"
    assert second[0].status == "skipped_duplicate"
