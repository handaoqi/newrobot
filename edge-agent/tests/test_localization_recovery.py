from roamerx_edge.localization_recovery import select_recovery_seed, seed_is_plausible
from types import SimpleNamespace


def test_seed_is_plausible_rejects_large_drift():
    anchor = {"x": 41.0, "y": 8.0, "source": "latest_pose"}
    stale = {"x": 3.6, "y": 1.0, "source": "last_trusted"}
    assert seed_is_plausible(stale, anchor, max_drift_m=15.0) is False


def test_select_recovery_seed_prefers_latest_pose():
    seed = select_recovery_seed(
        latest_pose=SimpleNamespace(x=41.7, y=8.0, z=0.0, yaw=-0.4),
        memory_trusted={"x": 3.6, "y": 1.0, "yaw": 0.1},
        disk_trusted={"x": 3.6, "y": 1.0, "yaw": 0.1},
        waypoint=None,
        max_drift_m=15.0,
    )
    assert seed["source"] == "latest_pose"
    assert seed["x"] == 41.7
