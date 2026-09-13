from roamerx_edge.recovery_arbiter import RecoveryArbiter


def test_recovery_arbiter_allows_one_owner_and_requires_matching_release():
    arbiter = RecoveryArbiter()
    first = arbiter.acquire("EDGE_OBSTACLE", "reverse")
    assert first is not None
    assert arbiter.acquire("EDGE_LOCALIZATION", "relocalize") is None
    assert arbiter.release(first) is True
    assert arbiter.release(first) is False
    second = arbiter.acquire("EDGE_LOCALIZATION", "relocalize")
    assert second is not None
    assert second.generation > first.generation


def test_recovery_arbiter_generation_reclaim_cannot_release_a_newer_lease():
    arbiter = RecoveryArbiter()
    first = arbiter.acquire("BT_NAVIGATOR", "navigation_recovery_spin")
    assert first is not None
    assert arbiter.release_generation("BT_NAVIGATOR", first.generation) is True
    second = arbiter.acquire("BT_NAVIGATOR", "navigation_recovery_backup")
    assert second is not None

    assert arbiter.release_generation("BT_NAVIGATOR", first.generation) is False
    assert arbiter.snapshot()["recovery_generation"] == second.generation
    assert arbiter.release_generation("OTHER", second.generation) is False
    assert arbiter.release_generation("BT_NAVIGATOR", second.generation) is True
