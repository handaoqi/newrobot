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

