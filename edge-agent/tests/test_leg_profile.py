from roamerx_edge.leg_profile import LegProfile


def test_leg_profile_is_immutable_and_keeps_collision_stop_enabled():
    profile = LegProfile("ndt", "ThetaStar", "FollowPath", True)
    assert profile.collision_stop_enabled is True
    try:
        profile.detour_enabled = False
    except AttributeError:
        pass
    else:
        raise AssertionError("LegProfile must be immutable")

