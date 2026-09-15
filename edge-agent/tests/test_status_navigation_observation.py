from roamerx_edge.app import navigation_status_observation


def test_navigation_status_marks_uninitialized_velocity_cache_unobserved():
    result = navigation_status_observation(
        {
            "actual_planar_speed_mps": 0.0,
            "actual_turn_speed_rps": 0.0,
            "actual_velocity_sample_age_seconds": None,
        }
    )

    assert result["observation_schema"] == "roamerx.navigation-observation.v1"
    assert result["actual_velocity_observed"] is False
    assert result["requested_velocity_observed"] is False
    assert result["cmd_vel_idle_uncommanded"] is True


def test_navigation_status_marks_timestamped_velocity_cache_observed():
    result = navigation_status_observation(
        {
            "actual_planar_speed_mps": 0.0,
            "actual_turn_speed_rps": 0.0,
            "actual_velocity_sample_age_seconds": 0.0,
            "requested_velocity_sample_age_seconds": 0.25,
        }
    )

    assert result["actual_velocity_observed"] is True
    assert result["requested_velocity_observed"] is True
    assert result["cmd_vel_idle_uncommanded"] is False
