import math

import pytest

from roamerx_patrol_demo.scenario import PatrolConfig, PatrolScenario


def test_scenario_is_deterministic_and_returns_home():
    config = PatrolConfig(duration_sec=120.0, seed=20260828)
    first = PatrolScenario(config)
    second = PatrolScenario(config)

    for t_ns in (0, 1_000_000_000, 45_000_000_000, 119_999_000_000):
        a = first.sample(t_ns)
        b = second.sample(t_ns)
        assert a == b

    end = first.sample(120_000_000_000)
    assert math.isclose(end.x, 1.0, abs_tol=0.10)
    assert math.isclose(end.y, 1.0, abs_tol=0.10)
    assert end.phase == "COMPLETED"


def test_pause_and_obstacle_window_are_stable():
    scenario = PatrolScenario(PatrolConfig(duration_sec=120.0))
    pause = scenario.sample(55_000_000_000)
    assert pause.phase == "PAUSE_P2"
    assert pause.speed_mps == 0.0

    obstacle = scenario.sample(80_000_000_000)
    assert obstacle.phase == "OBSTACLE_AVOIDANCE"
    assert obstacle.diagnostic_level == 1


@pytest.mark.parametrize(
    "kwargs",
    [
        {"duration_sec": 0},
        {"publish_rate_hz": 0},
        {"linear_speed_mps": -1},
        {"topic_namespace": "../escape"},
        {"waypoints": ((1.0, 1.0),)},
        {"waypoints": ((7.0, 1.0), (7.0, 6.0), (2.0, 6.0), (10.1, 3.0))},
    ],
)
def test_invalid_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        PatrolConfig(**kwargs)
