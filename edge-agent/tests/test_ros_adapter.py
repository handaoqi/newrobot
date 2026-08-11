import math

from roamerx_edge.ros_adapter import RosAdapter


def test_fresh_normal_streak_ignores_samples_before_candidate():
    samples = [(10, 3), (11, 3), (12, 3)]

    assert RosAdapter._fresh_normal_streak(samples, after_sequence=12) == 0


def test_fresh_normal_streak_requires_consecutive_successes():
    samples = [(20, 4), (21, 3), (22, 3), (23, 3)]

    assert RosAdapter._fresh_normal_streak(samples, after_sequence=20) == 3

    samples.append((24, 4))
    assert RosAdapter._fresh_normal_streak(samples, after_sequence=20) == 0


def test_relocalization_candidates_cover_full_yaw_and_nearby_positions():
    candidates = RosAdapter._relocalization_candidates(4.0, 5.0, 0.0, 0.0)

    center_yaws = {
        round(candidate["yaw"], 6)
        for candidate in candidates
        if candidate["x"] == 4.0 and candidate["y"] == 5.0
    }
    assert len(center_yaws) == 8
    assert round(math.pi, 6) in {round(abs(yaw), 6) for yaw in center_yaws}
    assert {(candidate["x"], candidate["y"]) for candidate in candidates[8:]} == {
        (5.0, 5.0),
        (3.0, 5.0),
        (4.0, 6.0),
        (4.0, 4.0),
    }
