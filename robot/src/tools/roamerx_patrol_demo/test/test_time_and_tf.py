from roamerx_patrol_demo.scenario import PatrolConfig, PatrolScenario


def test_simulated_time_is_monotonic_and_frames_are_fixed():
    scenario = PatrolScenario(PatrolConfig())
    samples = [scenario.sample(i * 100_000_000) for i in range(20)]
    assert [s.time_ns for s in samples] == sorted(s.time_ns for s in samples)
    assert samples[0].frame_id == "base_link"
    assert samples[0].odom_frame == "odom"
