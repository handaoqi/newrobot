import math

from roamerx_edge.imu_cross_check import ImuCrossCheck, ImuCrossCheckConfig

DEG = math.pi / 180.0


def _fill(check, *, lidar_dps=None, body_dps=None, count=40, start=100.0, step=0.01):
    """Feed both windows with pure-yaw rates, which is what the drift is about."""
    for index in range(count):
        at = start + index * step
        if lidar_dps is not None:
            check.add_lidar_gyro(at, 0.0, 0.0, lidar_dps * DEG)
        if body_dps is not None:
            check.add_body_gyro(at, 0.0, 0.0, body_dps * DEG)
    return start + (count - 1) * step


def test_magnitude_is_frame_independent():
    # The whole design rests on this: the same rotation expressed in two frames
    # has the same norm, so no extrinsic calibration is needed to compare them.
    livox = ImuCrossCheck.magnitude_dps(0.1, -0.2, 0.3)
    body = ImuCrossCheck.magnitude_dps(-0.2, 0.3, 0.1)

    assert livox == body
    assert round(ImuCrossCheck.magnitude_dps(0.0, 0.0, DEG), 6) == 1.0


def test_absent_body_stream_is_not_a_mismatch():
    check = ImuCrossCheck()
    at = _fill(check, lidar_dps=50.0)

    report = check.evaluate(at)

    # Today's real state: the ecal2ros bridge does not publish the topic.
    assert report["status"] == "body_stream_absent"
    assert report["body_samples"] == 0
    assert report["lidar_samples"] == 40


def test_stale_body_stream_is_reported_as_absent():
    check = ImuCrossCheck(ImuCrossCheckConfig(body_stale_seconds=1.0))
    _fill(check, body_dps=0.0, count=40, start=100.0)
    at = _fill(check, lidar_dps=50.0, count=40, start=101.5)

    report = check.evaluate(at)

    assert report["status"] == "body_stream_absent"
    assert report["body_age_seconds"] > 1.0


def test_sparse_windows_do_not_produce_a_verdict():
    check = ImuCrossCheck(ImuCrossCheckConfig(min_samples=20))
    at = _fill(check, lidar_dps=50.0, body_dps=5.0, count=5)

    report = check.evaluate(at)

    assert report["status"] == "insufficient_samples"


def test_agreeing_sensors_report_ok():
    check = ImuCrossCheck()
    at = _fill(check, lidar_dps=30.0, body_dps=30.4)

    report = check.evaluate(at)

    assert report["status"] == "ok"
    assert report["stationary"] is False
    assert abs(report["delta_dps"]) < 1.0


def test_disagreement_needs_to_persist_before_it_is_reported():
    config = ImuCrossCheckConfig(mismatch_samples=3, evaluate_interval_seconds=0.0)
    check = ImuCrossCheck(config)
    at = _fill(check, lidar_dps=30.0, body_dps=10.0)

    statuses = [check.evaluate(at + offset)["status"] for offset in (0.0, 0.001, 0.002)]

    # A single bad window is noise; three in a row is a finding.
    assert statuses == ["ok", "ok", "mismatch"]


def test_one_good_window_clears_the_streak():
    config = ImuCrossCheckConfig(mismatch_samples=2, evaluate_interval_seconds=0.0)
    check = ImuCrossCheck(config)
    at = _fill(check, lidar_dps=30.0, body_dps=10.0)
    assert check.evaluate(at)["mismatch_streak"] == 1

    check = ImuCrossCheck(config)
    at = _fill(check, lidar_dps=30.0, body_dps=30.0)
    assert check.evaluate(at)["mismatch_streak"] == 0


def test_resting_biases_are_compared_against_the_tighter_static_limit():
    # The measured field values: 0.740 deg/s on the Livox, 0.285 on the BMI088.
    config = ImuCrossCheckConfig(
        stationary_rate_dps=2.0,
        max_static_bias_gap_dps=0.3,
        max_rate_delta_dps=3.0,
        mismatch_samples=1,
    )
    check = ImuCrossCheck(config)
    at = _fill(check, lidar_dps=0.740, body_dps=0.285)

    report = check.evaluate(at)

    assert report["stationary"] is True
    # Would have passed the 3.0 deg/s moving limit; the point of the separate
    # stationary limit is that this gap is exactly what drives the yaw drift.
    assert report["threshold_dps"] == 0.3
    assert report["status"] == "mismatch"
    assert report["lidar_static_bias_dps"] == 0.74
    assert report["body_static_bias_dps"] == 0.285


def test_evaluation_is_rate_limited():
    check = ImuCrossCheck(ImuCrossCheckConfig(evaluate_interval_seconds=5.0))

    assert check.due(100.0) is True
    check.evaluate(100.0)
    assert check.due(102.0) is False
    assert check.due(105.0) is True


def test_window_drops_samples_older_than_the_horizon():
    check = ImuCrossCheck(ImuCrossCheckConfig(window_seconds=1.0, min_samples=1))
    _fill(check, lidar_dps=90.0, body_dps=90.0, count=40, start=100.0, step=0.01)
    at = _fill(check, lidar_dps=0.0, body_dps=0.0, count=40, start=102.0, step=0.01)

    report = check.evaluate(at)

    # The fast samples are outside the window, so the verdict is "standing still".
    assert report["lidar_rate_dps"] == 0.0
    assert report["stationary"] is True
