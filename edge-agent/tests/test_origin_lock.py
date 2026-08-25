import yaml
import time

import pytest

from roamerx_edge.origin_lock import OriginLockMonitor, OriginSample


def sample(stamp, *, latitude=39.9, position_fixed=True, heading_fixed=True, heading_std=0.5):
    return OriginSample(
        latitude=latitude,
        longitude=116.4,
        altitude=42.0,
        position_fixed=position_fixed,
        heading_fixed=heading_fixed,
        baseline_m=0.8,
        heading_deg=91.2,
        heading_std_deg=heading_std,
        horizontal_std_m=0.008,
        age_seconds=0.1,
        ntrip_quality="rtk_fixed",
        fix_status=2,
        solution_status=0,
        position_type=48,
        solution_satellites=18,
        vertical_std_m=0.012,
        differential_age_seconds=0.2,
        message_time_offset_seconds=0.03,
        measurement_time_source="device_utc",
        sampled_at=stamp,
    )


def test_locks_origin_after_all_three_checks_hold_continuously(tmp_path):
    output = tmp_path / "gnss_origin.yaml"
    monitor = OriginLockMonitor(lambda: sample(0), str(output), duration_seconds=2, max_spread_m=0.02)
    monitor._status.update(origin_status="waiting_fix")
    monitor._fix_wait_started_monotonic = time.monotonic()

    monitor.ingest(sample(100.0))
    monitor.ingest(sample(101.0))
    monitor.ingest(sample(102.0))
    status = monitor.ingest(sample(103.0))

    assert status["origin_status"] == "locked"
    assert status["heading_stable"] is False
    assert status["heading_review_status"] == "idle"
    assert status["position_type"] == 48
    assert status["solution_satellites"] == 18
    assert status["message_time_offset_seconds"] == 0.03
    saved = yaml.safe_load(output.read_text())
    assert saved["alignment_locked"] is True
    assert saved["alignment_source"] == "rtk_fixed_anchor"
    assert saved["sample_count"] == 4


def test_heading_review_waits_for_manual_confirmation_without_countdown(tmp_path):
    origin_file = tmp_path / "origin.yaml"
    monitor = OriginLockMonitor(
        lambda: sample(0),
        str(origin_file),
        duration_seconds=2,
        heading_offset_deg=180.0,
    )
    origin = {"alignment_locked": True, "origin_latitude": 39.9, "origin_longitude": 116.4}
    origin_file.write_text(yaml.safe_dump(origin))
    monitor._status.update(origin_status="locked", origin=origin)
    monitor.begin_heading_review()

    status = monitor.ingest(sample(100.0))

    assert status["origin_status"] == "locked"
    assert status["heading_review_status"] == "manual_confirmation"
    assert status["heading_review_elapsed_seconds"] == 0
    assert status["heading_stable"] is True
    assert "原地小范围转动" in status["message"]

    confirmed = monitor.confirm_heading_review()
    saved = yaml.safe_load(origin_file.read_text())

    assert confirmed["confirmed_receiver_heading_deg"] == pytest.approx(91.2)
    assert confirmed["confirmed_heading_deg"] == pytest.approx(271.2)
    assert confirmed["confirmed_enu_yaw_deg"] == pytest.approx(178.8)
    assert confirmed["confirmed_heading_std_deg"] == pytest.approx(0.5)
    assert confirmed["heading_confirmation_source"] == "operator"
    assert saved["heading_confirmed"] is True
    assert saved["confirmed_heading_deg"] == pytest.approx(271.2)
    assert saved["heading_offset_deg"] == pytest.approx(180.0)
    assert saved["heading_confirmed_at_unix"] > 0


def test_invalid_heading_remains_visible_but_does_not_disable_manual_review(tmp_path):
    monitor = OriginLockMonitor(lambda: sample(0), str(tmp_path / "origin.yaml"), duration_seconds=2)
    monitor._status.update(origin_status="locked")
    monitor.begin_heading_review()

    status = monitor.ingest(sample(101.0, heading_fixed=False))

    assert status["heading_review_status"] == "manual_confirmation"
    assert status["heading_review_elapsed_seconds"] == 0
    assert status["heading_review_reset_count"] == 0
    assert status["heading_stable"] is False

    confirmed = monitor.confirm_heading_review()
    assert confirmed["heading_review_status"] == "confirmed"


def test_position_fix_timeout_stops_origin_lock(tmp_path):
    monitor = OriginLockMonitor(
        lambda: sample(time.time(), position_fixed=False),
        str(tmp_path / "origin.yaml"),
        sample_interval_seconds=0.01,
        no_signal_timeout_seconds=0.1,
    )
    monitor.start()
    try:
        deadline = time.monotonic() + 1.0
        status = monitor.status()
        while status["origin_status"] != "failed" and time.monotonic() < deadline:
            time.sleep(0.01)
            status = monitor.status()
    finally:
        monitor.stop()

    assert status["error_code"] == "RTK_FIX_TIMEOUT"
    assert "固定解" in status["message"]


def test_missing_rtk_signal_fails_after_timeout(tmp_path):
    def no_signal():
        raise RuntimeError("/fix 超时无数据")

    monitor = OriginLockMonitor(
        no_signal,
        str(tmp_path / "gnss_origin.yaml"),
        sample_interval_seconds=0.5,
        no_signal_timeout_seconds=0.1,
    )
    started = time.monotonic()
    monitor.start()
    try:
        deadline = time.monotonic() + 1.0
        status = monitor.status()
        while status["origin_status"] != "failed" and time.monotonic() < deadline:
            time.sleep(0.01)
            status = monitor.status()
    finally:
        monitor.stop()

    assert status["origin_status"] == "failed"
    assert status["error_code"] == "RTK_SIGNAL_TIMEOUT"
    assert "无信号" in status["message"]
    assert time.monotonic() - started < 0.35


def test_prepare_streams_live_preview_without_starting_quality_window(tmp_path):
    monitor = OriginLockMonitor(
        lambda: sample(time.time(), position_fixed=False, heading_fixed=False),
        str(tmp_path / "gnss_origin.yaml"),
        sample_interval_seconds=0.01,
    )
    monitor.prepare()
    try:
        deadline = time.monotonic() + 1.0
        status = monitor.status()
        while "latitude" not in status and time.monotonic() < deadline:
            time.sleep(0.01)
            status = monitor.status()
    finally:
        monitor.stop()

    assert status["origin_status"] == "ready"
    assert status["latitude"] == pytest.approx(39.9)
    assert status["sample_count"] == 0
    assert status["continuous_seconds"] == 0.0
    assert "实时检查" in status["message"]
    assert status["origin_position_ready"] is False
    assert status["origin_heading_ready"] is False
    assert status["origin_spread_ready"] is False


def test_heading_review_still_requires_operator_confirmation(tmp_path):
    monitor = OriginLockMonitor(lambda: sample(0), str(tmp_path / "origin.yaml"), duration_seconds=1)
    monitor._status.update(origin_status="locked")
    monitor.begin_heading_review()
    monitor.ingest(sample(100.0))
    monitor.ingest(sample(101.0))

    status = monitor.confirm_heading_review()

    assert status["heading_review_status"] == "confirmed"
    assert status["heading_stable"] is True


def test_locked_origin_publishes_live_rtk_enu_xy_and_corrected_base_yaw(tmp_path):
    monitor = OriginLockMonitor(
        lambda: sample(0),
        str(tmp_path / "origin.yaml"),
        heading_offset_deg=180.0,
    )
    monitor._status.update(
        origin_status="locked",
        origin={"origin_latitude": 39.9, "origin_longitude": 116.4},
    )

    status = monitor.ingest(sample(100.0, latitude=39.900001))

    assert status["rtk_enu_x_m"] == pytest.approx(0.0, abs=1e-6)
    assert status["rtk_enu_y_m"] == pytest.approx(0.1113, rel=0.01)
    assert status["rtk_yaw_deg"] == pytest.approx(178.8)
