import yaml
import time

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
        sampled_at=stamp,
    )


def test_locks_after_continuous_quality_window(tmp_path):
    output = tmp_path / "gnss_origin.yaml"
    monitor = OriginLockMonitor(lambda: sample(0), str(output), duration_seconds=2, max_spread_m=0.02)

    monitor.ingest(sample(100.0))
    monitor.ingest(sample(101.0, latitude=39.90000001))
    status = monitor.ingest(sample(102.0, latitude=39.90000002))

    assert status["origin_status"] == "locked"
    assert status["position_spread_m"] < 0.02
    saved = yaml.safe_load(output.read_text())
    assert saved["alignment_locked"] is True
    assert saved["alignment_source"] == "dual_antenna_anchor"
    assert saved["sample_count"] == 3


def test_invalid_sample_resets_continuous_timer(tmp_path):
    monitor = OriginLockMonitor(lambda: sample(0), str(tmp_path / "origin.yaml"), duration_seconds=2)
    monitor.ingest(sample(100.0))
    status = monitor.ingest(sample(101.0, heading_fixed=False))

    assert status["origin_status"] == "waiting_quality"
    assert status["continuous_seconds"] == 0
    assert status["sample_count"] == 0
    assert "航向" in status["message"]


def test_more_than_two_centimetres_resets_window(tmp_path):
    monitor = OriginLockMonitor(lambda: sample(0), str(tmp_path / "origin.yaml"), duration_seconds=2)
    monitor.ingest(sample(100.0))
    status = monitor.ingest(sample(101.0, latitude=39.900001))

    assert status["origin_status"] == "waiting_quality"
    assert status["reset_count"] == 1
    assert "2 cm" in status["message"]


def test_missing_rtk_signal_fails_after_timeout(tmp_path):
    def no_signal():
        raise RuntimeError("/fix 超时无数据")

    monitor = OriginLockMonitor(
        no_signal,
        str(tmp_path / "gnss_origin.yaml"),
        sample_interval_seconds=0.01,
        no_signal_timeout_seconds=0.05,
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

    assert status["origin_status"] == "failed"
    assert status["error_code"] == "RTK_SIGNAL_TIMEOUT"
    assert "无信号" in status["message"]
