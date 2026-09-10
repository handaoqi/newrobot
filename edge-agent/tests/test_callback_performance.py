from roamerx_edge.callback_performance import CallbackPerformanceMonitor


def test_callback_metrics_are_bounded_and_report_latest_value_drops():
    monitor = CallbackPerformanceMonitor()

    monitor.record(
        "odometry_callback",
        0.001,
        processed=False,
        dropped=2,
        queue_depth=1,
    )
    monitor.record("odometry_processing", 0.004, queue_depth=0)
    summary = monitor.snapshot_and_reset()

    assert summary["callbacks"]["odometry_callback"]["calls"] == 1
    assert summary["callbacks"]["odometry_callback"]["dropped"] == 2
    assert summary["callbacks"]["odometry_processing"]["p99_ms"] == 4.0
    assert summary["callback_queue_depth"] == 1
    assert summary["stale_sample_dropped"] == 2


def test_callback_metrics_reset_after_snapshot():
    monitor = CallbackPerformanceMonitor()
    monitor.record("scan_callback", 0.002)

    monitor.snapshot_and_reset()
    empty = monitor.snapshot_and_reset()

    assert empty["callbacks"] == {}
    assert empty["stale_sample_dropped"] == 0
