from pathlib import Path
import importlib.util


SCRIPT = Path(__file__).parents[1] / "scripts" / "self_filter_scan.py"
SPEC = importlib.util.spec_from_file_location("self_filter_scan", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_scan_end_stamp_adds_frame_duration():
    assert MODULE.scan_end_stamp(10, 200_000_000, 0.1) == (10, 300_000_000)


def test_scan_end_stamp_normalizes_nanosecond_rollover():
    assert MODULE.scan_end_stamp(10, 950_000_000, 0.1) == (11, 50_000_000)


def test_scan_end_stamp_never_moves_backwards():
    assert MODULE.scan_end_stamp(10, 200_000_000, -0.1) == (10, 200_000_000)
