import json
import time
from dataclasses import asdict

import pytest

from roamerx_edge.origin_lock import OriginSignalUnavailable
from roamerx_edge.rtk_origin import RtkOriginPayloadCache, build_origin_sample


def payloads():
    fix = {
        "status": {"status": 2},
        "latitude": 39.9,
        "longitude": 116.4,
        "altitude": 42.0,
    }
    pvh = {
        "header": {"stamp": {"sec": int(time.time()), "nanosec": 0}},
        "bestnav": {
            "p_sol_status": 0,
            "pos_type": 48,
            "latitude_deg": 39.9,
            "longitude_deg": 116.4,
            "altitude_m": 42.0,
            "lat_std": 0.008,
            "lon_std": 0.009,
            "hgt_std": 0.015,
            "diff_age_s": 0.2,
            "soln_svs_num": 18,
        },
        "heading": {
            "sol_status": 0,
            "heading_type": 4,
            "base_line": 0.8,
            "heading_deg": 90.0,
            "heading_std": 0.4,
        },
    }
    ntrip = {
        "data": (
            "quality: rtk_fixed\nage_sec: 0.1\n"
            "measurement_time_source: receiver_header\n"
        )
    }
    return fix, pvh, ntrip


def test_live_payload_cache_returns_complete_snapshot():
    cache = RtkOriginPayloadCache(stale_after_seconds=1.0)
    fix, pvh, ntrip = payloads()
    cache.update("fix", fix)
    cache.update("pvh", pvh)
    cache.update("ntrip", ntrip)

    cached_fix, cached_pvh, cached_ntrip, sampled_at = cache.snapshot()
    sample = build_origin_sample(cached_fix, cached_pvh, cached_ntrip, sampled_at=sampled_at)

    assert sample.position_fixed is True
    assert sample.heading_fixed is True
    assert sample.solution_satellites == 18
    assert sample.measurement_time_source == "receiver_header"
    assert sampled_at > 0


def test_live_payload_cache_reports_missing_stream_duration():
    cache = RtkOriginPayloadCache(stale_after_seconds=1.0)
    fix, _, _ = payloads()
    cache.update("fix", fix)
    time.sleep(0.02)

    with pytest.raises(OriginSignalUnavailable) as error:
        cache.snapshot()

    assert "pvh" in str(error.value)
    assert "ntrip" in str(error.value)
    assert error.value.unavailable_seconds >= 0.01


def test_invalid_zero_position_is_preserved_and_strict_json_safe():
    fix = {
        "status": {"status": -1},
        "latitude": 0.0,
        "longitude": 0.0,
        "altitude": 0.0,
    }
    pvh = {
        "header": {"stamp": {"sec": 0, "nanosec": 0}},
        "bestnav": {
            "p_sol_status": 2,
            "pos_type": 0,
            "latitude_deg": 0.0,
            "longitude_deg": 0.0,
            "altitude_m": 0.0,
            "lat_std": 99.9,
            "lon_std": 99.9,
            "hgt_std": 99.9,
        },
        "heading": {
            "sol_status": 2,
            "heading_type": 0,
            "base_line": 0.0,
            "heading_deg": 0.0,
            "heading_std": 180.0,
        },
    }
    sample = build_origin_sample(fix, pvh, {"data": "quality: invalid\n"}, now=lambda: 1.0)

    assert sample.latitude == 0.0
    assert sample.longitude == 0.0
    assert sample.position_fixed is False
    assert sample.heading_fixed is False
    json.dumps(asdict(sample), allow_nan=False)
