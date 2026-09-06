from datetime import datetime, timedelta, timezone

import pytest

from roamerx_edge.local_store import LocalStore
from roamerx_edge.boundary_filter_mask import write_keepout_mask
from roamerx_edge.navigation_boundary import NavigationBoundaryManager
from roamerx_edge.protocol import ProtocolError
from roamerx_edge.structured_logging import StructuredLogEmitter


def test_debug_logging_is_explicit_sampled_and_bounded():
    published = []
    emitter = StructuredLogEmitter(lambda entries, urgent: published.append((entries, urgent)), batch_size=2)
    assert emitter.emit("DEBUG", "planner", "planner.sample", "sample", data={"open": 3}) is False
    config = emitter.configure_debug(
        enabled=True,
        modules=["planner"],
        sample_hz=1,
        expires_at=(datetime.now(timezone.utc) + timedelta(minutes=5)).isoformat(),
    )
    assert config["enabled"] is True
    assert emitter.emit("DEBUG", "planner", "planner.sample", "sample", data={"open": 3}) is True
    assert emitter.emit("DEBUG", "planner", "planner.sample", "sample", data={"open": 4}) is False
    assert emitter.emit("INFO", "navigation", "navigation.started", "started") is True
    assert len(published) == 1
    assert published[0][1] is False
    assert len(published[0][0]) == 2


def test_warning_flushes_immediately():
    published = []
    emitter = StructuredLogEmitter(lambda entries, urgent: published.append((entries, urgent)))
    emitter.emit("WARNING", "avoidance", "avoidance.blocked", "blocked")
    assert published[0][1] is True
    assert published[0][0][0]["level"] == "WARNING"


def test_boundary_persists_and_enforces_margin_route_and_speed(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    manager = NavigationBoundaryManager(store)
    result = manager.apply({
        "map_id": "map-1",
        "map_version": "v1",
        "boundary": {
            "revision": 3,
            "outer_polygon": [[0, 0], [10, 0], [10, 10], [0, 10]],
            "safety_margin_m": 0.2,
            "zones": [
                {"id": 1, "name": "pond", "zone_type": "forbidden", "active": True,
                 "polygon": [[4, 4], [6, 4], [6, 6], [4, 6]]},
                {"id": 2, "name": "slow", "zone_type": "restricted", "active": True,
                 "speed_limit_mps": 0.12, "polygon": [[7, 7], [9, 7], [9, 9], [7, 9]]},
            ],
        },
    })
    assert result["active_revision"] == 3
    assert NavigationBoundaryManager(store).snapshot()["revision"] == 3

    manager.validate_point("map-1", 2, 2, expected_revision=3)
    with pytest.raises(ProtocolError, match="外边界"):
        manager.validate_point("map-1", 0.1, 2, expected_revision=3)
    with pytest.raises(ProtocolError, match="禁入区"):
        manager.validate_point("map-1", 3.9, 5, expected_revision=3)
    with pytest.raises(ProtocolError, match="禁入区"):
        manager.validate_route({
            "map": {"map_id": "map-1"}, "boundary_revision": 3,
            "waypoints": [{"x": 2, "y": 5}, {"x": 8, "y": 5}],
        })
    observation = manager.observe_pose("map-1", 8, 8)
    assert observation["speed_limit_mps"] == 0.12
    assert observation["speed_changed"] is True
    store.close()


def test_keepout_mask_rasterizes_outer_boundary_forbidden_zone_and_margin(tmp_path):
    result = write_keepout_mask({
        "map_id": "map-1", "revision": 1,
        "map": {"width": 10, "height": 10, "resolution": 1.0, "origin": [0, 0, 0]},
        "outer_polygon": [[1, 1], [9, 1], [9, 9], [1, 9]],
        "safety_margin_m": 0.6,
        "zones": [{
            "zone_type": "forbidden", "active": True,
            "polygon": [[4, 4], [6, 4], [6, 6], [4, 6]],
        }],
    }, str(tmp_path))
    raw = (tmp_path / "keepout_mask.pgm").read_bytes()
    header, pixels = raw.split(b"255\n", 1)
    assert b"10 10" in header
    assert pixels[0] == 0  # outside the outer boundary
    assert pixels[2 * 10 + 2] == 255  # safely inside
    assert pixels[4 * 10 + 4] == 0  # forbidden island (PGM row is top-down)
    assert result["occupied_cells"] > 0


def test_offline_system_log_outbox_keeps_only_the_newest_bounded_batches(tmp_path):
    store = LocalStore(str(tmp_path / "outbox.db"), system_log_outbox_limit=2)
    for index in range(3):
        store.enqueue_outbox(
            "robots/rx/events/system-log",
            {"message_type": "system.log.batch", "payload": {"entries": [{"index": index}]}},
            qos=0,
        )
    pending = store.list_pending_outbox()
    assert len(pending) == 2
    assert [row["payload"]["payload"]["entries"][0]["index"] for row in pending] == [1, 2]
    store.close()
