from roamerx_edge.local_store import LocalStore
from roamerx_edge.telemetry_collector import PoseSnapshot
from roamerx_edge.trajectory_buffer import TrajectoryBuffer


def pose(x: float, y: float) -> PoseSnapshot:
    return PoseSnapshot(
        sampled_at="2026-09-06T00:00:00+00:00",
        x=x,
        y=y,
        z=0.0,
        yaw=0.0,
        speed_mps=0.2,
        localization_status="normal",
        source_status=0,
        coord_type=0,
    )


def test_task_switch_flushes_previous_execution_before_sampling_new_one(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    buffer = TrajectoryBuffer(
        robot_id="rx-001",
        session_id="session",
        store=store,
        batch_size=20,
        flush_seconds=60,
    )

    assert buffer.sample("execution-a", "map-a", "v1", pose(1.0, 2.0)) is None
    assert buffer.sample("execution-b", "map-b", "v2", pose(3.0, 4.0)) is None

    pending = store.list_pending_outbox(limit=10)
    flushed = pending[0]["payload"]
    assert flushed["payload"]["task_execution_id"] == "execution-a"
    assert flushed["payload"]["first_seq"] == 0
    assert flushed["payload"]["last_seq"] == 0
    assert flushed["payload"]["points"][0]["seq"] == 0

    current = buffer.flush_active()
    assert current["payload"]["task_execution_id"] == "execution-b"
    assert current["payload"]["first_seq"] == 0
    assert current["payload"]["last_seq"] == 0
    assert store.outbox_count() == 2
    store.close()


def test_successful_ack_removes_outbox_row(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    buffer = TrajectoryBuffer(
        robot_id="rx-001",
        session_id="session",
        store=store,
        batch_size=1,
        flush_seconds=60,
    )
    message = buffer.sample("execution-a", "map-a", "v1", pose(1.0, 2.0))
    batch_id = message["payload"]["batch_id"]
    assert store.outbox_count() == 1

    buffer.handle_ack({"batch_id": batch_id, "accepted": True})
    assert store.outbox_count() == 0
    store.close()


def test_transient_rejection_keeps_outbox_row(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    buffer = TrajectoryBuffer(
        robot_id="rx-001",
        session_id="session",
        store=store,
        batch_size=1,
        flush_seconds=60,
    )
    message = buffer.sample("execution-a", "map-a", "v1", pose(1.0, 2.0))
    batch_id = message["payload"]["batch_id"]

    buffer.handle_ack({
        "batch_id": batch_id,
        "accepted": False,
        "reason_code": "CENTER_BUSY",
    })
    assert store.outbox_count() == 1
    store.close()


def test_unknown_task_execution_rejection_drops_outbox_row(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    buffer = TrajectoryBuffer(
        robot_id="rx-001",
        session_id="session",
        store=store,
        batch_size=1,
        flush_seconds=60,
    )
    message = buffer.sample("execution-a", "map-a", "v1", pose(1.0, 2.0))
    batch_id = message["payload"]["batch_id"]

    buffer.handle_ack({
        "batch_id": batch_id,
        "accepted": False,
        "reason_code": "UNKNOWN_TASK_EXECUTION",
    })
    assert store.outbox_count() == 0
    store.close()


def test_invalid_message_rejection_drops_outbox_row(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    buffer = TrajectoryBuffer(
        robot_id="rx-001",
        session_id="session",
        store=store,
        batch_size=1,
        flush_seconds=60,
    )
    message = buffer.sample("execution-a", "map-a", "v1", pose(1.0, 2.0))
    batch_id = message["payload"]["batch_id"]

    buffer.handle_ack({
        "batch_id": batch_id,
        "accepted": False,
        "reason_code": "INVALID_MESSAGE",
    })
    assert store.outbox_count() == 0
    store.close()


def test_flush_active_emits_partial_terminal_batch(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    buffer = TrajectoryBuffer(
        robot_id="rx-001",
        session_id="session",
        store=store,
        batch_size=20,
        flush_seconds=60,
    )

    buffer.sample("execution-a", "map-a", "v1", pose(1.0, 2.0))
    terminal = buffer.flush_active()

    assert terminal["payload"]["task_execution_id"] == "execution-a"
    assert terminal["payload"]["points"][0]["x"] == 1.0
    assert buffer.flush_active() is None
    store.close()


def test_keyframe_is_carried_with_trajectory_sample(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    buffer = TrajectoryBuffer(
        robot_id="rx-001",
        session_id="session",
        store=store,
        batch_size=1,
        flush_seconds=60,
    )

    message = buffer.sample(
        "execution-a",
        "map-a",
        "v1",
        pose(1.0, 2.0),
        keyframe={
            "slam": {"x": 1.0, "y": 2.0, "yaw": 0.0},
            "rtk": {"quality": "fixed", "x": 1.1, "y": 2.1, "yaw": 0.0},
            "task": {"state": "accepted", "waypoint_index": 0},
        },
    )

    point = message["payload"]["points"][0]
    assert point["keyframe"]["rtk"]["quality"] == "fixed"
    assert point["keyframe"]["task"]["state"] == "accepted"
    store.close()
