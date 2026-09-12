import sqlite3

from roamerx_edge.local_store import LocalStore


def test_sqlite_restart_recovery(tmp_path):
    path = tmp_path / "edge.db"
    first = LocalStore(str(path))
    first.save_command_ack("cmd-1", {"accepted": True})
    first.save_task_context(
        {
            "task_execution_id": "exec-1",
            "state": "running",
            "state_version": 2,
            "route_snapshot": {"waypoints": [{"sequence": 0}]},
            "current_waypoint_index": 0,
            "start_command_id": "cmd-1",
            "record_rosbag": True,
            "loop_execution": True,
            "loop_session_id": "63b66a16-1947-4be7-889b-d851a5f4ba20",
            "continuous_rosbag": True,
        }
    )
    first.enqueue_outbox("topic", {"message_type": "task.progress"}, dedupe_key="event-1")
    first.close()

    second = LocalStore(str(path))
    assert second.get_processed_command("cmd-1")["ack"]["accepted"] is True
    restored = second.load_active_task_context()
    assert restored["state"] == "running"
    assert restored["record_rosbag"] is True
    assert restored["loop_execution"] is True
    assert restored["loop_session_id"] == "63b66a16-1947-4be7-889b-d851a5f4ba20"
    assert restored["continuous_rosbag"] is True
    assert second.outbox_count() == 1
    second.close()


def test_task_context_migration_and_post_arrival_state_persist(tmp_path):
    path = tmp_path / "legacy-edge.db"
    connection = sqlite3.connect(path)
    connection.execute(
        """
        CREATE TABLE task_context (
            task_execution_id TEXT PRIMARY KEY,
            state TEXT NOT NULL,
            state_version INTEGER NOT NULL,
            route_snapshot_json TEXT NOT NULL,
            current_waypoint_index INTEGER NOT NULL DEFAULT 0,
            start_command_id TEXT,
            record_rosbag INTEGER NOT NULL DEFAULT 0,
            updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
        )
        """
    )
    connection.commit()
    connection.close()

    store = LocalStore(str(path))
    store.save_task_context(
        {
            "task_execution_id": "exec-post-arrival",
            "state": "paused",
            "state_version": 7,
            "route_snapshot": {"waypoints": [{"sequence": 0}]},
            "current_waypoint_index": 0,
            "start_command_id": "cmd-1",
            "post_arrival_waypoint_index": 0,
            "post_arrival_stage": "xy_adjusting",
            "arrival_side_effects_started": True,
            "arrival_micro_adjust_total_m": 0.30,
            "arrival_micro_adjust_steps": 2,
            "arrival_micro_adjust_started_at": 1234.5,
        }
    )

    restored = store.load_active_task_context()

    assert restored["post_arrival_waypoint_index"] == 0
    assert restored["post_arrival_stage"] == "xy_adjusting"
    assert restored["arrival_side_effects_started"] is True
    assert restored["arrival_micro_adjust_total_m"] == 0.30
    assert restored["arrival_micro_adjust_steps"] == 2
    assert restored["arrival_micro_adjust_started_at"] == 1234.5
    assert restored["loop_execution"] is False
    assert restored["loop_session_id"] == ""
    assert restored["continuous_rosbag"] is False
    store.close()


def test_trajectory_sequence_persists(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    assert store.next_trajectory_seq("exec") == 0
    assert store.next_trajectory_seq("exec") == 1
    store.close()


def test_command_result_is_persisted_even_when_ack_was_not_saved(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    result = {"message_type": "command.result", "payload": {"status": "failed"}}

    store.save_command_result("cmd-before-ack", result)

    saved = store.get_processed_command("cmd-before-ack")
    assert saved["ack"] is None
    assert saved["result"] == result
    store.close()


def test_last_trusted_pose_is_scoped_by_map(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    pose = {"x": 12.5, "y": -3.0, "yaw": 0.7, "source": "test"}
    store.save_last_trusted_pose("92", "v1", pose)

    assert store.load_last_trusted_pose("92", "v1") == pose
    assert store.load_last_trusted_pose("92", "v2") is None
    store.close()


def test_outbox_prioritizes_control_events_ahead_of_trajectory(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    for index in range(3):
        store.enqueue_outbox(
            "trajectory",
            {"message_type": "trajectory.batch", "payload": {"batch_id": f"batch-{index}"}},
            dedupe_key=f"batch-{index}",
        )
    store.enqueue_outbox("task", {"message_type": "task.failed", "payload": {}})

    pending = store.list_pending_outbox(limit=1)

    assert pending[0]["payload"]["message_type"] == "task.failed"
    store.close()


def test_trajectory_outbox_is_bounded_and_pruned_again_on_restart(tmp_path):
    path = tmp_path / "edge.db"
    store = LocalStore(str(path), trajectory_outbox_limit=4)
    for index in range(6):
        store.enqueue_outbox(
            "trajectory",
            {"message_type": "trajectory.batch", "payload": {"batch_id": f"batch-{index}"}},
            dedupe_key=f"batch-{index}",
        )
    assert store.outbox_count() == 4
    store.close()

    restarted = LocalStore(str(path), trajectory_outbox_limit=2)
    pending = restarted.list_pending_outbox()

    assert restarted.outbox_count() == 2
    assert [row["dedupe_key"] for row in pending] == ["batch-4", "batch-5"]
    restarted.close()
