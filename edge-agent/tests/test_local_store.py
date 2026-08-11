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
        }
    )
    first.enqueue_outbox("topic", {"message_type": "task.progress"}, dedupe_key="event-1")
    first.close()

    second = LocalStore(str(path))
    assert second.get_processed_command("cmd-1")["ack"]["accepted"] is True
    assert second.load_active_task_context()["state"] == "running"
    assert second.outbox_count() == 1
    second.close()


def test_trajectory_sequence_persists(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    assert store.next_trajectory_seq("exec") == 0
    assert store.next_trajectory_seq("exec") == 1
    store.close()


def test_last_trusted_pose_is_scoped_by_map(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    pose = {"x": 12.5, "y": -3.0, "yaw": 0.7, "source": "test"}
    store.save_last_trusted_pose("92", "v1", pose)

    assert store.load_last_trusted_pose("92", "v1") == pose
    assert store.load_last_trusted_pose("92", "v2") is None
    store.close()
