from types import SimpleNamespace
from unittest.mock import Mock

from roamerx_edge.app import EdgeAgentApplication
from roamerx_edge.local_store import LocalStore
from roamerx_edge.task_executor import TaskExecutor


def _application():
    app = EdgeAgentApplication.__new__(EdgeAgentApplication)
    app._latest_task_state_event = {}
    app.task_executor = SimpleNamespace(
        context=SimpleNamespace(
            task_execution_id="execution-1",
            state="paused",
            state_version=8,
            trace_id="trace-1",
        )
    )
    app.store = SimpleNamespace(outbox_count=lambda: 0)
    app.mqtt = SimpleNamespace(
        session_id="session-1",
        publish=Mock(),
        publish_task_event=Mock(),
    )
    app.config = SimpleNamespace(robot=SimpleNamespace(id="robot-1"))
    return app


def test_sync_repeats_the_latest_pause_reason():
    app = _application()
    app._publish_task_event(
        "task.paused",
        {
            "task_execution_id": "execution-1",
            "state": "paused",
            "state_version": 8,
            "reason_code": "ARRIVAL_CONFIRMATION_UNSTABLE",
            "reason_message": "到点位姿未稳定",
        },
    )

    app._publish_sync_request()

    envelope = app.mqtt.publish.call_args.args[1]
    assert envelope["payload"]["local_task_state"] == "paused"
    assert envelope["payload"]["local_task_state_version"] == 8
    assert envelope["payload"]["local_task_reason_code"] == "ARRIVAL_CONFIRMATION_UNSTABLE"
    assert envelope["payload"]["local_task_reason_message"] == "到点位姿未稳定"


def test_heartbeat_repeats_task_state_sync():
    app = _application()
    app.config.telemetry = SimpleNamespace(heartbeat_interval_seconds=10)
    app._current_map_payload = lambda: {}
    app.mqtt.publish_presence = Mock()
    app._publish_sync_request = Mock()

    class StopAfterOneHeartbeat:
        calls = 0

        def wait(self, _interval):
            self.calls += 1
            return self.calls > 1

    app.stop_event = StopAfterOneHeartbeat()
    app._heartbeat_loop()

    app.mqtt.publish_presence.assert_called_once()
    app._publish_sync_request.assert_called_once()


def test_restart_preserves_an_already_stopped_paused_task(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    store.save_task_context({
        "task_execution_id": "execution-paused",
        "state": "paused",
        "state_version": 11,
        "route_snapshot": {"waypoints": [{"x": 1.0, "y": 2.0}]},
        "current_waypoint_index": 0,
        "start_command_id": "command-1",
    })
    results = []

    executor = TaskExecutor(
        store,
        SimpleNamespace(),
        event_callback=lambda *_args: None,
        start_result_callback=lambda *args: results.append(args),
    )
    executor.report_startup_interruption()

    assert executor.context.state == "paused"
    assert executor.context.state_version == 11
    assert results == []
    store.close()
