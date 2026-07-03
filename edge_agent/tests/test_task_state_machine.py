from roamerx_edge.local_store import LocalStore
from roamerx_edge.protocol import decode_message
from roamerx_edge.task_executor import TaskExecutor


class FakeNavigation:
    def __init__(self):
        self.feedback = None
        self.result = None
        self.sent = []
        self.cancelled = 0
        self.stopped = True

    def send_waypoints(self, waypoints, feedback_cb, result_cb):
        self.sent.append(waypoints)
        self.feedback = feedback_cb
        self.result = result_cb
        return True

    def cancel_navigation(self, timeout_seconds=5):
        self.cancelled += 1
        return True

    def is_robot_stopped(self):
        return self.stopped


def command(message_type, state_version=0, resume_index=None):
    import json
    from pathlib import Path

    payload = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    payload["message_type"] = message_type
    payload["payload"]["expected_robot_state_version"] = state_version
    if message_type != "task.start":
        payload["payload"]["command"] = (
            {"resume_from_waypoint_index": resume_index}
            if message_type == "task.resume"
            else {"reason": "test"}
        )
    return decode_message(payload)


def test_pause_resume_cancel(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(command("task.start"))
    assert executor.context.state == "running"
    nav.feedback(1, None)
    paused = executor.pause_task(executor.context.task_execution_id)
    assert paused["final_task_state"] == "paused"
    assert nav.cancelled == 1
    resumed = executor.resume_task(executor.context.task_execution_id, 1)
    assert resumed["final_task_state"] == "running"
    assert len(nav.sent[-1]) == 2
    cancelled = executor.cancel_task(executor.context.task_execution_id)
    assert cancelled["final_task_state"] == "cancelled"
    store.close()


def test_navigation_success_finishes_start_command(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(command("task.start"))
    nav.result("succeeded", "")
    assert executor.context.state == "completed"
    assert results[0][1] == "succeeded"
    store.close()
