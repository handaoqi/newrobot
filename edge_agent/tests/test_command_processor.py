import json
from pathlib import Path

from roamerx_edge.command_processor import CommandProcessor
from roamerx_edge.local_store import LocalStore
from roamerx_edge.safety_policy import RuntimeSafetyState, SafetyPolicy
from roamerx_edge.config import SafetyConfig
from roamerx_edge.task_executor import TaskExecutor


class FakeNavigation:
    def send_waypoints(self, waypoints, feedback_cb, result_cb):
        self.result_cb = result_cb
        return True

    def cancel_navigation(self, timeout_seconds=5):
        return True

    def is_robot_stopped(self):
        return True


def test_duplicate_command_is_not_executed_twice(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store,
        navigation,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    state = RuntimeSafetyState(
        localization_status="normal",
        nav_ready=True,
        control_mode="autonomous",
        current_map_id="site-a-main",
        current_map_version="v1",
    )
    acks = []
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda command_id, payload: acks.append(payload),
        publish_result=lambda *args: None,
    )
    first, _ = processor.handle_command(raw)
    second, _ = processor.handle_command(raw)
    assert first == second
    assert len(acks) == 2
    assert executor.context.task_execution_id == raw["payload"]["task_execution_id"]
    store.close()


def test_expired_command_is_rejected(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["payload"]["expires_at"] = "2026-06-22T10:30:30+08:00"
    store = LocalStore(str(tmp_path / "edge.db"))
    executor = TaskExecutor(
        store,
        FakeNavigation(),
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    state = RuntimeSafetyState(localization_status="normal", nav_ready=True)
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
    )
    ack, _ = processor.handle_command(raw)
    assert ack["payload"]["ack"] == "rejected"
    assert ack["payload"]["reason_code"] == "COMMAND_EXPIRED"
    store.close()
