import json
from pathlib import Path

from roamerx_edge.command_processor import CommandProcessor
from roamerx_edge.local_store import LocalStore
from roamerx_edge.safety_policy import RuntimeSafetyState, SafetyPolicy
from roamerx_edge.config import SafetyConfig
from roamerx_edge.task_executor import TaskExecutor


class FakeNavigation:
    def __init__(self):
        self.initial_pose = None
        self.teleop_actions = []
        self.teleop_velocities = []

    def send_waypoints(self, waypoints, feedback_cb, result_cb):
        self.result_cb = result_cb
        return True

    def cancel_navigation(self, timeout_seconds=5):
        return True

    def is_robot_stopped(self):
        return True

    def set_initial_pose(self, pose):
        self.initial_pose = pose
        return {"topic": "/initialpose", **pose}

    def teleop_action(self, action):
        self.teleop_actions.append(action)
        return {"topic": "/teleop_action", "action": action}

    def teleop_velocity(self, vx=0.0, vy=0.0, yaw_rate=0.0):
        payload = {"topic": "/cmd_vel", "vx": vx, "vy": vy, "yaw_rate": yaw_rate}
        self.teleop_velocities.append(payload)
        return payload


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
    ack, result = processor.handle_command(raw)
    assert ack == {}
    assert result["payload"]["status"] == "failed"
    assert result["payload"]["error_code"] == "COMMAND_EXPIRED"
    store.close()


def test_nav_initial_pose_is_dispatched_to_navigation_adapter(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.initial_pose"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"x": 1.0, "y": 2.0, "yaw": 0.5, "frame_id": "map"}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store,
        navigation,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    state = RuntimeSafetyState(localization_status="normal", nav_ready=True)
    results = []
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda command_id, payload: results.append(payload),
        localization_adapter=navigation,
    )
    _, result = processor.handle_command(raw)
    assert navigation.initial_pose["x"] == 1.0
    assert result["payload"]["status"] == "succeeded"
    assert results[0]["payload"]["result"]["topic"] == "/initialpose"
    store.close()


def test_teleop_move_is_dispatched_to_navigation_adapter(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "teleop.move_forward"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"vx": 0.2}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store,
        navigation,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    state = RuntimeSafetyState(localization_status="normal", nav_ready=True, control_mode="manual_takeover")
    results = []
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda command_id, payload: results.append(payload),
        localization_adapter=navigation,
    )
    _, result = processor.handle_command(raw)
    assert navigation.teleop_velocities[-1]["vx"] == 0.2
    assert result["payload"]["status"] == "succeeded"
    assert results[0]["payload"]["result"]["topic"] == "/cmd_vel"
    store.close()
