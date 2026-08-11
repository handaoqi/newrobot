import json
import time
from pathlib import Path
from types import SimpleNamespace
import pytest
from roamerx_edge.command_processor import CommandProcessor
from roamerx_edge.local_store import LocalStore
from roamerx_edge.safety_policy import RuntimeSafetyState, SafetyPolicy
from roamerx_edge.config import SafetyConfig
from roamerx_edge.task_executor import TaskExecutor
from roamerx_edge.protocol import decode_message, ProtocolError


class FakeNavigation:
    def __init__(self):
        self.initial_pose = None
        self.teleop_actions = []
        self.teleop_velocities = []
        self.pose = SimpleNamespace(x=3.0, y=4.0)
        self.rtk_initial_pose_requests = 0

    def send_waypoints(self, waypoints, feedback_cb, result_cb):
        self.result_cb = result_cb
        return True

    def cancel_navigation(self, timeout_seconds=5):
        return True

    def latest_pose(self):
        return self.pose

    def is_robot_stopped(self):
        return True

    def set_initial_pose(self, pose):
        self.initial_pose = pose
        return {"topic": "/initialpose", **pose}

    def set_initial_pose_from_rtk(self, wait_seconds=30.0):
        self.rtk_initial_pose_requests += 1
        return {"source": "rtk_fixed", "wait_seconds": wait_seconds}

    def active_relocalize(self, pose):
        self.initial_pose = pose
        return {"mode": "stationary_bounded_search", "motion_commanded": False, **pose}

    def teleop_action(self, action):
        self.teleop_actions.append(action)
        return {"topic": "/teleop_action", "action": action}

    def teleop_velocity(self, vx=0.0, vy=0.0, yaw_rate=0.0):
        payload = {"topic": "/cmd_vel", "vx": vx, "vy": vy, "yaw_rate": yaw_rate}
        self.teleop_velocities.append(payload)
        return payload


def test_task_start_rejects_transient_localization(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    envelope = decode_message(raw)
    state = RuntimeSafetyState(
        localization_status="normal",
        localization_normal_since_monotonic=time.monotonic(),
        nav_ready=True,
        control_mode="autonomous",
        current_map_id="site-a-main",
        current_map_version="v1",
    )

    with pytest.raises(ProtocolError, match="requires 3.0s") as error:
        SafetyPolicy(SafetyConfig(), state).validate_task_start(envelope, False)

    assert error.value.code == "LOCALIZATION_NOT_STABLE"


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


def test_task_progress_version_follows_start_ack_version(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    events = []
    acks = []
    executor = TaskExecutor(
        store,
        navigation,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(
            SafetyConfig(),
            RuntimeSafetyState(
                localization_status="normal",
                nav_ready=True,
                control_mode="autonomous",
                current_map_id="site-a-main",
                current_map_version="v1",
            ),
        ),
        task_executor=executor,
        publish_ack=lambda _id, payload: acks.append(payload),
        publish_result=lambda *args: None,
    )

    processor.handle_command(raw)
    navigation.result_cb("succeeded", "", {"missed_waypoints": []})

    assert acks[0]["payload"]["edge_state_version"] == 2
    assert events[0][0] == "task.started"
    assert events[0][1]["state_version"] == 3
    assert events[-1][0] == "task.completed"
    assert events[-1][1]["state_version"] > acks[0]["payload"]["edge_state_version"]
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
    acks = []
    results = []
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda _id, payload: acks.append(payload),
        publish_result=lambda _id, payload: results.append(payload),
    )
    ack, result = processor.handle_command(raw)
    assert ack["payload"]["ack"] == "rejected"
    assert ack["payload"]["reason_code"] == "COMMAND_EXPIRED"
    assert result is None
    assert acks == [ack]
    assert results == []
    assert store.get_processed_command(raw["payload"]["command_id"])["ack"] == ack
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


def test_nav_initial_pose_bypasses_stack_management_lock(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.initial_pose"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"x": 1.0, "y": 2.0, "yaw": 0.5}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None
    )
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), RuntimeSafetyState()),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
    )
    processor._navigation_command_lock.acquire()
    try:
        _, result = processor.handle_command(raw)
    finally:
        processor._navigation_command_lock.release()

    assert result["payload"]["status"] == "succeeded"
    assert navigation.initial_pose["x"] == 1.0
    store.close()


def test_nav_initial_pose_uses_fixed_rtk_seed(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.initial_pose"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"seed_source": "rtk", "wait_seconds": 12.0}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None
    )
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), RuntimeSafetyState()),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
    )

    _, result = processor.handle_command(raw)

    assert navigation.rtk_initial_pose_requests == 1
    assert result["payload"]["result"]["source"] == "rtk_fixed"
    assert result["payload"]["result"]["wait_seconds"] == 12.0
    store.close()


def test_active_relocalization_uses_map_scoped_trusted_pose(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.relocalize"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"map_id": "92", "map_version": "v1"}
    store = LocalStore(str(tmp_path / "edge.db"))
    store.save_last_trusted_pose("92", "v1", {"x": 8.0, "y": 9.0, "yaw": -0.4})
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None
    )
    state = RuntimeSafetyState(current_map_id="92", current_map_version="v1")
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
    )

    _, result = processor.handle_command(raw)

    assert result["payload"]["status"] == "succeeded"
    assert navigation.initial_pose == {"x": 8.0, "y": 9.0, "yaw": -0.4}
    assert result["payload"]["result"]["motion_commanded"] is False
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


def test_follow_velocity_is_combined_and_safety_limited(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "teleop.move_velocity"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"vx": 0.8, "vy": -0.8, "yaw_rate": 1.2}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store,
        navigation,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(
            SafetyConfig(),
            RuntimeSafetyState(localization_status="normal", nav_ready=True, control_mode="manual_takeover"),
        ),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
    )
    _, result = processor.handle_command(raw)
    assert navigation.teleop_velocities[-1] == {
        "topic": "/cmd_vel",
        "vx": 0.2,
        "vy": -0.15,
        "yaw_rate": 0.35,
    }
    assert result["payload"]["status"] == "succeeded"
    store.close()
