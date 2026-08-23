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

    def confirmed_teleop_action(self, action, success_states, failure_states=None, timeout_seconds=4.0):
        self.teleop_actions.append(action)
        state = next(iter(success_states))
        return {"topic": "/teleop_action", "action": action, "confirmed": True, "motion_state": state}

    def confirmed_remote_teleop_action(self, action, success_states, failure_states=None, timeout_seconds=4.0):
        return self.confirmed_teleop_action(action, success_states, failure_states, timeout_seconds)

    def release_to_remote_control(self, timeout_seconds=3.0):
        self.teleop_actions.append("release_remote")
        return {
            "topic": "/teleop_action",
            "action": "release_remote",
            "confirmed": True,
            "motion_state": "remote_control",
        }

    def teleop_velocity(self, vx=0.0, vy=0.0, yaw_rate=0.0):
        payload = {"topic": "/teleop_cmd_vel", "vx": vx, "vy": vy, "yaw_rate": yaw_rate}
        self.teleop_velocities.append(payload)
        return payload

    def obstacle_monitor_snapshot(self):
        return {}


class FakeTeleopControl:
    def __init__(self):
        self.ready_calls = 0

    def ensure_ready(self):
        self.ready_calls += 1
        return {"action": "start", "returncode": 0}


class FakePersonFollow:
    def __init__(self):
        self.calls = []

    def start(self, track_id):
        self.calls.append(("start", track_id))
        return {"status": "running", "track_id": track_id}

    def stop(self, reason):
        self.calls.append(("stop", reason))
        return {"status": "stopped", "reason": reason}

    def status(self):
        self.calls.append(("status",))
        return {"status": "running"}


class FakeMapActivation:
    def activate(self, command):
        return {
            "map_id": command["map_id"],
            "map_version": command["map_version"],
            "current_map": {
                "active_files": {
                    "map.pcd": "/maps/selected/map.pcd",
                    "map.yaml": "/maps/selected/map.yaml",
                },
            },
        }


class FakeNavigationStack:
    def __init__(self):
        self.reload_calls = []

    def reload_map(self, pcd_path, yaml_path):
        self.reload_calls.append((pcd_path, yaml_path))
        return {"action": "reload_map", "returncode": 0}


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


def test_task_start_clears_manual_control_before_validation(tmp_path):
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
        control_mode="manual_takeover",
        current_map_id="site-a-main",
        current_map_version="v1",
    )
    teleop_control = FakeTeleopControl()
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
        teleop_control_adapter=teleop_control,
    )

    ack, _ = processor.handle_command(raw)

    assert ack["payload"]["ack"] == "accepted"
    assert state.control_mode == "autonomous"
    assert navigation.teleop_velocities[-1] == {
        "topic": "/teleop_cmd_vel", "vx": 0.0, "vy": 0.0, "yaw_rate": 0.0,
    }
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
    assert results[0]["payload"]["result"]["topic"] == "/teleop_cmd_vel"
    store.close()


@pytest.mark.parametrize(
    ("message_type", "action", "motion_state"),
    [
        ("teleop.shake_hand", "shake_hand", "greeting"),
        ("teleop.two_leg_stand", "two_leg_stand", "two_leg_standing"),
    ],
)
def test_remote_trick_actions_use_the_remote_bridge(tmp_path, message_type, action, motion_state):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = message_type
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store,
        navigation,
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
        localization_adapter=navigation,
    )

    _, result = processor.handle_command(raw)

    assert navigation.teleop_actions == [action]
    assert result["payload"]["result"]["motion_state"] == motion_state
    assert state.control_mode == "manual_takeover"
    store.close()


def test_person_follow_start_and_stop_are_dispatched_locally(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "teleop.person_follow_start"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"track_id": "person-7"}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None
    )
    follow = FakePersonFollow()
    state = RuntimeSafetyState(localization_status="normal", nav_ready=True)
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
        person_follow_controller=follow,
    )

    _, result = processor.handle_command(raw)

    assert follow.calls == [("start", "person-7")]
    assert navigation.teleop_actions == ["stand_up"]
    assert state.control_mode == "manual_takeover"
    assert result["payload"]["result"]["follow"]["status"] == "running"

    raw["message_id"] = "55555555-5555-4555-8555-555555555555"
    raw["payload"]["command_id"] = "66666666-6666-4666-8666-666666666666"
    raw["message_type"] = "teleop.person_follow_stop"
    raw["payload"]["command"] = {}
    _, result = processor.handle_command(raw)

    assert follow.calls[-1] == ("stop", "operator_stop")
    assert result["payload"]["result"]["status"] == "stopped"
    store.close()


def test_skill_list_returns_the_local_executable_catalog_without_starting_control(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "teleop.skill_list"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None
    )
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), RuntimeSafetyState(localization_status="normal", nav_ready=True)),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
    )

    _, result = processor.handle_command(raw)

    presets = result["payload"]["result"]["presets"]
    assert len(presets) == 16
    assert any(item["name"] == "turn_right_full_circle" for item in presets)
    assert navigation.teleop_actions == []
    assert navigation.teleop_velocities == []
    store.close()


def test_takeover_exit_keeps_bridge_and_returns_control_to_remote(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "teleop.takeover_exit"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store,
        navigation,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    state = RuntimeSafetyState(localization_status="normal", nav_ready=True, control_mode="manual_takeover")
    teleop_control = FakeTeleopControl()
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
        teleop_control_adapter=teleop_control,
    )

    _, result = processor.handle_command(raw)

    assert navigation.teleop_velocities[-1] == {
        "topic": "/teleop_cmd_vel",
        "vx": 0.0,
        "vy": 0.0,
        "yaw_rate": 0.0,
    }
    assert navigation.teleop_actions[-1] == "release_remote"
    assert result["payload"]["result"]["confirmed"] is True
    assert "teleop_bridge_stop" not in result["payload"]["result"]
    assert state.control_mode == "autonomous"
    store.close()


def test_passive_keeps_bridge_resident(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "teleop.passive"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store,
        navigation,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    state = RuntimeSafetyState(localization_status="normal", nav_ready=True, control_mode="manual_takeover")
    teleop_control = FakeTeleopControl()
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
        teleop_control_adapter=teleop_control,
    )

    _, result = processor.handle_command(raw)

    assert teleop_control.ready_calls == 1
    assert navigation.teleop_actions[-1] == "passive"
    assert "teleop_bridge_stop" not in result["payload"]["result"]
    assert state.control_mode == "autonomous"
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
        "topic": "/teleop_cmd_vel",
        "vx": 0.5,
        "vy": -0.5,
        "yaw_rate": 0.5,
    }
    assert result["payload"]["status"] == "succeeded"
    store.close()


def test_map_activation_reloads_both_map_consumers_and_requires_reseed(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "map.activate"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"map_id": "95", "map_version": "selected-map"}
    store = LocalStore(str(tmp_path / "edge.db"))
    state = RuntimeSafetyState(localization_status="normal", nav_ready=True)
    stack = FakeNavigationStack()
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=TaskExecutor(
            store,
            FakeNavigation(),
            event_callback=lambda *args: None,
            start_result_callback=lambda *args: None,
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        map_activation_adapter=FakeMapActivation(),
        navigation_stack_adapter=stack,
    )

    _, result = processor.handle_command(raw)

    assert stack.reload_calls == [("/maps/selected/map.pcd", "/maps/selected/map.yaml")]
    assert result["payload"]["result"]["localization_reset_required"] is True
    assert state.localization_status == "initializing"
    store.close()
