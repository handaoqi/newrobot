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
        self.pose = SimpleNamespace(x=3.0, y=4.0, yaw=0.0)
        self.rtk_initial_pose_requests = 0
        self.global_relocalize_requests = []
        self.progressive_relocalize_requests = []
        self.quick_then_global_requests = []
        self.operator_localization_events = []
        self.global_controllers = []

    def begin_operator_localization(self):
        self.operator_localization_events.append("begin")

    def end_operator_localization(self):
        self.operator_localization_events.append("end")

    def send_waypoints(self, waypoints, feedback_cb, result_cb):
        self.sent_waypoints = waypoints
        self.result_cb = result_cb
        return True

    def set_global_controller(self, mode):
        self.global_controllers.append(mode)

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

    def global_relocalize(self, wait_seconds=90.0):
        self.global_relocalize_requests.append(wait_seconds)
        return {"mode": "global_position_yaw_search", "motion_commanded": False}

    def progressive_relocalize(self, *, origin, waypoints, trusted_seed=None, wait_seconds=180.0):
        request = {"origin": origin, "waypoints": waypoints, "trusted_seed": trusted_seed, "wait_seconds": wait_seconds}
        self.progressive_relocalize_requests.append(request)
        return {
            "mode": "progressive_stationary_search",
            "selected_stage": "mapping_origin",
            "motion_commanded": False,
        }

    def quick_then_global_relocalize(self, **request):
        self.quick_then_global_requests.append(request)
        return {
            "mode": "quick_then_global",
            "selected_stage": "mapping_origin",
            "motion_commanded": False,
        }

    def teleop_action(self, action):
        self.teleop_actions.append(action)
        return {"topic": "/teleop_action", "action": action}

    def confirmed_teleop_action(self, action, success_states, failure_states=None, timeout_seconds=4.0):
        self.teleop_actions.append(action)
        state = next(iter(success_states))
        return {"topic": "/teleop_action", "action": action, "confirmed": True, "motion_state": state}

    def confirmed_remote_teleop_action(self, action, success_states, failure_states=None, timeout_seconds=4.0):
        return self.confirmed_teleop_action(action, success_states, failure_states, timeout_seconds)

    def remote_teleop_action(self, action):
        return self.teleop_action(action)

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

    def manual_assist_velocity(self, vx=0.0, vy=0.0, yaw_rate=0.0):
        payload = {"topic": "/cmd_vel_assist", "vx": vx, "vy": vy, "yaw_rate": yaw_rate}
        self.teleop_velocities.append(payload)
        return payload

    def obstacle_monitor_snapshot(self):
        return {}

    def wait_until_ready(self, timeout_seconds=45):
        self.wait_until_ready_timeout = timeout_seconds
        return True


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
    def mapping_start_pose(self):
        return {"x": 0.0, "y": 0.0, "yaw": 0.0, "source": "mapping_start"}

    def resolve_source_dir(self, command):
        return Path("/maps/source")

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
        self.start_calls = []

    def start(self, command=None):
        self.start_calls.append(command or {})
        return {"action": "start", "returncode": 0}

    def reload_map(self, pcd_path, yaml_path):
        self.reload_calls.append((pcd_path, yaml_path))
        return {"action": "reload_map", "returncode": 0}


class FakeDeferredNavigationStack(FakeNavigationStack):
    def reload_map_if_running(self, pcd_path, yaml_path):
        return {
            "action": "reload_map",
            "returncode": 0,
            "deferred": True,
            "reason": "map_consumers_inactive",
            "pcd_path": pcd_path,
            "yaml_path": yaml_path,
        }


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


def test_wait_until_localization_stable_blocks_until_window(monkeypatch):
    clock = {"t": 100.0}
    monkeypatch.setattr("roamerx_edge.safety_policy.time.monotonic", lambda: clock["t"])
    monkeypatch.setattr(
        "roamerx_edge.safety_policy.time.sleep",
        lambda dt: clock.__setitem__("t", clock["t"] + dt),
    )
    state = RuntimeSafetyState(
        localization_status="normal",
        localization_normal_since_monotonic=99.1,
    )
    SafetyPolicy(SafetyConfig(), state).wait_until_localization_stable()
    assert clock["t"] >= 102.1


def test_task_start_waits_then_accepts_fresh_normal_localization(tmp_path, monkeypatch):
    clock = {"t": 100.0}
    monkeypatch.setattr("roamerx_edge.safety_policy.time.monotonic", lambda: clock["t"])
    monkeypatch.setattr(
        "roamerx_edge.safety_policy.time.sleep",
        lambda dt: clock.__setitem__("t", clock["t"] + dt),
    )
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
        localization_normal_since_monotonic=99.1,
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

    ack, _ = processor.handle_command(raw)

    assert ack["payload"]["ack"] == "accepted"
    assert clock["t"] >= 102.1
    store.close()


def test_task_start_persists_ack_before_smart_initialization(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    command_id = raw["payload"]["command_id"]
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store,
        navigation,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    ack_seen_during_initialization = []

    def initialize():
        saved = store.get_processed_command(command_id)
        ack_seen_during_initialization.append(saved["ack"]["payload"]["ack"])

    executor.initialize_before_navigation = initialize
    state = RuntimeSafetyState(
        localization_status="normal",
        localization_normal_since_monotonic=time.monotonic() - 10.0,
        nav_ready=True,
        control_mode="autonomous",
        current_map_id="site-a-main",
        current_map_version="v1",
    )
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
    )

    ack, _ = processor.handle_command(raw)

    assert ack["payload"]["ack"] == "accepted"
    assert ack_seen_during_initialization == ["accepted"]
    store.close()


def test_task_start_forwards_localization_attempt_progress(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    store = LocalStore(str(tmp_path / "edge.db"))

    class ProgressNavigation(FakeNavigation):
        def __init__(self):
            super().__init__()
            self.attempt_progress_callback = None

        def set_attempt_progress_callback(self, callback):
            self.attempt_progress_callback = callback

    navigation = ProgressNavigation()
    executor = TaskExecutor(
        store,
        navigation,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    def initialize():
        navigation.attempt_progress_callback({
            "state": "running",
            "selected_stage": "rtk_fixed",
            "attempts": [{"candidate_number": 1, "status": "verifying"}],
        })

    executor.initialize_before_navigation = initialize
    state = RuntimeSafetyState(
        localization_status="normal",
        localization_normal_since_monotonic=time.monotonic() - 10.0,
        nav_ready=True,
        control_mode="autonomous",
        current_map_id="site-a-main",
        current_map_version="v1",
    )
    progress = []
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        publish_progress=lambda *args: progress.append(args),
        localization_adapter=navigation,
    )

    processor.handle_command(raw)

    localization_progress = next(
        payload for _, payload in progress
        if payload["payload"]["result"].get("selected_stage") == "rtk_fixed"
    )
    assert localization_progress["payload"]["result"]["attempts"][0]["candidate_number"] == 1
    startup = localization_progress["payload"]["result"]["startup_progress"]
    assert startup["phase"] == "rtk_fixed"
    assert startup["current_action"] == "智能初始化定位：验证 RTK 固定解"
    assert [item["status"] for item in startup["actions"]] == [
        "completed", "completed", "in_progress", "waiting",
    ]
    assert progress[-1][1]["payload"]["result"]["selected_stage"] == "navigation_start"
    assert progress[-1][1]["payload"]["result"]["navigation_start"]["status"] == "accepted"
    assert progress[-1][1]["payload"]["result"]["navigation_start"]["finished_at"]
    final_startup = progress[-1][1]["payload"]["result"]["startup_progress"]
    assert final_startup["status"] == "completed"
    assert final_startup["current_action"] == "首航点已下发，Nav2 开始执行"
    assert navigation.attempt_progress_callback is None
    store.close()


def test_task_start_repairs_nav_stack_only_when_not_ready(tmp_path):
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
        localization_normal_since_monotonic=time.monotonic() - 10.0,
        nav_ready=False,
        control_mode="autonomous",
        current_map_id="site-a-main",
        current_map_version="v1",
    )
    stack = FakeNavigationStack()
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        navigation_stack_adapter=stack,
    )

    ack, _ = processor.handle_command(raw)

    assert ack["payload"]["ack"] == "accepted"
    assert stack.start_calls == [{"reason": "task_start"}]
    assert navigation.wait_until_ready_timeout == 45
    assert state.nav_ready is True
    store.close()


def test_cross_map_docking_switches_map_before_final_validation(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    command = raw["payload"]["command"]
    command["map"] = {"map_id": "charge-map", "map_version": "dock-v1"}
    command["docking"] = {"enabled": True, "final_waypoint_index": 2}
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
        localization_normal_since_monotonic=time.monotonic() - 10.0,
        nav_ready=True,
        control_mode="autonomous",
        current_map_id="patrol-map",
        current_map_version="patrol-v1",
    )
    order = []

    class DockMapActivation(FakeMapActivation):
        def activate(self, map_payload):
            order.append("activate")
            state.current_map_id = map_payload["map_id"]
            state.current_map_version = map_payload["map_version"]
            return super().activate(map_payload)

    class DockNavigationStack(FakeNavigationStack):
        def switch_map(self):
            order.append("switch")
            return {"action": "switch_map", "returncode": 0}

    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        map_activation_adapter=DockMapActivation(),
        navigation_stack_adapter=DockNavigationStack(),
    )

    ack, _ = processor.handle_command(raw)

    assert ack["payload"]["ack"] == "accepted"
    assert order == ["activate", "switch"]
    assert state.current_map_id == "charge-map"
    assert executor.context.docking["enabled"] is True
    store.close()


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
    # This test verifies event-version ordering, not stationary arrival. Make
    # the final batch a pass-through route so the newer fresh-frame arrival
    # gate does not turn the test double's missing ROS samples into SAFE_HOLD.
    for waypoint in raw["payload"]["command"]["route_snapshot"]["waypoints"]:
        waypoint["arrival_policy"] = "pass_through"
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
    assert any(event[0] == "task.started" for event in events)
    assert events[0][1]["state_version"] == 3
    assert events[-1][0] == "task.completed"
    assert events[-1][1]["state_version"] > acks[0]["payload"]["edge_state_version"]
    store.close()


def test_task_start_releases_manual_takeover_before_launching_navigation(tmp_path):
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

    ack, result = processor.handle_command(raw)

    assert ack["payload"]["ack"] == "accepted"
    assert result is None
    assert state.control_mode == "autonomous"
    assert navigation.teleop_velocities[-1] == {
        "topic": "/teleop_cmd_vel",
        "vx": 0.0,
        "vy": 0.0,
        "yaw_rate": 0.0,
    }
    assert navigation.teleop_actions[-1] == "release_remote"
    store.close()


def test_task_start_clears_stale_manual_assist_before_launching_navigation(tmp_path):
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
        control_mode="manual_assist",
        current_map_id="site-a-main",
        current_map_version="v1",
    )
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
    )

    ack, result = processor.handle_command(raw)

    assert ack["payload"]["ack"] == "accepted"
    assert result is None
    assert state.control_mode == "autonomous"
    assert navigation.teleop_velocities[-1] == {
        "topic": "/cmd_vel_assist",
        "vx": 0.0,
        "vy": 0.0,
        "yaw_rate": 0.0,
    }
    assert navigation.teleop_actions == []
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


def test_nav_initial_pose_uses_origin_first_ndt_pipeline(tmp_path):
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
    assert navigation.initial_pose is None
    assert navigation.progressive_relocalize_requests[0]["waypoints"][0]["x"] == 1.0
    assert result["payload"]["status"] == "succeeded"
    assert results[0]["payload"]["result"]["mode"] == "progressive_stationary_search"
    assert navigation.operator_localization_events == ["begin", "end"]
    store.close()


def test_nav_single_goal_applies_global_controller(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.single_goal"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {
        "x": 1.0,
        "y": 2.0,
        "yaw": 0.5,
        "global_controller": "navfn",
    }
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(
            SafetyConfig(),
            RuntimeSafetyState(localization_status="normal", nav_ready=True),
        ),
        task_executor=TaskExecutor(
            store,
            navigation,
            event_callback=lambda *args: None,
            start_result_callback=lambda *args: None,
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
    )

    _, result = processor.handle_command(raw)

    assert result["payload"]["status"] == "succeeded"
    assert navigation.global_controllers == ["navfn"]
    assert navigation.sent_waypoints[0]["global_controller"] == "navfn"
    store.close()


def test_nav_initial_pose_bootstraps_cold_localization_before_starting_nav2(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.initial_pose"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"x": 1.0, "y": 2.0, "yaw": 0.5}

    class ColdNavigation(FakeNavigation):
        def __init__(self):
            super().__init__()
            self.subscriber_ready = False
            self.subscriber_waits = []

        def wait_for_initial_pose_subscriber(self, timeout_seconds=0.0):
            self.subscriber_waits.append(timeout_seconds)
            return self.subscriber_ready

    class ColdStack(FakeNavigationStack):
        def __init__(self, navigation):
            super().__init__()
            self.navigation = navigation
            self.restart_localization_calls = 0

        def restart_localization(self):
            self.restart_localization_calls += 1
            self.navigation.subscriber_ready = True
            return {"action": "restart-localization", "returncode": 0}

    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = ColdNavigation()
    stack = ColdStack(navigation)
    state = RuntimeSafetyState(localization_status="unknown", nav_ready=False)
    progress = []
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=TaskExecutor(
            store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None,
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        publish_progress=lambda *_args: progress.append(_args),
        localization_adapter=navigation,
        navigation_stack_adapter=stack,
    )

    _, result = processor.handle_command(raw)

    payload = result["payload"]["result"]
    assert result["payload"]["status"] == "succeeded"
    assert stack.restart_localization_calls == 1
    assert navigation.progressive_relocalize_requests[0]["waypoints"][0]["x"] == 1.0
    assert stack.start_calls == [{"reason": "initial_pose_bootstrap"}]
    assert payload["localization_bootstrap"]["action"] == "restart-localization"
    assert payload["navigation_start"]["action"] == "start"
    assert payload["navigation_start"]["ready"] is True
    assert progress[-1][1]["payload"]["result"]["navigation_start"]["ready"] is True
    assert state.nav_ready is True
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
    assert navigation.progressive_relocalize_requests[0]["waypoints"][0]["x"] == 1.0
    store.close()


def test_second_operator_localization_command_is_rejected_without_superseding_first(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.initial_pose"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"x": 1.0, "y": 2.0, "yaw": 0.5}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), RuntimeSafetyState()),
        task_executor=TaskExecutor(
            store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None,
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
    )

    processor._localization_command_lock.acquire()
    try:
        _, result = processor.handle_command(raw)
    finally:
        processor._localization_command_lock.release()

    assert result["payload"]["status"] == "failed"
    assert result["payload"]["error_code"] == "LOCALIZATION_COMMAND_BUSY"
    assert navigation.initial_pose is None
    assert navigation.operator_localization_events == []
    store.close()


def test_nav_initial_pose_legacy_rtk_seed_uses_ndt_progressive_pipeline(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.initial_pose"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {
        "seed_source": "rtk",
        "wait_seconds": 12.0,
        "start_navigation": True,
    }
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    stack = FakeNavigationStack()
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
        navigation_stack_adapter=stack,
    )

    _, result = processor.handle_command(raw)

    assert navigation.rtk_initial_pose_requests == 0
    assert len(navigation.progressive_relocalize_requests) == 1
    assert result["payload"]["result"]["mode"] == "progressive_stationary_search"
    assert result["payload"]["result"]["navigation_start"]["action"] == "start"
    assert stack.start_calls == [{"reason": "initial_pose_bootstrap"}]
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


def test_relocalization_secondary_correction_uses_unique_message_id(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.relocalize"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {
        "map_id": "92", "map_version": "v1", "localization_mode": "ukf",
    }
    store = LocalStore(str(tmp_path / "edge.db"))
    store.save_last_trusted_pose("92", "v1", {"x": 8.0, "y": 9.0, "yaw": -0.4})
    navigation = FakeNavigation()
    transactions = []

    def control(transaction_id, mode, command="start"):
        transactions.append((transaction_id, mode, command))
        return {"accepted": True, "transaction_id": transaction_id, "status": "waiting_source"}

    navigation.control_localization_correction = control
    navigation.localization_decision = lambda: {
        "one_shot_correction": {
            "transaction_id": transactions[0][0],
            "status": "completed",
            "selected_source": "ukf_fused",
        },
    }
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(
            SafetyConfig(), RuntimeSafetyState(current_map_id="92", current_map_version="v1")
        ),
        task_executor=TaskExecutor(
            store, navigation, event_callback=lambda *args: None,
            start_result_callback=lambda *args: None,
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
    )

    _, result = processor.handle_command(raw)

    assert result["payload"]["status"] == "succeeded"
    assert raw["message_id"] in transactions[0][0]
    assert result["payload"]["result"]["secondary_correction"]["status"] == "completed"
    assert result["payload"]["result"]["secondary_correction"]["selected_source"] == "ukf_fused"
    store.close()


def test_relocalization_rejects_unaccepted_secondary_correction(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.relocalize"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {
        "map_id": "92", "map_version": "v1", "localization_mode": "rtk",
    }
    store = LocalStore(str(tmp_path / "edge.db"))
    store.save_last_trusted_pose("92", "v1", {"x": 8.0, "y": 9.0, "yaw": -0.4})
    navigation = FakeNavigation()
    navigation.control_localization_correction = lambda *_args: {
        "accepted": False,
        "status": "rejected",
        "message": "fixed RTK unavailable",
    }
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(
            SafetyConfig(), RuntimeSafetyState(current_map_id="92", current_map_version="v1")
        ),
        task_executor=TaskExecutor(
            store, navigation, event_callback=lambda *args: None,
            start_result_callback=lambda *args: None,
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
    )

    _, result = processor.handle_command(raw)

    assert result["payload"]["status"] == "failed"
    assert result["payload"]["error_code"] == "LOCALIZATION_SECONDARY_CORRECTION_REJECTED"
    store.close()


def test_trusted_pose_relocalization_preserves_operator_search_controls(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    store.save_last_trusted_pose("92", "v1", {"x": 8.0, "y": 9.0, "yaw": -0.4})
    navigation = FakeNavigation()
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(
            SafetyConfig(), RuntimeSafetyState(current_map_id="92", current_map_version="v1")
        ),
        task_executor=TaskExecutor(
            store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None,
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
    )

    seed = processor._resolve_localization_seed({
        "seed_source": "last_trusted",
        "wait_seconds": 17.0,
        "max_attempts": 4,
        "candidate_wait_seconds": 3.0,
        "source": "operator_retry",
    })

    assert seed == {
        "x": 8.0,
        "y": 9.0,
        "yaw": -0.4,
        "wait_seconds": 17.0,
        "max_attempts": 4,
        "candidate_wait_seconds": 3.0,
        "source": "operator_retry",
    }
    store.close()


def test_active_relocalization_global_uses_quick_then_global_without_seed(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.relocalize"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"seed_source": "global", "wait_seconds": 42.0}
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

    assert result["payload"]["status"] == "succeeded"
    request = navigation.quick_then_global_requests[0]
    assert request["origin"] is None
    assert request["manual_seed"] is None
    assert request["wait_seconds"] == 42.0
    store.close()


def test_progressive_relocalization_starts_at_mapping_origin_then_receives_waypoints(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.relocalize"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {
        "seed_source": "progressive",
        "waypoints": [
            {"x": 1.0, "y": 2.0, "yaw": 0.1},
            {"x": 3.0, "y": 4.0, "yaw": 0.2},
        ],
        "wait_seconds": 150.0,
    }
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    stack = FakeNavigationStack()
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), RuntimeSafetyState()),
        task_executor=TaskExecutor(
            store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None,
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
        map_activation_adapter=FakeMapActivation(),
        navigation_stack_adapter=stack,
    )

    _, result = processor.handle_command(raw)

    assert result["payload"]["status"] == "succeeded"
    request = navigation.progressive_relocalize_requests[0]
    assert request["origin"]["source"] == "mapping_start"
    assert request["waypoints"][1]["x"] == 3.0
    assert request["wait_seconds"] == 150.0
    assert result["payload"]["result"]["selected_stage"] == "mapping_origin"
    assert result["payload"]["result"]["navigation_start"]["action"] == "start"
    assert stack.start_calls == [{"reason": "initial_pose_bootstrap"}]
    store.close()


def test_relocalization_keeps_accepted_localization_evidence_when_navigation_activation_fails(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.relocalize"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {
        "seed_source": "progressive",
        "wait_seconds": 60.0,
    }

    class FailingStack(FakeNavigationStack):
        def start(self, command=None):
            raise ProtocolError(
                "NAV_COMMAND_FAILED",
                "controller lifecycle resume failed",
                details={"action": "activate-execution", "stderr": "controller_server failed"},
            )

    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), RuntimeSafetyState()),
        task_executor=TaskExecutor(
            store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None,
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
        map_activation_adapter=FakeMapActivation(),
        navigation_stack_adapter=FailingStack(),
    )

    _, result = processor.handle_command(raw)

    payload = result["payload"]
    assert payload["status"] == "failed"
    assert payload["error_code"] == "NAV_COMMAND_FAILED"
    assert payload["result"]["localization_attempts"]["state"] == "accepted"
    assert payload["result"]["navigation_start"]["error_code"] == "NAV_COMMAND_FAILED"
    assert payload["result"]["navigation_start"]["details"]["action"] == "activate-execution"
    store.close()


def test_last_trusted_seed_never_falls_back_to_mapping_start(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None
    )
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), RuntimeSafetyState(current_map_id="149", current_map_version="v1")),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
        map_activation_adapter=FakeMapActivation(),
    )

    with pytest.raises(ProtocolError) as exc:
        processor._resolve_localization_seed({"seed_source": "last_trusted"})

    assert exc.value.code == "RELOCALIZATION_SEED_UNAVAILABLE"
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


def test_manual_assist_uses_collision_monitored_pipeline_without_blocking_task(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "teleop.takeover_enter"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"assist": True}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None)
    executor.has_active_task = lambda: True
    state = RuntimeSafetyState(localization_status="normal", nav_ready=True, control_mode="autonomous")
    released_profiles = []
    processor = CommandProcessor(
        robot_id="rx-001", store=store, safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor, publish_ack=lambda *args: None, publish_result=lambda *args: None,
        localization_adapter=navigation,
        temporary_fusion_release_callback=released_profiles.append,
    )

    _, result = processor.handle_command(raw)

    assert state.control_mode == "manual_assist"
    assert released_profiles == ["manual_control_takeover_enter"]
    assert result["payload"]["result"]["motion_topic"] == "/cmd_vel_assist"
    raw["message_type"] = "teleop.move_velocity"
    raw["payload"]["command_id"] = "11111111-1111-4111-8111-111111111111"
    raw["payload"]["command"] = {"vx": 0.4, "vy": -0.4, "yaw_rate": 0.8}
    _, result = processor.handle_command(raw)
    assert result["payload"]["result"] == {
        "topic": "/cmd_vel_assist", "vx": 0.1, "vy": -0.1, "yaw_rate": 0.25, "mode": "manual_assist",
    }
    store.close()


@pytest.mark.parametrize(
    "message_type",
    ["teleop.stand_up", "teleop.lie_down", "teleop.shake_hand", "teleop.two_leg_stand"],
)
def test_manual_assist_discrete_actions_keep_navigation_control_mode(tmp_path, message_type):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = message_type
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"assist": True}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None
    )
    executor.has_active_task = lambda: True
    state = RuntimeSafetyState(
        localization_status="normal", nav_ready=True, control_mode="manual_assist"
    )
    released_profiles = []
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
        temporary_fusion_release_callback=released_profiles.append,
    )

    _, result = processor.handle_command(raw)

    assert result["payload"]["status"] == "succeeded"
    assert result["payload"]["result"]["mode"] == "manual_assist"
    assert state.control_mode == "manual_assist"
    assert released_profiles == []
    store.close()


def test_manual_assist_action_is_rejected_after_navigation_task_finishes(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "teleop.move_velocity"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"assist": True, "vx": 0.05}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None
    )
    executor.has_active_task = lambda: False
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(
            SafetyConfig(),
            RuntimeSafetyState(
                localization_status="normal", nav_ready=True, control_mode="manual_assist"
            ),
        ),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        localization_adapter=navigation,
    )

    _, result = processor.handle_command(raw)

    assert result["payload"]["status"] == "failed"
    assert result["payload"]["error_code"] == "MANUAL_ASSIST_REQUIRES_ACTIVE_TASK"
    assert navigation.teleop_velocities == []
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


def test_map_activation_is_rejected_while_a_task_is_active(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "map.activate"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"map_id": "95", "map_version": "selected-map"}
    store = LocalStore(str(tmp_path / "edge.db"))
    executor = TaskExecutor(
        store,
        FakeNavigation(),
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.context = SimpleNamespace(state="running", state_version=0)
    activation = FakeMapActivation()
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), RuntimeSafetyState()),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        map_activation_adapter=activation,
        navigation_stack_adapter=FakeNavigationStack(),
    )

    _, result = processor.handle_command(raw)

    assert result["payload"]["status"] == "failed"
    assert result["payload"]["error_code"] == "ROBOT_BUSY"
    store.close()


def test_map_activation_succeeds_when_reload_is_deferred_after_mapping(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "map.activate"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {"map_id": "131", "map_version": "mapped-site"}
    store = LocalStore(str(tmp_path / "edge.db"))
    state = RuntimeSafetyState(localization_status="normal", nav_ready=False)
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
        navigation_stack_adapter=FakeDeferredNavigationStack(),
    )

    _, result = processor.handle_command(raw)

    assert result["payload"]["status"] == "succeeded"
    assert result["payload"]["result"]["map_reload"]["deferred"] is True
    assert state.localization_status == "initializing"
    store.close()


def test_map_activation_completes_backend_localization_before_reporting_ready(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "map.activate"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {
        "map_id": "95", "map_version": "selected-map", "scene_scope": "indoor",
    }
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    navigation.lio_readiness = lambda **_kwargs: {"ready": True, "state": "ready"}
    navigation.wait_until_prepared = lambda **_kwargs: True
    navigation.wait_for_final_localization_gate = lambda **_kwargs: {"accepted": True}
    calls = []

    class Stack(FakeNavigationStack):
        def deactivate_execution(self):
            calls.append("deactivate")
            return {"action": "deactivate_execution"}

        def prepare(self, command=None):
            calls.append(("prepare", command))
            return {"action": "prepare"}

        def activate_execution(self):
            calls.append("activate")
            return {"action": "activate_execution"}

    state = RuntimeSafetyState(localization_status="normal", nav_ready=True)
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=TaskExecutor(
            store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        map_activation_adapter=FakeMapActivation(),
        navigation_stack_adapter=Stack(),
        localization_adapter=navigation,
    )

    _, result = processor.handle_command(raw)

    payload = result["payload"]["result"]
    assert payload["localization_reset_required"] is False
    assert payload["navigation_allowed"] is True
    assert payload["localization"]["final_localization_gate"]["status"] == "accepted"
    assert calls == ["deactivate", ("prepare", {"reason": "map.activate"}), "activate"]
    store.close()


def test_map_optimize_uses_resolved_source_without_reloading_navigation(tmp_path):
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "map.optimize"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {
        "source_map_id": "131",
        "selected_candidates": [{"candidate_id": "10:50"}],
    }
    store = LocalStore(str(tmp_path / "edge.db"))

    class FakeOfflineMapping:
        def optimize_historical_map(self, command, source_dir):
            assert source_dir == Path("/maps/source")
            return {"source_map_id": command["source_map_id"], "upload_result": {"id": 212}}

    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), RuntimeSafetyState(localization_status="normal", nav_ready=True)),
        task_executor=TaskExecutor(
            store, FakeNavigation(), event_callback=lambda *args: None, start_result_callback=lambda *args: None,
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        mapping_adapter=FakeOfflineMapping(),
        map_activation_adapter=FakeMapActivation(),
        navigation_stack_adapter=FakeNavigationStack(),
    )

    _, result = processor.handle_command(raw)

    assert result["payload"]["status"] == "succeeded"
    assert result["payload"]["result"]["upload_result"]["id"] == 212
    store.close()



def test_nav_start_waits_for_in_flight_stack_command(tmp_path):
    import threading

    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = "nav.start"
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {}
    store = LocalStore(str(tmp_path / "edge.db"))

    class Stack:
        def __init__(self):
            self.started = False

        def start(self, command=None):
            self.started = True
            return {"action": "start", "returncode": 0}

    stack = Stack()
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), RuntimeSafetyState()),
        task_executor=TaskExecutor(
            store, FakeNavigation(), event_callback=lambda *args: None, start_result_callback=lambda *args: None
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        navigation_stack_adapter=stack,
    )
    processor._navigation_command_lock.acquire()

    def release_soon():
        time.sleep(0.3)
        processor._navigation_command_lock.release()

    threading.Thread(target=release_soon, daemon=True).start()
    _, result = processor.handle_command(raw)
    assert result["payload"]["status"] == "succeeded"
    assert stack.started is True
    store.close()


def _nav_command(message_type: str) -> dict:
    raw = json.loads((Path(__file__).parent / "fixtures" / "task_start.json").read_text())
    raw["message_type"] = message_type
    raw["payload"].pop("task_execution_id", None)
    raw["payload"]["command"] = {}
    return raw


def test_navigation_rosbag_stop_command_closes_requested_loop_scope(tmp_path):
    loop_session_id = "63b66a16-1947-4be7-889b-d851a5f4ba20"
    raw = _nav_command("diagnostics.nav_rosbag_stop")
    raw["payload"]["command"] = {"loop_session_id": loop_session_id}
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    executor = TaskExecutor(
        store,
        navigation,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    calls = []

    def stop_loop_rosbag(requested):
        calls.append(requested)
        return {"running": False, "loop_session_id": requested, "ignored": False}

    executor.stop_loop_rosbag = stop_loop_rosbag
    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), RuntimeSafetyState()),
        task_executor=executor,
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
    )

    ack, result = processor.handle_command(raw)

    assert ack["payload"]["ack"] == "accepted"
    assert result["payload"]["status"] == "succeeded"
    assert result["payload"]["result"]["running"] is False
    assert calls == [loop_session_id]
    store.close()


def test_nav_start_waits_until_nav2_ready(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    state = RuntimeSafetyState(nav_ready=False)

    class Stack:
        def start(self, command=None):
            return {"action": "start", "returncode": 0}

    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=TaskExecutor(
            store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        navigation_stack_adapter=Stack(),
    )

    _, result = processor.handle_command(_nav_command("nav.start"))

    assert result["payload"]["status"] == "succeeded"
    assert navigation.wait_until_ready_timeout == 45
    assert state.nav_ready is True
    store.close()


def test_nav_start_fails_when_nav2_does_not_become_ready(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    navigation.wait_until_ready = lambda timeout_seconds=45: False
    state = RuntimeSafetyState(nav_ready=False)

    class Stack:
        def start(self, command=None):
            return {"action": "start", "returncode": 0}

    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), state),
        task_executor=TaskExecutor(
            store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        navigation_stack_adapter=Stack(),
    )

    _, result = processor.handle_command(_nav_command("nav.start"))

    assert result["payload"]["status"] == "failed"
    assert result["payload"]["error_code"] == "NAV_STACK_NOT_READY"
    assert state.nav_ready is False
    store.close()


def test_nav_start_uses_prepared_ndt_first_lifecycle_when_ros_adapter_is_available(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    navigation = FakeNavigation()
    navigation.lio_readiness = lambda **_kwargs: {"ready": True, "state": "ready"}
    navigation.wait_until_prepared = lambda **_kwargs: True
    navigation.wait_for_final_localization_gate = lambda **_kwargs: {"accepted": True}
    calls = []

    class Stack:
        def prepare(self, command=None):
            calls.append(("prepare", command))
            return {"action": "prepare", "returncode": 0}

        def activate_execution(self):
            calls.append(("activate_execution",))
            return {"action": "activate_execution", "returncode": 0}

    processor = CommandProcessor(
        robot_id="rx-001",
        store=store,
        safety=SafetyPolicy(SafetyConfig(), RuntimeSafetyState(nav_ready=False)),
        task_executor=TaskExecutor(
            store, navigation, event_callback=lambda *args: None, start_result_callback=lambda *args: None
        ),
        publish_ack=lambda *args: None,
        publish_result=lambda *args: None,
        navigation_stack_adapter=Stack(),
        localization_adapter=navigation,
        map_activation_adapter=FakeMapActivation(),
    )

    _, result = processor.handle_command(_nav_command("nav.start"))

    assert result["payload"]["status"] == "succeeded"
    assert calls == [("prepare", {"reason": "nav.start"}), ("activate_execution",)]
    assert navigation.progressive_relocalize_requests[0]["origin"]["source"] == "mapping_start"
    assert result["payload"]["result"]["navigation_allowed"] is True
    store.close()
