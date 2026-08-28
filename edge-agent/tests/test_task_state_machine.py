from types import SimpleNamespace
from math import atan2, hypot
import hashlib
import json
import time

from roamerx_edge.local_store import LocalStore
from roamerx_edge.protocol import decode_message
from roamerx_edge.task_executor import TaskExecutor, straighten_pass_through_waypoints


class FakeNavigation:
    def __init__(self):
        self.feedback = None
        self.result = None
        self.sent = []
        self.cancelled = 0
        self.stop_commands = 0
        self.stopped = True
        self.pose = SimpleNamespace(x=1.0, y=2.0)
        self.trusted_pose = {
            "x": 3.0,
            "y": 4.0,
            "z": 0.0,
            "yaw": 0.5,
            "sampled_at": "2026-08-10T12:00:00+08:00",
            "frame_id": "map",
        }
        self.stand_confirmed = True
        self.stand_requests = 0
        self.localization_policies = []
        self.waypoint_profiles = []
        self.goal_precisions = []
        self.docking_profiles = []
        self.live_profiles = []

    def prepare_for_navigation(self, timeout_seconds=12):
        self.stand_requests += 1
        return self.stand_confirmed

    def send_waypoints(self, waypoints, feedback_cb, result_cb):
        self.sent.append(waypoints)
        self.feedback = feedback_cb
        self.result = result_cb
        return True

    def cancel_navigation(self, timeout_seconds=5):
        self.cancelled += 1
        return True

    def stop_motion(self):
        self.stop_commands += 1

    def is_robot_stopped(self):
        return self.stopped

    def latest_pose(self):
        return self.pose

    def latest_trusted_pose(self):
        return dict(self.trusted_pose)

    def localization_diagnostics(self):
        return {
            "raw_pose": {"x": 5.0, "y": 6.0, "yaw": 0.8, "frame_id": "map"},
            "quality": {"matching_error": 0.75, "has_converged": False},
            "decision": {"active_source": "unavailable"},
        }

    def set_localization_policy(self, source, phase):
        self.localization_policies.append((source, phase))

    def set_waypoint_profile(self, *, avoid_obstacles, require_yaw, final_approach=False, live=False):
        self.waypoint_profiles.append((avoid_obstacles, require_yaw, final_approach))
        self.live_profiles.append(live)

    def set_docking_profile(self, *, final_approach):
        self.docking_profiles.append(final_approach)

    def set_goal_precision(self, *, enabled):
        self.goal_precisions.append(enabled)

    def localization_decision(self):
        return {"active_source": "ndt_imu", "absolute_stable": True}


class FakeBlockedNavigation(FakeNavigation):
    def obstacle_monitor_snapshot(self):
        return {
            "requested_forward_speed_mps": 0.2,
            "actual_forward_speed_mps": 0.0,
            "localized_speed_mps": 0.0,
            "front_obstacle_distance_m": 0.45,
        }


class FakeCollisionLimitedNavigation(FakeNavigation):
    def obstacle_monitor_snapshot(self):
        return {
            "requested_planar_speed_mps": 0.2,
            "actual_planar_speed_mps": 0.04,
            "requested_turn_speed_rps": 0.0,
            "actual_turn_speed_rps": 0.0,
            "front_obstacle_distance_m": None,
        }


class FakeRosbagRecorder:
    def __init__(self):
        self.started = []
        self.stopped = 0

    def start(self, label):
        self.started.append(label)
        return {
            "running": True,
            "bag_dir": f"/home/dogrobot/runtime/nx-edge/data/rosbags/navigation/{label}",
            "started_at_unix": 100,
            "size_bytes": 0,
        }

    def stop(self):
        self.stopped += 1
        return {
            "running": False,
            "bag_dir": "/home/dogrobot/runtime/nx-edge/data/rosbags/navigation/test",
            "duration_seconds": 12,
            "size_bytes": 1024,
        }


class FakeMapSetCoordinator:
    def __init__(self):
        self.activated = []

    def build_segments(self, route):
        return [
            SimpleNamespace(start_index=0, end_index=1, waypoints=route["waypoints"][:1]),
            SimpleNamespace(start_index=1, end_index=3, waypoints=route["waypoints"][1:]),
        ]

    def activate(self, segment):
        self.activated.append(segment)

def ids(batch):
    return [waypoint["waypoint_id"] for waypoint in batch]


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
    assert nav.stand_requests == 1
    nav.feedback(1, None)
    paused = executor.pause_task(executor.context.task_execution_id)
    assert paused["final_task_state"] == "paused"
    assert nav.cancelled == 1
    resumed = executor.resume_task(executor.context.task_execution_id, 1)
    assert resumed["final_task_state"] == "running"
    assert ids(nav.sent[-1]) == ["wp-2", "wp-3"]
    cancelled = executor.cancel_task(executor.context.task_execution_id)
    assert cancelled["final_task_state"] == "cancelled"
    store.close()


def test_task_starts_from_nearest_waypoint_and_reports_earlier_points_complete(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=2.1, y=3.1)
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )

    executor.start_task(command("task.start"))

    assert executor.context.current_waypoint_index == 1
    assert ids(nav.sent[0]) == ["wp-2", "wp-3"]
    assert events[0][0] == "task.started"
    assert events[0][1]["initial_waypoint_index"] == 1
    assert events[1][0] == "task.progress"
    assert events[1][1]["completed_waypoints"] == 1
    store.close()


def test_round_trip_starts_from_first_copy_when_start_and_end_overlap(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.12, y=2.04)
    envelope = command("task.start")
    waypoints = envelope.payload["command"]["route_snapshot"]["waypoints"]
    waypoints.extend(
        [
            {**waypoints[1], "waypoint_id": "wp-4", "sequence": 3, "name": "B-return"},
            {
                "waypoint_id": "wp-5",
                "sequence": 4,
                "name": "A-return",
                "x": 1.15,
                "y": 2.05,
                "yaw": 3.14,
                "dwell_seconds": 0,
                "actions": [],
            },
        ]
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)

    assert executor.context.current_waypoint_index == 0
    assert ids(nav.sent[0]) == ["wp-1", "wp-2", "wp-3"]
    store.close()


def test_round_trip_starts_from_first_when_end_click_is_more_than_one_meter_off(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.32, y=1.32)
    envelope = command("task.start")
    waypoints = envelope.payload["command"]["route_snapshot"]["waypoints"]
    waypoints[:] = [
        {"waypoint_id": "wp-1", "sequence": 0, "name": "点1", "x": 1.5705, "y": 0.0698, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-2", "sequence": 1, "name": "点2", "x": 6.1205, "y": 0.0198, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-3", "sequence": 2, "name": "点3", "x": 10.7705, "y": -0.1302, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-4", "sequence": 3, "name": "点4", "x": 10.7705, "y": 1.3198, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-5", "sequence": 4, "name": "点5", "x": 5.9705, "y": 1.1698, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-6", "sequence": 5, "name": "点6", "x": 1.3205, "y": 1.3198, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
    ]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)

    assert executor.context.current_waypoint_index == 0
    assert ids(nav.sent[0]) == ["wp-1", "wp-2", "wp-3"]
    store.close()


def test_round_trip_resume_does_not_skip_outbound_legs_to_return_copy(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.12, y=2.04)
    envelope = command("task.start")
    waypoints = envelope.payload["command"]["route_snapshot"]["waypoints"]
    waypoints.extend(
        [
            {**waypoints[1], "waypoint_id": "wp-4", "sequence": 3, "name": "B-return"},
            {
                "waypoint_id": "wp-5",
                "sequence": 4,
                "name": "A-return",
                "x": 1.15,
                "y": 2.05,
                "yaw": 3.14,
                "dwell_seconds": 0,
                "actions": [],
            },
        ]
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)
    assert ids(nav.sent[0]) == ["wp-1", "wp-2", "wp-3"]
    nav.feedback(1, None)
    assert executor.context.current_waypoint_index == 1

    # Robot is nearer the return copy of B (wp-4) than the outbound B (wp-2).
    nav.pose = SimpleNamespace(x=2.05, y=3.02)
    executor.on_localization_lost()
    executor.on_localization_recovered()

    assert executor.context.current_waypoint_index == 1
    assert ids(nav.sent[-1]) == ["wp-2", "wp-3"]
    store.close()


def test_round_trip_keeps_running_after_outbound_through_poses_succeed(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.12, y=2.04)
    envelope = command("task.start")
    waypoints = envelope.payload["command"]["route_snapshot"]["waypoints"]
    waypoints.extend(
        [
            {**waypoints[1], "waypoint_id": "wp-4", "sequence": 3, "name": "B-return"},
            {
                "waypoint_id": "wp-5",
                "sequence": 4,
                "name": "A-return",
                "x": 1.15,
                "y": 2.05,
                "yaw": 3.14,
                "dwell_seconds": 0,
                "actions": [],
            },
        ]
    )
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)
    assert ids(nav.sent[0]) == ["wp-1", "wp-2", "wp-3"]

    nav.pose = SimpleNamespace(x=3.0, y=4.0)
    nav.result("succeeded", "", {"missed_waypoints": []})

    assert executor.context.state == "running"
    assert "task.completed" not in [event[0] for event in events]
    assert ids(nav.sent[-1]) == ["wp-4", "wp-5"]
    store.close()


def test_loop_execution_reverses_when_uniquely_at_route_end(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=3.0, y=4.0)
    envelope = command("task.start")
    envelope.payload["command"]["loop_execution"] = True
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)

    assert executor.context.route_snapshot["execution_order"] == "reverse_from_route_end"
    assert [waypoint["waypoint_id"] for waypoint in executor.context.route_snapshot["waypoints"]] == [
        "wp-3",
        "wp-2",
        "wp-1",
    ]
    assert executor.context.current_waypoint_index == 0
    assert ids(nav.sent[0]) == ["wp-3", "wp-2", "wp-1"]
    store.close()


def test_reverse_execution_reports_each_actual_waypoint_identity(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=3.0, y=4.0)
    envelope = command("task.start")
    envelope.payload["command"]["loop_execution"] = True
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)
    nav.feedback(1, None)
    nav.feedback(2, None)
    nav.pose = SimpleNamespace(x=1.0, y=2.0)
    nav.result("succeeded", "", {"missed_waypoints": []})

    milestones = [
        (event[1]["milestone"], event[1]["waypoint"]["waypoint_id"])
        for event in events
        if event[0] == "task.progress" and event[1].get("milestone")
    ]
    assert milestones == [
        ("target_dispatched", "wp-3"),
        ("waypoint_reached", "wp-3"),
        ("target_dispatched", "wp-2"),
        ("waypoint_reached", "wp-2"),
        ("target_dispatched", "wp-1"),
        ("waypoint_reached", "wp-1"),
    ]
    assert executor.context.state == "completed"
    store.close()


def test_waypoint_speech_blocks_next_navigation_until_playback_finishes(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    first = envelope.payload["command"]["route_snapshot"]["waypoints"][0]
    first["speech_template_id"] = 7
    status_dir = tmp_path / "audio-status"
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        waypoint_speech=SimpleNamespace(
            status_dir=str(status_dir),
            timeout_seconds=2.0,
            poll_interval_seconds=0.01,
        ),
    )

    executor.start_task(envelope)
    assert ids(nav.sent[0]) == ["wp-1"]
    nav.pose = SimpleNamespace(x=float(first["x"]), y=float(first["y"]))
    nav.result("succeeded", "", {"missed_waypoints": []})
    assert len(nav.sent) == 1

    waypoint_key = hashlib.sha256(b"wp-1").hexdigest()
    status_path = status_dir / executor.context.task_execution_id / f"{waypoint_key}.json"
    status_path.write_text(
        json.dumps({"status": "finished", "waypoint_id": "wp-1"}),
        encoding="utf-8",
    )
    for _ in range(100):
        if len(nav.sent) == 2:
            break
        time.sleep(0.01)
    executor._speech_wait_thread.join(timeout=1)
    assert ids(nav.sent[1]) == ["wp-2", "wp-3"]
    store.close()


def test_waypoint_profile_uses_target_for_initial_approach_and_source_afterwards(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    envelope = command("task.start")
    points = envelope.payload["command"]["route_snapshot"]["waypoints"]
    points[0]["avoidance_to_next"] = False
    points[1]["require_yaw"] = True
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )

    executor.start_task(envelope)
    assert ids(nav.sent[0]) == ["wp-1", "wp-2"]
    assert nav.waypoint_profiles[0] == (False, False, False)
    nav.result("succeeded", "", {"missed_waypoints": []})
    assert ids(nav.sent[-1]) == ["wp-3"]
    assert nav.waypoint_profiles[-1] == (True, False, True)
    store.close()


def test_docking_final_waypoint_requires_precise_position_and_heading(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.0, y=2.0, yaw=0.0)
    envelope = command("task.start")
    envelope.payload["command"]["docking"] = {"enabled": True, "final_waypoint_index": 2}
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
        docking_goal_tolerance_m=0.08,
        docking_goal_yaw_tolerance_rad=0.0872665,
    )

    executor.start_task(envelope)
    nav.result("succeeded", "", {"missed_waypoints": []})
    nav.result("succeeded", "", {"missed_waypoints": []})
    assert nav.goal_precisions[-1] is True
    assert nav.waypoint_profiles[-1] == (False, True, True)

    final = envelope.payload["command"]["route_snapshot"]["waypoints"][-1]
    nav.pose = SimpleNamespace(x=final["x"] + 0.03, y=final["y"] - 0.02, yaw=final["yaw"] + 0.04)
    nav.result("succeeded", "", {"missed_waypoints": []})
    assert results[-1][1] == "succeeded"
    store.close()


def test_map_set_task_skips_segments_before_nearest_waypoint(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=2.1, y=3.1)
    coordinator = FakeMapSetCoordinator()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
        map_set_coordinator=coordinator,
    )

    executor.start_task(command("task.start"))

    assert coordinator.activated == [coordinator.build_segments(executor.context.route_snapshot)[1]]
    assert executor.context.current_segment_index == 1
    assert ids(nav.sent[0]) == ["wp-2", "wp-3"]
    store.close()


def test_localization_loss_pauses_active_navigation(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    events = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))

    executor.on_localization_lost()

    assert executor.context.state == "paused"
    assert nav.cancelled == 1
    assert nav.stop_commands == 1
    assert [event[0] for event in events[-2:]] == ["task.pausing", "task.paused"]
    assert events[-2][1]["last_trusted_pose"] == nav.trusted_pose
    assert events[-2][1]["raw_pose"]["x"] == 5.0
    assert events[-2][1]["localization_quality"]["matching_error"] == 0.75
    assert events[-2][1]["current_waypoint"]["map_point_number"] == 1
    assert events[-2][1]["map_id"] == "site-a-main"
    assert "last_trusted_pose" not in events[-1][1]
    assert events[-1][1]["reason_code"] == "LOCALIZATION_LOST"
    store.close()


def test_localization_loss_cancel_timeout_stays_paused_and_auto_resumes(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.cancel_navigation = lambda timeout_seconds=5: False
    events = []
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(command("task.start"))
    nav.feedback(1, None)

    executor.on_localization_lost()
    assert executor.context.state == "paused"
    assert results == []
    assert nav.stop_commands == 1

    executor.on_localization_recovered()
    assert executor.context.state == "running"
    assert ids(nav.sent[-1]) == ["wp-2", "wp-3"]
    recovery_event = next(event for event in reversed(events) if event[0] == "task.resuming")
    assert recovery_event[1]["reason_code"] == "LOCALIZATION_RECOVERED"
    store.close()


def test_navigation_does_not_start_or_announce_obstacle_when_standup_fails(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeBlockedNavigation()
    nav.stand_confirmed = False
    events = []
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
        obstacle_speech=SimpleNamespace(
            enabled=True,
            obstacle_clear_seconds=3.0,
            no_progress_seconds=0.0,
            min_progress_m=0.5,
            obstacle_max_distance_m=0.9,
            collision_limit_ratio=0.6,
        ),
    )

    executor.start_task(command("task.start"))

    assert executor.context.state == "failed"
    assert nav.stand_requests == 1
    assert nav.sent == []
    assert not [event for event in events if event[0] == "task.obstacle_speech"]
    assert results[0][3] == "ROBOT_STANDUP_FAILED"
    executor.stop()
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
    assert ids(nav.sent[0]) == ["wp-1", "wp-2", "wp-3"]
    nav.pose = SimpleNamespace(x=3.0, y=4.0)
    nav.result("succeeded", "")
    assert executor.context.state == "completed"
    assert results[0][1] == "succeeded"
    store.close()


def test_navigation_rosbag_follows_task_lifecycle(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    recorder = FakeRosbagRecorder()
    results = []
    envelope = command("task.start")
    envelope.payload["command"]["record_rosbag"] = True
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
        rosbag_recorder=recorder,
    )

    executor.start_task(envelope)
    assert recorder.started == [f"task_{executor.context.task_execution_id[:8]}"]
    assert executor.context.record_rosbag is True
    nav.pose = SimpleNamespace(x=3.0, y=4.0)
    nav.result("succeeded", "", {"missed_waypoints": []})

    assert recorder.stopped == 1
    assert results[0][2]["rosbag"]["size_bytes"] == 1024
    store.close()


def test_navigation_missed_waypoints_fails_task(tmp_path):
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
    nav.result("succeeded", "", {"missed_waypoints": [2]})
    assert executor.context.state == "failed"
    assert results[0][1] == "failed"
    assert results[0][3] == "NAVIGATION_MISSED_WAYPOINTS"
    store.close()


def test_pass_through_waypoints_use_travel_heading(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.0, y=2.0)
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    sent = nav.sent[0]
    assert ids(sent) == ["wp-1", "wp-2", "wp-3"]
    assert abs(sent[0]["yaw"] - atan2(1.0, 1.0)) < 1e-6
    assert abs(sent[1]["yaw"] - atan2(1.0, 1.0)) < 1e-6
    assert abs(sent[2]["yaw"] - atan2(1.0, 1.0)) < 1e-6
    store.close()


def test_straighten_pass_through_flattens_click_noise_but_keeps_real_turns():
    noisy_line = [
        {"waypoint_id": "a", "x": 0.0, "y": 0.0, "yaw": 0.0},
        {"waypoint_id": "b", "x": 10.0, "y": 0.25, "yaw": 0.0},
        {"waypoint_id": "c", "x": 20.0, "y": -0.18, "yaw": 0.0},
        {"waypoint_id": "d", "x": 30.0, "y": 0.05, "yaw": 0.0},
    ]
    straight = straighten_pass_through_waypoints(noisy_line)
    assert straight[0]["x"] == 0.0 and straight[0]["y"] == 0.0
    assert straight[-1]["x"] == 30.0
    assert hypot(straight[1]["x"] - 10.0, straight[1]["y"]) < 0.05
    assert hypot(straight[2]["x"] - 20.0, straight[2]["y"]) < 0.05
    assert hypot(straight[1]["x"] - noisy_line[1]["x"], straight[1]["y"] - noisy_line[1]["y"]) > 0.15

    corner = [
        {"waypoint_id": "a", "x": 0.0, "y": 0.0, "yaw": 0.0},
        {"waypoint_id": "b", "x": 10.0, "y": 0.0, "yaw": 0.0},
        {"waypoint_id": "c", "x": 10.0, "y": 10.0, "yaw": 0.0},
    ]
    kept = straighten_pass_through_waypoints(corner)
    assert kept[1]["x"] == 10.0
    assert kept[1]["y"] == 0.0


def test_patrol_nav2_goal_uses_straightened_corridor(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=0.0, y=0.0)
    envelope = command("task.start")
    envelope.payload["command"]["route_snapshot"]["waypoints"] = [
        {"waypoint_id": "wp-1", "sequence": 0, "name": "A", "x": 0.0, "y": 0.0, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-2", "sequence": 1, "name": "B", "x": 10.0, "y": 0.25, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-3", "sequence": 2, "name": "C", "x": 20.0, "y": -0.18, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
        {"waypoint_id": "wp-4", "sequence": 3, "name": "D", "x": 30.0, "y": 0.0, "yaw": 0.0, "dwell_seconds": 0, "actions": []},
    ]
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(envelope)
    sent = nav.sent[0]
    assert ids(sent) == ["wp-1", "wp-2", "wp-3", "wp-4"]
    assert sent[0]["y"] == 0.0
    assert sent[-1]["y"] == 0.0
    assert abs(sent[1]["y"]) < 0.05
    assert abs(sent[2]["y"]) < 0.05
    assert abs(sent[0]["yaw"]) < 1e-6
    store.close()


def test_navigation_success_requires_final_pose_near_last_waypoint(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    nav.pose = SimpleNamespace(x=1.0, y=2.0)
    results = []
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: results.append(args),
    )
    executor.start_task(command("task.start"))
    assert ids(nav.sent[0]) == ["wp-1", "wp-2", "wp-3"]
    nav.feedback(2, None)
    assert nav.waypoint_profiles[-1] == (True, False, True)
    assert nav.live_profiles[-1] is True
    assert nav.goal_precisions == []
    nav.result("succeeded", "", {"missed_waypoints": []})
    assert nav.stop_commands >= 1
    assert nav.goal_precisions == []
    assert nav.docking_profiles == []
    assert nav.stop_commands >= 1
    assert executor.context.state == "failed"
    assert results[0][1] == "failed"
    assert results[0][3] == "FINAL_POSE_OUT_OF_TOLERANCE"
    store.close()


def test_patrol_final_pose_uses_045_meter_postcheck_tolerance(tmp_path):
    for distance, expected_state in ((0.42, "completed"), (0.46, "failed")):
        store = LocalStore(str(tmp_path / f"edge-{distance}.db"))
        nav = FakeNavigation()
        executor = TaskExecutor(
            store,
            nav,
            event_callback=lambda *args: None,
            start_result_callback=lambda *args: None,
        )
        executor.start_task(command("task.start"))
        final = executor.context.route_snapshot["waypoints"][-1]
        nav.pose = SimpleNamespace(x=float(final["x"]) + distance, y=float(final["y"]))
        nav.result("succeeded", "", {"missed_waypoints": []})
        assert executor.context.state == expected_state
        store.close()


def test_patrol_dispatches_remaining_waypoints_in_one_goal(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeNavigation()
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    assert ids(nav.sent[0]) == ["wp-1", "wp-2", "wp-3"]
    store.close()


def test_restart_reports_interrupted_task_and_closes_start_command(tmp_path):
    db_path = str(tmp_path / "edge.db")
    store = LocalStore(db_path)
    executor = TaskExecutor(
        store,
        FakeNavigation(),
        event_callback=lambda *args: None,
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    store.close()

    recovered_store = LocalStore(db_path)
    events = []
    results = []
    recovered = TaskExecutor(
        recovered_store,
        FakeNavigation(),
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: results.append(args),
    )
    recovered.report_startup_interruption()

    assert recovered.context.state == "failed"
    assert events[0][0] == "task.failed"
    assert results[0][1] == "failed"
    assert results[0][2]["final_task_state"] == "failed"
    assert results[0][3] == "EDGE_RESTARTED"
    recovered_store.close()


def test_obstacle_speech_escalates_after_three_no_progress_recovery_attempts(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    nav = FakeBlockedNavigation()
    events = []
    speech = SimpleNamespace(
        enabled=True,
        no_progress_seconds=0.0,
        min_progress_m=0.08,
        obstacle_max_distance_m=0.9,
    )
    executor = TaskExecutor(
        store,
        nav,
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
        obstacle_speech=speech,
    )
    executor.start_task(command("task.start"))
    executor._evaluate_obstacle_progress()  # first front obstacle detection
    executor._evaluate_obstacle_progress()  # recovery attempt 1
    executor._evaluate_obstacle_progress()  # recovery attempt 2
    executor._evaluate_obstacle_progress()  # recovery attempt 3 + leave-route
    speech_events = [event for event in events if event[0] == "task.obstacle_speech"]
    assert [event[1]["template_name"] for event in speech_events] == [
        "发现障碍物",
        "后退尝试避障",
        "后退尝试避障",
        "后退尝试避障",
        "劝阻离开线路",
    ]
    assert [event[1]["recovery_attempt"] for event in speech_events] == [0, 1, 2, 3, 3]
    executor.stop()
    store.close()


def test_collision_monitor_speed_reduction_triggers_first_obstacle_speech(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    events = []
    executor = TaskExecutor(
        store,
        FakeCollisionLimitedNavigation(),
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
        obstacle_speech=SimpleNamespace(
            enabled=True,
            no_progress_seconds=4.0,
            min_progress_m=0.08,
            obstacle_max_distance_m=0.9,
            collision_limit_ratio=0.6,
        ),
    )
    executor.start_task(command("task.start"))
    executor._evaluate_obstacle_progress()
    speech_events = [event for event in events if event[0] == "task.obstacle_speech"]
    assert len(speech_events) == 1
    assert speech_events[0][1]["template_name"] == "发现障碍物"
    executor.stop()
    store.close()


def test_report_docking_charge_emits_without_name_error(tmp_path):
    store = LocalStore(str(tmp_path / "edge.db"))
    events = []
    executor = TaskExecutor(
        store,
        FakeNavigation(),
        event_callback=lambda *args: events.append(args),
        start_result_callback=lambda *args: None,
    )
    executor.start_task(command("task.start"))
    executor.context.docking = {"enabled": True, "final_waypoint_index": 1}

    executor.report_docking_charge(
        "task.docking_contact_checking", message="已到充电桩，正在检查蓝牙与极片"
    )

    docking_events = [event for event in events if event[0] == "task.docking_contact_checking"]
    assert len(docking_events) == 1
    executor.stop()
    store.close()
