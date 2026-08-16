from types import SimpleNamespace

from roamerx_edge.local_store import LocalStore
from roamerx_edge.protocol import decode_message
from roamerx_edge.task_executor import TaskExecutor


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

    def set_localization_policy(self, source, phase):
        self.localization_policies.append((source, phase))

    def set_waypoint_profile(self, *, avoid_obstacles, require_yaw):
        self.waypoint_profiles.append((avoid_obstacles, require_yaw))

    def set_docking_profile(self, *, final_approach):
        pass

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
            "bag_dir": f"/home/robot/rosbags/navigation/{label}",
            "started_at_unix": 100,
            "size_bytes": 0,
        }

    def stop(self):
        self.stopped += 1
        return {
            "running": False,
            "bag_dir": "/home/robot/rosbags/navigation/test",
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
    assert len(nav.sent[-1]) == 1
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
    assert [waypoint["waypoint_id"] for waypoint in nav.sent[0]] == ["wp-2"]
    assert events[0][0] == "task.started"
    assert events[0][1]["initial_waypoint_index"] == 1
    assert events[1][0] == "task.progress"
    assert events[1][1]["completed_waypoints"] == 1
    store.close()


def test_waypoint_profile_uses_previous_point_for_next_segment(tmp_path):
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
    assert nav.waypoint_profiles[0] == (True, False)
    nav.result("succeeded", "", {"missed_waypoints": []})
    assert nav.waypoint_profiles[-1] == (False, True)
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
    assert [waypoint["waypoint_id"] for waypoint in nav.sent[0]] == ["wp-2"]
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
    assert len(nav.sent[-1]) == 1
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
    for x, y in ((1.0, 2.0), (2.0, 3.0), (3.0, 4.0)):
        nav.pose = SimpleNamespace(x=x, y=y)
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
    for x, y in ((1.0, 2.0), (2.0, 3.0), (3.0, 4.0)):
        nav.pose = SimpleNamespace(x=x, y=y)
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
    nav.result("succeeded", "", {"missed_waypoints": []})
    nav.result("succeeded", "", {"missed_waypoints": []})
    nav.result("succeeded", "", {"missed_waypoints": []})
    assert executor.context.state == "failed"
    assert results[0][1] == "failed"
    assert results[0][3] == "FINAL_POSE_OUT_OF_TOLERANCE"
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
