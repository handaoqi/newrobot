import math
import time

from roamerx_edge.teleop_skill_executor import TeleopSkillExecutor


class Pose:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


class Navigation:
    def __init__(self):
        self.calls = []
        self.pose = Pose()

    def teleop_velocity(self, vx=0.0, vy=0.0, yaw_rate=0.0):
        self.calls.append((vx, vy, yaw_rate))
        return {}

    def remote_teleop_action(self, action):
        self.calls.append(("action", action))
        return {}

    def latest_pose(self):
        return self.pose

    def obstacle_monitor_snapshot(self):
        return {"front_obstacle_distance_m": None}


def test_preset_runs_and_stops():
    navigation = Navigation()
    executor = TeleopSkillExecutor(navigation)
    outcomes = []
    run = executor.start("test-command", {"preset": "micro_reverse_5s"}, outcomes.append)
    assert run["status"] == "running"
    executor.cancel("test-command")
    deadline = time.monotonic() + 1
    while not outcomes and time.monotonic() < deadline:
        time.sleep(0.01)
    assert outcomes[0]["status"] == "cancelled"
    assert navigation.calls[-1] == (0.0, 0.0, 0.0)


def test_structured_steps_dispatch_action_and_speed():
    navigation = Navigation()
    executor = TeleopSkillExecutor(navigation)
    outcomes = []
    executor.start(
        "test-steps",
        {"description": "AI 测试", "steps": [{"kind": "action", "action": "lie_down"}, {"kind": "speed", "level": "micro"}]},
        outcomes.append,
    )
    deadline = time.monotonic() + 1
    while not outcomes and time.monotonic() < deadline:
        time.sleep(0.01)
    assert outcomes[0]["status"] == "succeeded"
    assert ("action", "lie_down") in navigation.calls
    assert ("action", "speed_micro") in navigation.calls


def test_turn_requires_pose_and_does_not_time_guess():
    navigation = Navigation()
    navigation.pose = None
    executor = TeleopSkillExecutor(navigation)
    outcomes = []
    executor.start("turn", {"steps": [{"kind": "turn", "angle_rad": math.pi, "yaw_rate": 0.4}]}, outcomes.append)
    deadline = time.monotonic() + 1
    while not outcomes and time.monotonic() < deadline:
        time.sleep(0.01)
    assert outcomes[0]["status"] == "failed"
    assert outcomes[0]["error_code"] == "SKILL_POSE_UNAVAILABLE"
