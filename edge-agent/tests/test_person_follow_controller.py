import json
import time
from types import SimpleNamespace

import pytest

from roamerx_edge.config import PersonFollowConfig
from roamerx_edge.person_follow_controller import PersonFollowController
from roamerx_edge.protocol import ProtocolError


class FakeNavigation:
    def __init__(self):
        self.velocities = []
        self.obstacle = {}

    def teleop_velocity(self, vx=0.0, vy=0.0, yaw_rate=0.0):
        self.velocities.append({"vx": vx, "vy": vy, "yaw_rate": yaw_rate})
        return self.velocities[-1]

    def obstacle_monitor_snapshot(self):
        return self.obstacle


def write_detection(path, *, captured_monotonic=None, detections=None):
    path.write_text(json.dumps({
        "captured_monotonic": captured_monotonic if captured_monotonic is not None else time.monotonic(),
        "frame_width": 1280,
        "frame_height": 720,
        "detections": detections if detections is not None else [{
            "track_id": "person-7", "label": "person",
            "bbox": {"x": 520, "y": 100, "width": 180, "height": 280},
        }],
    }), encoding="utf-8")


def test_follow_start_and_operator_stop_publish_zero_velocity(tmp_path):
    snapshot = tmp_path / "person.json"
    write_detection(snapshot)
    navigation = FakeNavigation()
    controller = PersonFollowController(
        navigation,
        PersonFollowConfig(detection_snapshot_path=str(snapshot), control_interval_seconds=0.01),
    )

    started = controller.start("person-7")
    assert started["status"] == "running"
    time.sleep(0.03)
    stopped = controller.stop()

    assert stopped["status"] == "stopped"
    assert navigation.velocities[-1] == {"vx": 0.0, "vy": 0.0, "yaw_rate": 0.0}


def test_follow_rejects_stale_local_detection(tmp_path):
    snapshot = tmp_path / "person.json"
    write_detection(snapshot, captured_monotonic=time.monotonic() - 2)
    controller = PersonFollowController(
        FakeNavigation(),
        PersonFollowConfig(detection_snapshot_path=str(snapshot), detection_stale_seconds=0.2),
    )

    with pytest.raises(ProtocolError) as exc:
        controller.start("person-7")

    assert exc.value.code == "PERSON_DETECTION_STALE"


def test_follow_stops_when_front_obstacle_is_too_close(tmp_path):
    snapshot = tmp_path / "person.json"
    write_detection(snapshot)
    navigation = FakeNavigation()
    navigation.obstacle = {"front_obstacle_distance_m": 0.5}
    controller = PersonFollowController(
        navigation,
        PersonFollowConfig(detection_snapshot_path=str(snapshot), control_interval_seconds=0.01),
    )

    controller.start("person-7")
    time.sleep(0.03)

    assert controller.status()["reason"] == "PERSON_FOLLOW_OBSTACLE"
    assert navigation.velocities[-1] == {"vx": 0.0, "vy": 0.0, "yaw_rate": 0.0}
