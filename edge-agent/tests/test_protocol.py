import json
from pathlib import Path

import pytest

from roamerx_edge.protocol import ProtocolError, decode_message


FIXTURE = Path(__file__).parent / "fixtures" / "task_start.json"


def test_contract_task_start_fixture():
    envelope = decode_message(json.loads(FIXTURE.read_text()))
    assert envelope.message_type == "task.start"
    assert len(envelope.payload["command"]["route_snapshot"]["waypoints"]) == 3


def test_accepts_continuous_loop_rosbag_and_stop_command():
    loop_session_id = "63b66a16-1947-4be7-889b-d851a5f4ba20"
    payload = json.loads(FIXTURE.read_text())
    payload["payload"]["command"].update({
        "record_rosbag": True,
        "loop_execution": True,
        "loop_session_id": loop_session_id,
        "continuous_rosbag": True,
    })
    assert decode_message(payload).message_type == "task.start"

    payload["message_type"] = "diagnostics.nav_rosbag_stop"
    payload["payload"].pop("task_execution_id", None)
    payload["payload"]["command"] = {"loop_session_id": loop_session_id}
    assert decode_message(payload).message_type == "diagnostics.nav_rosbag_stop"


def test_rejects_non_contiguous_waypoints():
    payload = json.loads(FIXTURE.read_text())
    payload["payload"]["command"]["route_snapshot"]["waypoints"][1]["sequence"] = 9
    with pytest.raises(ProtocolError) as exc:
        decode_message(payload)
    assert exc.value.code == "INVALID_MESSAGE"


def test_rejects_invalid_waypoint_global_controller():
    payload = json.loads(FIXTURE.read_text())
    payload["payload"]["command"]["route_snapshot"]["waypoints"][0]["global_controller"] = "invalid"
    with pytest.raises(ProtocolError) as exc:
        decode_message(payload)
    assert exc.value.code == "INVALID_MESSAGE"


def test_accepts_smac_hybrid_and_ilqr_waypoint():
    payload = json.loads(FIXTURE.read_text())
    waypoint = payload["payload"]["command"]["route_snapshot"]["waypoints"][0]
    waypoint["global_controller"] = "smac_hybrid"
    waypoint["local_controller"] = "ilqr"
    envelope = decode_message(payload)
    decoded = envelope.payload["command"]["route_snapshot"]["waypoints"][0]
    assert decoded["global_controller"] == "smac_hybrid"
    assert decoded["local_controller"] == "ilqr"


def test_force_localization_correction_requires_boolean():
    payload = json.loads(FIXTURE.read_text())
    waypoint = payload["payload"]["command"]["route_snapshot"]["waypoints"][1]
    waypoint["force_localization_correction"] = True
    assert decode_message(payload).message_type == "task.start"
    waypoint["force_localization_correction"] = "true"
    with pytest.raises(ProtocolError) as exc:
        decode_message(payload)
    assert exc.value.code == "INVALID_MESSAGE"


def test_accepts_nav_single_goal_global_controller():
    payload = json.loads(FIXTURE.read_text())
    payload["message_type"] = "nav.single_goal"
    payload["payload"].pop("task_execution_id", None)
    payload["payload"]["command"] = {
        "x": 1.0,
        "y": 2.0,
        "yaw": 0.5,
        "global_controller": "navfn",
    }
    envelope = decode_message(payload)
    assert envelope.payload["command"]["global_controller"] == "navfn"


def test_accepts_nav_recover_command():
    payload = json.loads(FIXTURE.read_text())
    payload["message_type"] = "nav.recover"
    payload["payload"].pop("task_execution_id", None)
    payload["payload"]["command"] = {"reason": "test"}
    envelope = decode_message(payload)
    assert envelope.message_type == "nav.recover"


def test_accepts_nav_initial_pose_command():
    payload = json.loads(FIXTURE.read_text())
    payload["message_type"] = "nav.initial_pose"
    payload["payload"].pop("task_execution_id", None)
    payload["payload"]["command"] = {"x": 1.0, "y": 2.0, "yaw": 0.5, "frame_id": "map"}
    envelope = decode_message(payload)
    assert envelope.message_type == "nav.initial_pose"


def test_accepts_nav_initial_pose_with_mapping_start_seed():
    payload = json.loads(FIXTURE.read_text())
    payload["message_type"] = "nav.initial_pose"
    payload["payload"].pop("task_execution_id", None)
    payload["payload"]["command"] = {"seed_source": "mapping_start"}
    envelope = decode_message(payload)
    assert envelope.payload["command"]["seed_source"] == "mapping_start"


def test_accepts_nav_relocalize_with_mapping_start_seed():
    payload = json.loads(FIXTURE.read_text())
    payload["message_type"] = "nav.relocalize"
    payload["payload"].pop("task_execution_id", None)
    payload["payload"]["command"] = {"seed_source": "mapping_start"}
    envelope = decode_message(payload)
    assert envelope.message_type == "nav.relocalize"


def test_accepts_nav_relocalize_with_global_seed():
    payload = json.loads(FIXTURE.read_text())
    payload["message_type"] = "nav.relocalize"
    payload["payload"].pop("task_execution_id", None)
    payload["payload"]["command"] = {"seed_source": "global"}
    assert decode_message(payload).payload["command"]["seed_source"] == "global"


def test_accepts_legacy_quick_then_global_relocalize_seed():
    payload = json.loads(FIXTURE.read_text())
    payload["message_type"] = "nav.relocalize"
    payload["payload"].pop("task_execution_id", None)
    payload["payload"]["command"] = {
        "seed_source": "quick_then_global",
        "scene_scope": "indoor",
        "coordinate_mode": "local_only",
    }
    assert decode_message(payload).payload["command"]["seed_source"] == "quick_then_global"


def test_nav_relocalize_rejects_unknown_seed_source():
    payload = json.loads(FIXTURE.read_text())
    payload["message_type"] = "nav.relocalize"
    payload["payload"].pop("task_execution_id", None)
    payload["payload"]["command"] = {"seed_source": "guess"}
    with pytest.raises(ProtocolError) as exc:
        decode_message(payload)
    assert exc.value.code == "INVALID_MESSAGE"


def test_nav_relocalize_rejects_partial_pose():
    payload = json.loads(FIXTURE.read_text())
    payload["message_type"] = "nav.relocalize"
    payload["payload"].pop("task_execution_id", None)
    payload["payload"]["command"] = {"x": 1.0, "y": 2.0}
    with pytest.raises(ProtocolError) as exc:
        decode_message(payload)
    assert exc.value.code == "INVALID_MESSAGE"


def test_accepts_person_follow_commands():
    payload = json.loads(FIXTURE.read_text())
    payload["message_type"] = "teleop.person_follow_start"
    payload["payload"].pop("task_execution_id", None)
    payload["payload"]["command"] = {"track_id": "person-7"}
    assert decode_message(payload).message_type == "teleop.person_follow_start"


def test_accepts_skill_list_command():
    payload = json.loads(FIXTURE.read_text())
    payload["message_type"] = "teleop.skill_list"
    payload["payload"].pop("task_execution_id", None)
    payload["payload"]["command"] = {}
    assert decode_message(payload).message_type == "teleop.skill_list"
