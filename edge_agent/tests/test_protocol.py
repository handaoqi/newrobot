import json
from pathlib import Path

import pytest

from roamerx_edge.protocol import ProtocolError, decode_message


FIXTURE = Path(__file__).parent / "fixtures" / "task_start.json"


def test_contract_task_start_fixture():
    envelope = decode_message(json.loads(FIXTURE.read_text()))
    assert envelope.message_type == "task.start"
    assert len(envelope.payload["command"]["route_snapshot"]["waypoints"]) == 3


def test_rejects_non_contiguous_waypoints():
    payload = json.loads(FIXTURE.read_text())
    payload["payload"]["command"]["route_snapshot"]["waypoints"][1]["sequence"] = 9
    with pytest.raises(ProtocolError) as exc:
        decode_message(payload)
    assert exc.value.code == "INVALID_MESSAGE"


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
