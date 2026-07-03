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
