import uuid

import pytest

from roamerx_dev_agent.protocol import DevTaskRequest, TaskMessageError


def test_parse_task_request():
    task_id = str(uuid.uuid4())
    task = DevTaskRequest.parse({"task_id": task_id, "prompt": "修改并测试", "workspace": "robot-main"})
    assert task.task_id == task_id
    assert task.prompt == "修改并测试"
    assert task.model == "gpt-5.6-terra"


def test_rejects_model_outside_allowlist():
    with pytest.raises(TaskMessageError, match="model is not allowed"):
        DevTaskRequest.parse({
            "task_id": str(uuid.uuid4()),
            "prompt": "修改并测试",
            "workspace": "robot-main",
            "model": "not-a-model",
        })


@pytest.mark.parametrize(
    "payload",
    [
        {},
        {"task_id": "bad", "prompt": "x", "workspace": "robot-main"},
        {"task_id": str(uuid.uuid4()), "prompt": "", "workspace": "robot-main"},
        {"task_id": str(uuid.uuid4()), "prompt": "x", "workspace": ""},
    ],
)
def test_reject_invalid_task(payload):
    with pytest.raises(TaskMessageError):
        DevTaskRequest.parse(payload)
