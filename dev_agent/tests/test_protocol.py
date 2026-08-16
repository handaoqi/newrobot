import uuid

import pytest

from roamerx_dev_agent.protocol import DevTaskRequest, TaskMessageError, extract_wake_command


def test_extract_wake_command_requires_name_at_start_and_an_operation():
    assert extract_wake_command("小太阳，停止跟随") == "停止跟随"
    assert extract_wake_command("  小太阳：检查导航 ") == "检查导航"
    assert extract_wake_command("请小太阳停止") is None
    assert extract_wake_command("小太阳") is None


def test_parse_task_request():
    task_id = str(uuid.uuid4())
    task = DevTaskRequest.parse({"task_id": task_id, "prompt": "修改并测试", "workspace": "robot-main"})
    assert task.task_id == task_id
    assert task.prompt == "修改并测试"
    assert task.model == "gpt-5.6-terra"
    assert task.execution_mode == "execute"


def test_accepts_plan_execution_mode():
    task = DevTaskRequest.parse({
        "task_id": str(uuid.uuid4()),
        "prompt": "先分析方案",
        "workspace": "robot-main",
        "execution_mode": "plan",
    })
    assert task.execution_mode == "plan"


def test_rejects_model_outside_allowlist():
    with pytest.raises(TaskMessageError, match="model is not allowed"):
        DevTaskRequest.parse({
            "task_id": str(uuid.uuid4()),
            "prompt": "修改并测试",
            "workspace": "robot-main",
            "model": "not-a-model",
        })


def test_rejects_unknown_execution_mode():
    with pytest.raises(TaskMessageError, match="execution_mode is not allowed"):
        DevTaskRequest.parse({
            "task_id": str(uuid.uuid4()),
            "prompt": "修改并测试",
            "workspace": "robot-main",
            "execution_mode": "unsafe",
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
