import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).parents[2] / "robot" / "mcp_server" / "roamerx_robot_mcp.py"
SPEC = importlib.util.spec_from_file_location("roamerx_robot_mcp_under_test", MODULE_PATH)
mcp_module = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(mcp_module)


class FakePlatformClient:
    def __init__(self):
        self.calls = []

    def command(self, robot_id, category, payload):
        self.calls.append((robot_id, category, payload))
        return {"id": "command-1", "status": "succeeded", "category": category}

    def wait_for_command(self, robot_id, command, wait_seconds):
        return {"command": command, "completed": True, "wait_seconds": wait_seconds}

    def validation_get(self, path):
        self.calls.append(("GET", path))
        if path == "validation-profiles/":
            return [{"id": "00000000-0000-0000-0000-000000000002", "name": "navigation-localization", "version": 1, "mode": "bag_replay"}]
        if path.endswith("/report/"):
            return {"id": path.split("/")[1], "verdict": "WARN", "checks": [{"rule_id": "tf.single_parent", "status": "PASS"}], "artifacts": []}
        return []

    def validation_post(self, path, payload):
        self.calls.append(("POST", path, payload))
        return {"id": "00000000-0000-0000-0000-000000000003", "state": "queued", **payload}


def test_default_robot_id_is_loaded_from_installation_environment(monkeypatch):
    monkeypatch.setenv("ROAMERX_DEFAULT_ROBOT_ID", "42")

    assert mcp_module.resolve_robot_id(None) == 42
    assert mcp_module.resolve_robot_id(7) == 7


def test_missing_default_robot_id_has_actionable_error(monkeypatch):
    monkeypatch.delenv("ROAMERX_DEFAULT_ROBOT_ID", raising=False)

    with pytest.raises(ValueError, match="ROAMERX_DEFAULT_ROBOT_ID"):
        mcp_module.resolve_robot_id(None)


def test_invalid_default_robot_id_is_rejected(monkeypatch):
    monkeypatch.setenv("ROAMERX_DEFAULT_ROBOT_ID", "not-a-number")

    with pytest.raises(ValueError, match="positive platform robot ID"):
        mcp_module.resolve_robot_id(None)


def test_platform_client_uses_default_robot_id_in_platform_route(monkeypatch):
    captured = {}

    class Response:
        def raise_for_status(self):
            return None

        def json(self):
            return {"id": "command-1"}

    def post(url, **kwargs):
        captured["url"] = url
        captured["kwargs"] = kwargs
        return Response()

    monkeypatch.setenv("ROAMERX_DEFAULT_ROBOT_ID", "42")
    monkeypatch.setattr(mcp_module.requests, "post", post)

    result = mcp_module.PlatformClient(base_url="https://platform.example", token="token").command(
        None, "direction", {"direction": "stop"}
    )

    assert result == {"id": "command-1"}
    assert captured["url"] == "https://platform.example/api/mcp/robots/42/direction/"


def test_platform_client_lists_safe_device_inventory(monkeypatch):
    captured = {}

    class Response:
        text = ""

        def raise_for_status(self):
            return None

        def json(self):
            return [
                {
                    "id": 42,
                    "code": "ZSL-1A-07",
                    "name": "巡检犬",
                    "location": "北门",
                    "area": "园区 A",
                    "connection_status": "online",
                    "status": "online",
                    "battery_level": 88,
                    "play_urls": {"sensitive": "not returned"},
                }
            ]

    def get(url, **kwargs):
        captured["url"] = url
        captured["kwargs"] = kwargs
        return Response()

    monkeypatch.setenv("ROAMERX_DEFAULT_ROBOT_ID", "42")
    monkeypatch.setattr(mcp_module.requests, "get", get)

    result = mcp_module.PlatformClient(base_url="https://platform.example", token="token").list_platform_devices()

    assert captured["url"] == "https://platform.example/api/robots/"
    assert captured["kwargs"]["headers"] == {"Authorization": "Token token"}
    assert result == {
        "devices": [{
            "id": 42,
            "code": "ZSL-1A-07",
            "name": "巡检犬",
            "location": "北门",
            "area": "园区 A",
            "connection_status": "online",
            "status": "online",
            "battery_level": 88,
        }],
        "count": 1,
        "default_robot_id": 42,
    }


def test_capability_list_contains_released_follow_lifecycle_tools():
    capabilities = mcp_module.robot_remote_control_capabilities()
    follow_group = next(group for group in capabilities["groups"] if group["name"] == "人员识别与跟随")

    assert follow_group["tools"] == [
        "robot_person_detection",
        "robot_person_detection_status",
        "robot_person_follow",
        "robot_person_follow_status",
        "robot_person_follow_stop",
    ]


def test_capability_list_contains_read_only_platform_device_inventory():
    capabilities = mcp_module.robot_remote_control_capabilities()
    devices_group = next(group for group in capabilities["groups"] if group["name"] == "平台设备")

    assert devices_group == {
        "name": "平台设备",
        "tool": "robot_list_platform_devices",
        "buttons": [{"label": "查看设备列表", "read_only": True}],
    }


def test_capability_list_contains_read_only_skill_catalog():
    capabilities = mcp_module.robot_remote_control_capabilities()
    skill_group = next(group for group in capabilities["groups"] if group["name"] == "组合动作")

    assert skill_group["tools"] == [
        "robot_skill_list", "robot_skill_run", "robot_skill_status", "robot_skill_cancel",
    ]
    assert skill_group["buttons"][0] == {"label": "列出预设组合动作", "read_only": True}


def test_person_follow_tools_dispatch_lifecycle_commands(monkeypatch):
    client = FakePlatformClient()
    monkeypatch.setattr(mcp_module, "client", client)

    started = mcp_module.robot_person_follow(1, "person-7")
    status = mcp_module.robot_person_follow_status(1)
    stopped = mcp_module.robot_person_follow_stop(1)

    assert started["completed"] is True
    assert status["completed"] is True
    assert stopped["completed"] is True
    assert [call[1] for call in client.calls] == [
        "person-follow-start", "person-follow-status", "person-follow-stop",
    ]
    assert client.calls[0][2] == {"track_id": "person-7"}


def test_person_follow_can_request_the_center_target(monkeypatch):
    client = FakePlatformClient()
    monkeypatch.setattr(mcp_module, "client", client)

    result = mcp_module.robot_person_follow(1, target="center")

    assert result["completed"] is True
    assert client.calls == [(1, "person-follow-start", {"target": "center"})]


def test_person_follow_requires_exact_track_or_center_target():
    with pytest.raises(ValueError, match="track_id"):
        mcp_module.robot_person_follow(1)
    with pytest.raises(ValueError, match="不能与 track_id 同时使用"):
        mcp_module.robot_person_follow(1, "person-7", target="center")


def test_all_control_actions_dispatch_without_area_confirmation(monkeypatch):
    client = FakePlatformClient()
    monkeypatch.setattr(mcp_module, "client", client)

    direction = mcp_module.robot_direction(1, "forward")
    speed = mcp_module.robot_speed(1, "low")
    action = mcp_module.robot_action(1, "motion_start")
    skill = mcp_module.robot_skill_run(1, preset="prone_forward_5s")
    follow = mcp_module.robot_person_follow(1, "person-7")

    assert direction["category"] == "direction"
    assert speed["category"] == "speed"
    assert action["category"] == "action"
    assert skill["completed"] is True
    assert follow["completed"] is True
    assert client.calls == [
        (1, "direction", {"direction": "forward", "command": {}}),
        (1, "speed", {"level": "low"}),
        (1, "action", {"action": "motion_start"}),
        (1, "skill", {"description": "", "preset": "prone_forward_5s"}),
        (1, "person-follow-start", {"track_id": "person-7"}),
    ]


def test_skill_list_uses_the_edge_backed_read_only_lifecycle(monkeypatch):
    client = FakePlatformClient()
    monkeypatch.setattr(mcp_module, "client", client)

    result = mcp_module.robot_skill_list(1)

    assert result["completed"] is True
    assert client.calls == [(1, "skill-list", {})]


def test_control_values_are_normalized_to_the_platform_contract(monkeypatch):
    client = FakePlatformClient()
    monkeypatch.setattr(mcp_module, "client", client)

    for direction in ("forward", "backward", "left", "right", "turn_left", "turn_right", "stop", "velocity"):
        mcp_module.robot_direction(1, direction)
    for level in ("micro", "low", "medium", "high"):
        mcp_module.robot_speed(1, level)
    for action in ("stand_up", "prone", "passive", "motion_start", "motion_stop"):
        mcp_module.robot_action(1, action)
    mcp_module.robot_action(1, "damping")
    mcp_module.robot_action(1, "阻尼")
    mcp_module.robot_direction(1, "前进")
    mcp_module.robot_speed(1, "中速")

    assert client.calls == [
        *( (1, "direction", {"direction": direction, "command": {}}) for direction in
           ("forward", "backward", "left", "right", "turn_left", "turn_right", "stop", "velocity") ),
        *( (1, "speed", {"level": level}) for level in ("micro", "low", "medium", "high") ),
        *( (1, "action", {"action": action}) for action in
           ("stand_up", "prone", "passive", "motion_start", "motion_stop", "passive", "passive") ),
        (1, "direction", {"direction": "forward", "command": {}}),
        (1, "speed", {"level": "medium"}),
    ]


def test_invalid_control_values_fail_before_a_platform_request(monkeypatch):
    client = FakePlatformClient()
    monkeypatch.setattr(mcp_module, "client", client)

    with pytest.raises(ValueError, match="不支持的 action 参数"):
        mcp_module.robot_action(1, "hover")
    with pytest.raises(ValueError, match="不支持的 direction 参数"):
        mcp_module.robot_direction(1, "fly")
    with pytest.raises(ValueError, match="需要 command_id"):
        mcp_module.robot_skill_cancel(1)

    assert client.calls == []


def test_validation_tools_only_send_platform_ids(monkeypatch):
    client = FakePlatformClient()
    monkeypatch.setattr(mcp_module, "client", client)

    result = mcp_module.validation_job_create(
        "00000000-0000-0000-0000-000000000001",
        "00000000-0000-0000-0000-000000000002",
        idempotency_key="ai-turn-7",
    )

    assert result["state"] == "queued"
    assert client.calls == [(
        "POST",
        "validation-jobs/",
        {
            "recording_id": "00000000-0000-0000-0000-000000000001",
            "profile_id": "00000000-0000-0000-0000-000000000002",
            "idempotency_key": "ai-turn-7",
        },
    )]


def test_validation_tools_reject_paths_and_non_uuid_values(monkeypatch):
    client = FakePlatformClient()
    monkeypatch.setattr(mcp_module, "client", client)

    with pytest.raises(ValueError, match="有效 UUID"):
        mcp_module.validation_job_create("/tmp/source.mcap", "profile")
    with pytest.raises(ValueError, match="有效 UUID"):
        mcp_module.validation_job_cancel("$(touch /tmp/not-allowed)")

    assert client.calls == []


def test_validation_evidence_returns_one_bounded_rule(monkeypatch):
    client = FakePlatformClient()
    monkeypatch.setattr(mcp_module, "client", client)
    job_id = "00000000-0000-0000-0000-000000000003"

    result = mcp_module.validation_evidence_get(job_id, "tf.single_parent")

    assert result["check"] == {"rule_id": "tf.single_parent", "status": "PASS"}
    assert result["artifacts"] == []
