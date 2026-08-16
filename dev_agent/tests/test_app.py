from types import SimpleNamespace

from roamerx_dev_agent import app as app_module
from roamerx_dev_agent.app import DevAgentApplication, build_effective_prompt, build_wake_task_payload


def test_new_conversation_bootstraps_project_instructions():
    prompt = build_effective_prompt(
        workspace_path="/workspace",
        prompt="检查导航",
        session_id=None,
    )

    assert "读取该目录适用的 AGENTS.md" in prompt
    assert "用户指令：\n检查导航" in prompt


def test_resumed_conversation_uses_existing_context_without_rebootstrap():
    prompt = build_effective_prompt(
        workspace_path="/workspace",
        prompt="继续修复",
        session_id="thread-123",
    )

    assert "沿用已有上下文" in prompt
    assert "不要重复读取已经加载的项目说明" in prompt
    assert "读取该目录适用的 AGENTS.md" not in prompt
    assert "用户新指令：\n继续修复" in prompt


def test_plan_prompt_forbids_changes_and_requests_confirmation():
    prompt = build_effective_prompt(
        workspace_path="/workspace",
        prompt="增加速度限制",
        session_id="thread-123",
        execution_mode="plan",
    )

    assert "严禁修改任何文件" in prompt
    assert "确认并执行计划" in prompt


def test_wake_prompt_routes_to_installed_skills_without_bypassing_safety():
    prompt = build_effective_prompt(
        workspace_path="/workspace",
        prompt="前进",
        session_id=None,
        wake_name="小太阳",
    )

    assert "自动提交" in prompt
    assert "所有适用的已安装 Skill" in prompt
    assert "不代表现场安全确认" in prompt


def test_wake_task_payload_strips_name_and_reuses_voice_uuid():
    payload = build_wake_task_payload(
        voice_id="92b9257a-cb1a-4e15-bbe8-10df7e057d00",
        transcript="小太阳，停止跟随",
        wake_name="小太阳",
        workspace="robot-main",
        conversation_id="voice",
    )

    assert payload is not None
    assert payload["task_id"] == "92b9257a-cb1a-4e15-bbe8-10df7e057d00"
    assert payload["prompt"] == "停止跟随"
    assert payload["_wake_name"] == "小太阳"


def test_non_wake_transcript_does_not_create_a_task():
    assert build_wake_task_payload(
        voice_id="not-a-uuid",
        transcript="停止跟随",
        wake_name="小太阳",
        workspace="robot-main",
        conversation_id="voice",
    ) is None


def test_local_wake_command_is_forwarded_to_cloud_for_audit_and_acknowledgement():
    agent = object.__new__(DevAgentApplication)
    agent.config = SimpleNamespace(voice=SimpleNamespace(
        wake_name="小太阳", wake_workspace="robot-main", wake_conversation_id="voice",
    ))
    published = []
    agent._topic = lambda suffix: suffix
    agent._publish = lambda topic, payload, retain=False: published.append((topic, payload, retain))

    payload = {
        "voice_id": "92b9257a-cb1a-4e15-bbe8-10df7e057d00",
        "transcript": "小太阳，停止跟随",
        "asr_engine": "nx-sensevoice",
    }
    agent._publish_voice(payload)

    assert published == [("dev/voice/audio", payload, False)]


def test_non_wake_transcript_still_forwards_audio_to_cloud():
    agent = object.__new__(DevAgentApplication)
    agent.config = SimpleNamespace(voice=SimpleNamespace(
        wake_name="小太阳", wake_workspace="robot-main", wake_conversation_id="voice",
    ))
    published = []
    agent._topic = lambda suffix: suffix
    agent._publish = lambda topic, payload, retain=False: published.append((topic, payload, retain))

    agent._publish_voice({"voice_id": "voice-1", "transcript": "检查导航"})

    assert [topic for topic, _, _ in published] == ["dev/voice/audio"]
