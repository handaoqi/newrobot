from roamerx_dev_agent.app import build_effective_prompt


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
