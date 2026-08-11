import json
from pathlib import Path

from roamerx_dev_agent.codex_runner import CodexRunner
from roamerx_dev_agent.config import CodexConfig


def test_streams_json_and_stderr(tmp_path: Path):
    fake = tmp_path / "codex"
    fake.write_text(
        "#!/bin/sh\n"
        "read prompt\n"
        "printf '%s\\n' '{\"type\":\"message\",\"text\":\"ok\"}'\n"
        "printf '%s\\n' 'warning' >&2\n",
        encoding="utf-8",
    )
    fake.chmod(0o755)
    outputs = []
    runner = CodexRunner(CodexConfig(binary=str(fake), home=str(tmp_path), yolo=True, timeout_seconds=30))
    code, _, cancelled = runner.run(
        task_id="task",
        prompt="hello",
        workspace=str(tmp_path),
        session_id=None,
        on_output=lambda stream, text, parsed: outputs.append((stream, text, parsed)),
    )
    assert code == 0
    assert cancelled is False
    assert any(item[0] == "stdout" and item[2]["type"] == "message" for item in outputs)
    assert any(item[0] == "stderr" and item[1] == "warning" for item in outputs)


def test_resume_argv_uses_persisted_session(tmp_path: Path):
    runner = CodexRunner(CodexConfig(binary="/opt/codex", home=str(tmp_path), yolo=True))
    argv = runner.build_argv(workspace="/workspace", session_id="thread-123")
    assert argv == [
        "/opt/codex",
        "exec",
        "resume",
        "--json",
        "--dangerously-bypass-approvals-and-sandbox",
        "thread-123",
        "-",
    ]
