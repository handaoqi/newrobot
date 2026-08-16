import json
from pathlib import Path

from roamerx_dev_agent.codex_runner import (
    CodexRunner,
    is_known_models_cache_warning,
    is_session_integrity_warning,
)
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
        model="gpt-5.6-terra",
        on_output=lambda stream, text, parsed: outputs.append((stream, text, parsed)),
    )
    assert code == 0
    assert cancelled is False
    assert any(item[0] == "stdout" and item[2]["type"] == "message" for item in outputs)
    assert any(item[0] == "stderr" and item[1] == "warning" for item in outputs)


def test_resume_argv_uses_persisted_session(tmp_path: Path):
    runner = CodexRunner(CodexConfig(binary="/opt/codex", home=str(tmp_path), yolo=True))
    argv = runner.build_argv(
        workspace="/workspace", session_id="thread-123", model="gpt-5.6-terra",
    )
    assert argv == [
        "/opt/codex",
        "exec",
        "--model",
        "gpt-5.6-terra",
        "--json",
        "--dangerously-bypass-approvals-and-sandbox",
        "resume",
        "thread-123",
        "-",
    ]


def test_plan_argv_is_read_only_even_when_execute_mode_uses_yolo(tmp_path: Path):
    runner = CodexRunner(CodexConfig(binary="/opt/codex", home=str(tmp_path), yolo=True))
    argv = runner.build_argv(
        workspace="/workspace", session_id="thread-123", model="gpt-5.6-terra", execution_mode="plan",
    )
    assert argv.index("--sandbox") < argv.index("resume")
    assert argv[argv.index("--sandbox") + 1] == "read-only"
    assert "--dangerously-bypass-approvals-and-sandbox" not in argv


def test_agent_codex_home_isolates_sessions_but_shares_credentials(tmp_path: Path):
    shared = tmp_path / "shared"
    shared.mkdir()
    for name in ("auth.json", "config.toml", "session_index.jsonl"):
        (shared / name).write_text("{}", encoding="utf-8")
    for name in ("sessions", "shell_snapshots", "skills", "plugins", "rules"):
        (shared / name).mkdir()
    agent_home = tmp_path / "agent"
    runner = CodexRunner(CodexConfig(
        binary="/opt/codex", home=str(agent_home), shared_home=str(shared), yolo=True,
    ))

    runner.prepare_codex_home()

    assert (agent_home / "auth.json").is_symlink()
    assert (agent_home / "auth.json").resolve() == (shared / "auth.json").resolve()
    assert (agent_home / "sessions").is_dir()
    assert not (agent_home / "sessions").is_symlink()
    assert not (agent_home / "session_index.jsonl").exists()
    assert not (agent_home / "models_cache.json").exists()


def test_agent_codex_home_replaces_legacy_shared_session_links(tmp_path: Path):
    shared = tmp_path / "shared"
    shared.mkdir()
    (shared / "sessions").mkdir()
    (shared / "session_index.jsonl").write_text("desktop session", encoding="utf-8")
    agent_home = tmp_path / "agent"
    agent_home.mkdir()
    (agent_home / "sessions").symlink_to(shared / "sessions", target_is_directory=True)
    (agent_home / "session_index.jsonl").symlink_to(shared / "session_index.jsonl")
    runner = CodexRunner(CodexConfig(
        binary="/opt/codex", home=str(agent_home), shared_home=str(shared), yolo=True,
    ))

    runner.prepare_codex_home()

    assert (agent_home / "sessions").is_dir()
    assert not (agent_home / "sessions").is_symlink()
    assert not (agent_home / "session_index.jsonl").exists()


def test_invalid_agent_models_cache_is_saved_as_backup(tmp_path: Path):
    home = tmp_path / "agent"
    home.mkdir()
    (home / "models_cache.json").write_text('{"models": [{"slug": "bad"}]}', encoding="utf-8")
    runner = CodexRunner(CodexConfig(binary="/opt/codex", home=str(home), yolo=True))

    runner.prepare_codex_home()

    assert not (home / "models_cache.json").exists()
    assert len(list(home.glob("models_cache.invalid-*.json"))) == 1


def test_recognizes_non_fatal_models_cache_warning():
    assert is_known_models_cache_warning(
        "ERROR codex_models_manager::cache: failed to load models cache: "
        "missing field `base_instructions`"
    )


def test_recognizes_recoverable_session_warnings():
    assert is_session_integrity_warning(
        "ERROR codex_core::util: Custom tool call output is missing for call id: call_x"
    )
    assert is_session_integrity_warning(
        "ERROR codex_rollout::list: state db returned stale rollout path for thread abc"
    )
