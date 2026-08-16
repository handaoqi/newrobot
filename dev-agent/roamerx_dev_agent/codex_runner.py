from __future__ import annotations

import json
import logging
import os
import selectors
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable

from .config import CodexConfig

LOGGER = logging.getLogger(__name__)

_SHARED_HOME_ENTRIES = (
    "auth.json",
    "config.toml",
    "shell_snapshots",
    "skills",
    "plugins",
    "rules",
)
_MODELS_CACHE_WARNING = "codex_models_manager::cache: failed to load models cache:"
_MISSING_TOOL_OUTPUT_WARNING = "Custom tool call output is missing for call id:"
_STALE_ROLLOUT_WARNING = "state db returned stale rollout path"


def is_known_models_cache_warning(text: str) -> bool:
    return _MODELS_CACHE_WARNING in text and "base_instructions" in text


def is_session_integrity_warning(text: str) -> bool:
    return _MISSING_TOOL_OUTPUT_WARNING in text or _STALE_ROLLOUT_WARNING in text


def is_missing_tool_output_warning(text: str) -> bool:
    return _MISSING_TOOL_OUTPUT_WARNING in text


class CodexRunner:
    def __init__(self, config: CodexConfig) -> None:
        self.config = config
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self._task_id: str | None = None
        self._cancel_requested = False
        self._session_integrity_error = False

    @property
    def active_task_id(self) -> str | None:
        with self._lock:
            return self._task_id

    @property
    def had_session_integrity_error(self) -> bool:
        with self._lock:
            return self._session_integrity_error

    def run(
        self,
        *,
        task_id: str,
        prompt: str,
        workspace: str,
        session_id: str | None,
        model: str,
        execution_mode: str = "execute",
        on_output: Callable[[str, str, dict | None], None],
    ) -> tuple[int, str, bool]:
        binary = Path(self.config.binary)
        if not binary.is_file():
            raise RuntimeError(f"Codex binary not found: {binary}")

        self.prepare_codex_home()

        argv = self.build_argv(
            workspace=workspace,
            session_id=session_id,
            model=model,
            execution_mode=execution_mode,
        )

        env = os.environ.copy()
        env["HOME"] = "/home/robot"
        env["CODEX_HOME"] = self.config.home
        env.setdefault("PYTHONUNBUFFERED", "1")
        # The npm-installed Codex launcher uses `#!/usr/bin/env node`. systemd
        # does not load the user's NVM shell setup, so make the launcher's Node
        # directory explicit for the child process.
        env["PATH"] = f"{binary.parent}:{env.get('PATH', '/usr/bin:/bin')}"

        started = time.monotonic()
        process = subprocess.Popen(
            argv,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            env=env,
            start_new_session=True,
        )
        with self._lock:
            if self._process is not None:
                process.terminate()
                raise RuntimeError("another Codex task is already running")
            self._process = process
            self._task_id = task_id
            self._cancel_requested = False
            self._session_integrity_error = False

        last_message = ""
        try:
            assert process.stdin is not None
            process.stdin.write(prompt)
            process.stdin.close()
            assert process.stdout is not None
            assert process.stderr is not None
            selector = selectors.DefaultSelector()
            selector.register(process.stdout, selectors.EVENT_READ, "stdout")
            selector.register(process.stderr, selectors.EVENT_READ, "stderr")

            while selector.get_map():
                if time.monotonic() - started > self.config.timeout_seconds:
                    self._terminate(process)
                    raise TimeoutError(f"Codex task exceeded {self.config.timeout_seconds} seconds")
                ready = selector.select(timeout=0.5)
                if not ready and process.poll() is not None:
                    for stream in (process.stdout, process.stderr):
                        try:
                            selector.unregister(stream)
                        except KeyError:
                            pass
                    break
                for key, _ in ready:
                    stream_name = key.data
                    line = key.fileobj.readline()
                    if line == "":
                        selector.unregister(key.fileobj)
                        continue
                    text = line.rstrip("\r\n")
                    if stream_name == "stderr" and is_missing_tool_output_warning(text):
                        with self._lock:
                            self._session_integrity_error = True
                    parsed = None
                    if stream_name == "stdout":
                        try:
                            parsed = json.loads(text)
                        except json.JSONDecodeError:
                            pass
                    if text:
                        last_message = text
                        on_output(stream_name, text, parsed)
            exit_code = process.wait()
            with self._lock:
                cancelled = self._cancel_requested
            return exit_code, last_message, cancelled
        finally:
            with self._lock:
                self._process = None
                self._task_id = None

    def prepare_codex_home(self) -> None:
        """Share credentials/configuration, but keep Agent conversations private.

        Desktop Codex records GUI-only custom tool calls in its session JSONL files.
        A background ``codex exec resume`` process cannot hydrate those calls, so
        sharing ``sessions`` causes it to fail with a missing tool-output error.
        The Agent therefore has its own session history and index, while still
        sharing the authentication and configured skills/plugins.
        """
        home = Path(self.config.home).expanduser()
        shared_home = Path(self.config.shared_home or self.config.home).expanduser()
        if not shared_home.is_dir():
            raise RuntimeError(f"shared Codex home does not exist: {shared_home}")
        if home.resolve() == shared_home.resolve():
            self._repair_models_cache(home)
            return

        home.mkdir(parents=True, exist_ok=True)
        migrated_legacy_sessions = self._isolate_session_state(home)
        if migrated_legacy_sessions:
            self._reset_rollout_index(home)
        for name in _SHARED_HOME_ENTRIES:
            source = shared_home / name
            target = home / name
            if not source.exists():
                continue
            if target.is_symlink():
                if target.resolve() != source.resolve():
                    raise RuntimeError(f"unexpected Agent Codex link: {target}")
                continue
            if target.exists():
                raise RuntimeError(f"Agent Codex home entry must be a symlink: {target}")
            target.symlink_to(source, target_is_directory=source.is_dir())
        self._repair_models_cache(home)

    @staticmethod
    def _isolate_session_state(home: Path) -> bool:
        """Replace legacy shared session links without touching their source data."""
        sessions = home / "sessions"
        migrated_legacy_sessions = sessions.is_symlink()
        if sessions.is_symlink():
            sessions.unlink()
        if sessions.exists() and not sessions.is_dir():
            raise RuntimeError(f"Agent Codex sessions path must be a directory: {sessions}")
        sessions.mkdir(parents=True, exist_ok=True)

        index = home / "session_index.jsonl"
        if index.is_symlink():
            index.unlink()
        if index.exists() and not index.is_file():
            raise RuntimeError(f"Agent Codex session index must be a file: {index}")
        return migrated_legacy_sessions

    @staticmethod
    def _reset_rollout_index(home: Path) -> None:
        """Discard only Agent-local rollout indexes left from shared sessions."""
        for pattern in ("state_*.sqlite", "state_*.sqlite-wal", "state_*.sqlite-shm"):
            for database in home.glob(pattern):
                backup = database.with_name(f"{database.name}.legacy-session-{time.time_ns()}")
                database.replace(backup)
                LOGGER.warning("moved legacy Agent rollout index to %s", backup)

    @staticmethod
    def _repair_models_cache(home: Path) -> None:
        cache = home / "models_cache.json"
        if not cache.exists():
            return
        try:
            payload = json.loads(cache.read_text(encoding="utf-8"))
            models = payload.get("models")
            valid = isinstance(models, list) and all(
                isinstance(model, dict) and isinstance(model.get("base_instructions"), str)
                for model in models
            )
        except (json.JSONDecodeError, OSError, TypeError):
            valid = False
        if valid:
            return
        backup = cache.with_name(f"models_cache.invalid-{time.time_ns()}.json")
        cache.replace(backup)
        LOGGER.warning("moved invalid Agent Codex model cache to %s", backup)

    def cancel(self, task_id: str) -> bool:
        with self._lock:
            process = self._process
            if process is None or self._task_id != task_id:
                return False
            self._cancel_requested = True
        self._terminate(process)
        return True

    def build_argv(
        self,
        *,
        workspace: str,
        session_id: str | None,
        model: str,
        execution_mode: str = "execute",
    ) -> list[str]:
        # `resume` has its own option parser.  Global `exec` options must
        # precede it, otherwise a resumed planning turn rejects `--sandbox`.
        argv = [self.config.binary, "exec", "--model", model, "--json"]
        if execution_mode == "plan":
            argv.extend(["--sandbox", "read-only"])
        elif self.config.yolo:
            argv.append("--dangerously-bypass-approvals-and-sandbox")
        else:
            argv.extend(["--sandbox", "workspace-write"])
        if session_id:
            argv.extend(["resume", session_id, "-"])
        else:
            argv.extend(["-C", workspace, "-"])
        return argv

    @staticmethod
    def _terminate(process: subprocess.Popen) -> None:
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
