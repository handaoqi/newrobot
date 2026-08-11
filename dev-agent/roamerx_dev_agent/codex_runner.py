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


class CodexRunner:
    def __init__(self, config: CodexConfig) -> None:
        self.config = config
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self._task_id: str | None = None
        self._cancel_requested = False

    @property
    def active_task_id(self) -> str | None:
        with self._lock:
            return self._task_id

    def run(
        self,
        *,
        task_id: str,
        prompt: str,
        workspace: str,
        session_id: str | None,
        on_output: Callable[[str, str, dict | None], None],
    ) -> tuple[int, str, bool]:
        binary = Path(self.config.binary)
        if not binary.is_file():
            raise RuntimeError(f"Codex binary not found: {binary}")

        argv = self.build_argv(workspace=workspace, session_id=session_id)

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

    def cancel(self, task_id: str) -> bool:
        with self._lock:
            process = self._process
            if process is None or self._task_id != task_id:
                return False
            self._cancel_requested = True
        self._terminate(process)
        return True

    def build_argv(self, *, workspace: str, session_id: str | None) -> list[str]:
        argv = [self.config.binary, "exec"]
        if session_id:
            argv.append("resume")
        argv.append("--json")
        if self.config.yolo:
            argv.append("--dangerously-bypass-approvals-and-sandbox")
        elif not session_id:
            argv.extend(["--sandbox", "workspace-write"])
        if session_id:
            argv.extend([session_id, "-"])
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
