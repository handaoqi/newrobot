#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import os
import shutil
import signal
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from typing import Any

import requests

from analyzer import inspect_mcap


class RunnerClient:
    def __init__(self) -> None:
        self.base_url = os.environ.get("ROAMERX_PLATFORM_URL", "http://127.0.0.1:8088").rstrip("/")
        self.token = os.environ.get("ROAMERX_VALIDATION_RUNNER_TOKEN", "")
        self.runner_id = os.environ.get("ROAMERX_VALIDATION_RUNNER_ID", "cpu-runner-1")
        self.workspace_root = Path(os.environ.get("ROAMERX_VALIDATION_WORK_ROOT", "/var/lib/roamerx-validation"))
        self.version = os.environ.get("ROAMERX_VALIDATION_RUNNER_VERSION", "dev")
        self.private_address = os.environ.get("ROAMERX_VALIDATION_PRIVATE_ADDRESS", "")
        if not self.token:
            raise RuntimeError("ROAMERX_VALIDATION_RUNNER_TOKEN is required")
        self.headers = {"X-Validation-Runner-Token": self.token}

    def claim(self) -> dict | None:
        response = requests.post(
            f"{self.base_url}/api/internal/validation-runners/{self.runner_id}/claim/",
            json={
                "display_name": self.runner_id,
                "kind": "cpu",
                "capabilities": ["bag_replay"],
                "version": self.version,
                "private_address": self.private_address,
            },
            headers=self.headers,
            timeout=20,
        )
        if response.status_code == 204:
            return None
        response.raise_for_status()
        return response.json()

    def heartbeat(self, job: dict, **payload) -> dict:
        response = requests.post(
            f"{self.base_url}/api/internal/validation-runners/{self.runner_id}/jobs/{job['id']}/heartbeat/",
            json={"lease_token": job["lease_token"], **payload},
            headers=self.headers,
            timeout=15,
        )
        response.raise_for_status()
        return response.json()

    def complete(self, job: dict, payload: dict) -> None:
        response = requests.post(
            f"{self.base_url}/api/internal/validation-runners/{self.runner_id}/jobs/{job['id']}/complete/",
            json={"lease_token": job["lease_token"], **payload},
            headers=self.headers,
            timeout=30,
        )
        response.raise_for_status()

    def download(self, job: dict, destination: Path) -> None:
        with requests.get(job["source"]["url"], headers=self.headers, stream=True, timeout=60) as response:
            response.raise_for_status()
            digest = hashlib.sha256()
            size = 0
            with destination.open("wb") as output:
                for chunk in response.iter_content(1024 * 1024):
                    if not chunk:
                        continue
                    output.write(chunk)
                    digest.update(chunk)
                    size += len(chunk)
        expected_size = int(job["source"].get("size_bytes") or 0)
        expected_sha = str(job["source"].get("sha256") or "").lower()
        if expected_size and size != expected_size:
            raise RuntimeError(f"SOURCE_SIZE_MISMATCH: expected {expected_size}, got {size}")
        if expected_sha and digest.hexdigest() != expected_sha:
            raise RuntimeError("SOURCE_SHA256_MISMATCH")

    def upload_artifact(self, job: dict, path: Path, role: str, content_type: str) -> dict:
        # Local/developer backend accepts a streamed multipart request. S3
        # deployments respond 409 and use a direct pre-signed PUT instead.
        endpoint = (
            f"{self.base_url}/api/internal/validation-runners/{self.runner_id}/jobs/{job['id']}/artifacts/"
        )
        with path.open("rb") as stream:
            response = requests.post(
                endpoint,
                data={"lease_token": job["lease_token"], "role": role, "name": path.name},
                files={"file": (path.name, stream, content_type)},
                headers=self.headers,
                timeout=300,
            )
        if response.status_code != 409:
            response.raise_for_status()
            return response.json()

        digest_builder = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest_builder.update(chunk)
        digest = digest_builder.hexdigest()
        presign = requests.post(
            endpoint + "presign/",
            json={
                "lease_token": job["lease_token"], "role": role, "name": path.name,
                "content_type": content_type,
            },
            headers=self.headers,
            timeout=20,
        )
        presign.raise_for_status()
        spec = presign.json()
        with path.open("rb") as stream:
            uploaded = requests.put(spec["url"], data=stream, headers={"Content-Type": content_type}, timeout=900)
            uploaded.raise_for_status()
        finalize = requests.post(
            endpoint + f"{spec['artifact_id']}/finalize/",
            json={"lease_token": job["lease_token"], "sha256": digest},
            headers=self.headers,
            timeout=20,
        )
        finalize.raise_for_status()
        return finalize.json()


class Heartbeat:
    def __init__(self, client: RunnerClient, job: dict) -> None:
        self.client = client
        self.job = job
        self.stop_event = threading.Event()
        self.cancel_event = threading.Event()
        self.state = "staging"
        self.progress = 1
        self.live_bridge_url = ""
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def update(self, state: str, progress: int, *, live_bridge_url: str | None = None) -> None:
        self.state, self.progress = state, progress
        if live_bridge_url is not None:
            self.live_bridge_url = live_bridge_url

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=5)

    def pulse(self) -> None:
        result = self.client.heartbeat(
            self.job,
            state=self.state,
            progress_percent=self.progress,
            live_bridge_url=self.live_bridge_url,
        )
        if result.get("cancel_requested"):
            self.cancel_event.set()

    def _run(self) -> None:
        while not self.stop_event.wait(10):
            try:
                self.pulse()
            except Exception as exc:
                print(f"heartbeat failed: {exc}", flush=True)


def _run_trusted_replay(job: dict, source: Path, workspace: Path, heartbeat: Heartbeat) -> Path | None:
    """Optionally run the admin-owned algorithm command and record derived topics."""
    config = job["profile"].get("runner_config") or {}
    command = config.get("launch_command") or []
    if not command:
        return None
    if not isinstance(command, list) or not command or command[0] not in {"ros2", "/opt/roamerx/bin/validation-launch"}:
        raise RuntimeError("PROFILE_LAUNCH_COMMAND_REJECTED")
    if any(not isinstance(value, str) or "\x00" in value for value in command):
        raise RuntimeError("PROFILE_LAUNCH_COMMAND_REJECTED")

    domain = 100 + (int(job["id"].replace("-", "")[:4], 16) % 100)
    while domain in {124, 177, 189}:  # avoids production/demo/MATRiX after the +100 mapping
        domain += 1
    env = {
        **os.environ,
        "ROS_DOMAIN_ID": str(domain),
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "ROS_HOME": str(workspace / "ros-home"),
    }
    output_dir = workspace / "result"
    output_topics = job["profile"].get("output_topics") or []
    replay_topics = job["profile"].get("replay_topics") or []
    if not output_topics or not replay_topics:
        raise RuntimeError("PROFILE_TOPIC_CONTRACT_EMPTY")
    rate = float(config.get("replay_rate", 1.0))
    clock_hz = int(config.get("clock_hz", 100))
    processes: list[subprocess.Popen] = []

    def start(args):
        process = subprocess.Popen(args, env=env, cwd=workspace, start_new_session=True)
        processes.append(process)
        return process

    try:
        algorithm = start(command)
        recorder = start([
            "ros2", "bag", "record", "--storage", "mcap", "--output", str(output_dir), *output_topics
        ])
        bridge = None
        if client_address := os.environ.get("ROAMERX_VALIDATION_PRIVATE_ADDRESS", "").strip():
            # The bridge is deliberately read-only: clients can only subscribe
            # to a bounded set of diagnostic topics and inspect the graph.
            bridge_port = 8800 + (int(job["id"].replace("-", "")[-4:], 16) % 700)
            bridge_params = workspace / "foxglove-bridge.yaml"
            bridge_params.write_text(
                "\n".join([
                    "/**:",
                    "  ros__parameters:",
                    "    address: 0.0.0.0",
                    f"    port: {bridge_port}",
                    "    capabilities: [connectionGraph]",
                    "    client_topic_whitelist: ['^$']",
                    "    service_whitelist: ['^$']",
                    "    param_whitelist: ['^$']",
                    "    asset_uri_allowlist: ['^$']",
                    "    topic_whitelist: ['^/(clock|map|tf|tf_static|odom|cmd_vel|scan|points_raw|plan|diagnostics|rosout)(/.*)?$']",
                    "    use_compression: true",
                    "    use_sim_time: true",
                    "",
                ]),
                encoding="utf-8",
            )
            bridge = start([
                "ros2", "run", "foxglove_bridge", "foxglove_bridge",
                "--ros-args", "--params-file", str(bridge_params),
            ])
            heartbeat.update(
                "staging", 20, live_bridge_url=f"ws://{client_address}:{bridge_port}"
            )
            heartbeat.pulse()
        time.sleep(3)
        if recorder.poll() is not None:
            raise RuntimeError(f"ROSBAG_RECORD_FAILED: {recorder.returncode}")
        if bridge is not None and bridge.poll() is not None:
            raise RuntimeError(f"FOXGLOVE_BRIDGE_FAILED: {bridge.returncode}")
        player = start([
            "ros2", "bag", "play", str(source), "--clock", str(clock_hz), "--rate", str(rate),
            "--topics", *replay_topics,
        ])
        heartbeat.update("running", 30)
        while player.poll() is None:
            if heartbeat.cancel_event.wait(1):
                raise InterruptedError("validation job cancelled")
            if algorithm.poll() is not None:
                raise RuntimeError(f"ALGORITHM_EXITED: {algorithm.returncode}")
        if player.returncode != 0:
            raise RuntimeError(f"ROSBAG_PLAY_FAILED: {player.returncode}")
        heartbeat.update("analyzing", 70)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 15
        for process in reversed(processes):
            try:
                process.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
    return next(output_dir.glob("*.mcap"), None) if output_dir.exists() else None


def process_job(client: RunnerClient, job: dict) -> None:
    workspace = client.workspace_root / job["id"]
    if workspace.exists():
        shutil.rmtree(workspace)
    workspace.mkdir(parents=True)
    heartbeat = Heartbeat(client, job)
    heartbeat.start()
    try:
        source = workspace / "source.mcap"
        client.download(job, source)
        heartbeat.update("staging", 15)
        result_mcap = _run_trusted_replay(job, source, workspace, heartbeat)
        analyzed = result_mcap or source
        heartbeat.update("analyzing", 75)
        checks, summary = inspect_mcap(analyzed, job["profile"], job.get("baseline_summary"))
        summary["execution_mode"] = "algorithm_replay" if result_mcap else "contract_inspection"
        report_path = workspace / "validation-report.json"
        report_path.write_text(
            json.dumps({"job_id": job["id"], "summary": summary, "checks": checks}, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        heartbeat.update("uploading", 90)
        client.upload_artifact(job, report_path, "report", "application/json")
        if result_mcap:
            client.upload_artifact(job, result_mcap, "result_mcap", "application/octet-stream")
        client.complete(
            job,
            {
                "outcome": "completed",
                "checks": checks,
                "summary": summary,
                "stack_git_sha": os.environ.get("ROAMERX_STACK_GIT_SHA", ""),
                "stack_image_digest": os.environ.get("ROAMERX_STACK_IMAGE_DIGEST", "unversioned"),
            },
        )
    except InterruptedError:
        client.complete(job, {"outcome": "cancelled"})
    except Exception as exc:
        client.complete(
            job,
            {"outcome": "infra_error", "error_code": type(exc).__name__[:64], "error_message": str(exc)},
        )
    finally:
        heartbeat.close()


def main() -> None:
    client = RunnerClient()
    client.workspace_root.mkdir(parents=True, exist_ok=True)
    poll_seconds = max(2.0, float(os.environ.get("ROAMERX_VALIDATION_POLL_SECONDS", "5")))
    once = os.environ.get("ROAMERX_VALIDATION_ONCE", "false").lower() == "true"
    while True:
        job = client.claim()
        if job:
            process_job(client, job)
        elif once:
            return
        time.sleep(poll_seconds)


if __name__ == "__main__":
    main()
