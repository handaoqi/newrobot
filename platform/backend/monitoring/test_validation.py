from __future__ import annotations

import hashlib
import tempfile
from pathlib import Path

from django.contrib.auth import get_user_model
from django.core.files.uploadedfile import SimpleUploadedFile
from django.test import TestCase, override_settings
from rest_framework.test import APIClient

from .models import GoldenBaseline, ValidationArtifact, ValidationJob, ValidationProfile, ValidationRecording
from .services.validation_object_store import object_store


@override_settings(
    VALIDATION_OBJECT_STORE_BACKEND="local",
    VALIDATION_RUNNER_TOKEN="runner-secret",
    VALIDATION_GATEWAY_TOKEN="gateway-secret",
    VALIDATION_RUNNER_LEASE_SECONDS=45,
    VALIDATION_MAX_ATTEMPTS=2,
)
class ValidationApiTests(TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.original_backend = object_store.backend
        self.original_root = object_store.root
        object_store.backend = "local"
        object_store.root = Path(self.temporary.name)
        self.user = get_user_model().objects.create_user(username="validator", password="secret")
        self.client = APIClient()
        self.client.force_authenticate(self.user)
        self.profile = ValidationProfile.objects.create(
            name="navigation-test",
            version=1,
            mode="bag_replay",
            required_topics=["/tf"],
            thresholds={"max_timestamp_regressions": 0},
        )

    def tearDown(self):
        object_store.backend = self.original_backend
        object_store.root = self.original_root
        self.temporary.cleanup()

    def upload_recording(self, content=b"not-a-real-mcap-but-immutable"):
        response = self.client.post(
            "/api/validation-recordings/",
            {"label": "navigation-smoke", "storage_format": "mcap", "file": SimpleUploadedFile("source.mcap", content)},
            format="multipart",
        )
        self.assertEqual(response.status_code, 201, response.data)
        return ValidationRecording.objects.get(id=response.data["id"])

    def create_job(self, recording, key="test-key"):
        return self.client.post(
            "/api/validation-jobs/",
            {"recording_id": str(recording.id), "profile_id": str(self.profile.id), "idempotency_key": key},
            format="json",
        )

    def runner_client(self):
        client = APIClient()
        client.credentials(HTTP_X_VALIDATION_RUNNER_TOKEN="runner-secret")
        return client

    def test_recording_upload_is_content_addressed_and_ready(self):
        content = b"immutable mcap fixture"
        recording = self.upload_recording(content)

        self.assertEqual(recording.state, "ready")
        self.assertEqual(recording.sha256, hashlib.sha256(content).hexdigest())
        self.assertEqual(recording.size_bytes, len(content))
        self.assertEqual(object_store.local_path(recording.object_key).read_bytes(), content)

    def test_job_creation_is_idempotent_per_user(self):
        recording = self.upload_recording()
        first = self.create_job(recording)
        second = self.create_job(recording)

        self.assertEqual(first.status_code, 201)
        self.assertEqual(second.status_code, 200)
        self.assertEqual(first.data["id"], second.data["id"])
        self.assertEqual(ValidationJob.objects.count(), 1)

    def test_runner_claim_heartbeat_complete_and_baseline_lifecycle(self):
        recording = self.upload_recording()
        created = self.create_job(recording)
        job_id = created.data["id"]
        runner = self.runner_client()

        claim = runner.post(
            "/api/internal/validation-runners/cpu-test/claim/",
            {"kind": "cpu", "capabilities": ["bag_replay"], "version": "test"},
            format="json",
        )
        self.assertEqual(claim.status_code, 200, claim.data)
        self.assertEqual(claim.data["id"], job_id)
        lease = claim.data["lease_token"]

        heartbeat = runner.post(
            f"/api/internal/validation-runners/cpu-test/jobs/{job_id}/heartbeat/",
            {"lease_token": lease, "state": "running", "progress_percent": 40},
            format="json",
        )
        self.assertEqual(heartbeat.status_code, 200, heartbeat.data)

        completed = runner.post(
            f"/api/internal/validation-runners/cpu-test/jobs/{job_id}/complete/",
            {
                "lease_token": lease,
                "outcome": "completed",
                "summary": {"metrics": {"message_count": 10}},
                "stack_image_digest": "sha256:test",
                "checks": [{
                    "rule_id": "input.required_topics", "title": "必需话题", "category": "input",
                    "status": "PASS", "hard_failure": True,
                }],
            },
            format="json",
        )
        self.assertEqual(completed.status_code, 200, completed.data)
        self.assertEqual(completed.data, {"state": "completed", "verdict": "WARN"})

        approved = self.client.post(f"/api/validation-jobs/{job_id}/approve-baseline/", {}, format="json")
        self.assertEqual(approved.status_code, 201, approved.data)
        self.assertTrue(GoldenBaseline.objects.filter(job_id=job_id, active=True).exists())

    def test_runner_token_and_lease_are_both_required(self):
        recording = self.upload_recording()
        job_id = self.create_job(recording).data["id"]
        anonymous = APIClient()
        denied = anonymous.post("/api/internal/validation-runners/cpu-test/claim/", {}, format="json")
        self.assertIn(denied.status_code, {401, 403})

        runner = self.runner_client()
        claim = runner.post(
            "/api/internal/validation-runners/cpu-test/claim/",
            {"capabilities": ["bag_replay"]},
            format="json",
        )
        rejected = runner.post(
            f"/api/internal/validation-runners/cpu-test/jobs/{job_id}/heartbeat/",
            {"lease_token": "00000000-0000-0000-0000-000000000000", "state": "running", "progress_percent": 10},
            format="json",
        )
        self.assertEqual(claim.status_code, 200)
        self.assertEqual(rejected.status_code, 409)

    def test_queued_job_cancels_without_runner(self):
        recording = self.upload_recording()
        job_id = self.create_job(recording).data["id"]

        cancelled = self.client.post(f"/api/validation-jobs/{job_id}/cancel/", {}, format="json")

        self.assertEqual(cancelled.status_code, 200)
        self.assertEqual(cancelled.data["state"], "cancelled")

    def test_signed_artifact_supports_http_range_without_user_token(self):
        recording = self.upload_recording()
        job = ValidationJob.objects.get(id=self.create_job(recording).data["id"])
        object_key = f"jobs/{job.id}/result/result.mcap"
        size, digest = object_store.save_stream(object_key, SimpleUploadedFile("result.mcap", b"0123456789"))
        artifact = ValidationArtifact.objects.create(
            job=job, role="result_mcap", name="result.mcap", object_key=object_key,
            sha256=digest, size_bytes=size, content_type="application/octet-stream",
        )
        detail = self.client.get(f"/api/validation-jobs/{job.id}/")
        signed_path = detail.data["artifacts"][0]["signed_download_path"]

        response = APIClient().get(signed_path, HTTP_RANGE="bytes=2-5")

        self.assertEqual(response.status_code, 206)
        self.assertEqual(b"".join(response.streaming_content), b"2345")
        self.assertEqual(response["Content-Range"], "bytes 2-5/10")

    def test_matrix_mode_fails_closed_without_gpu_runner(self):
        recording = self.upload_recording()
        matrix = ValidationProfile.objects.create(name="matrix", version=1, mode="matrix_scenario")

        response = self.client.post(
            "/api/validation-jobs/",
            {"recording_id": str(recording.id), "profile_id": str(matrix.id)},
            format="json",
        )

        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.data["code"], "RUNNER_UNAVAILABLE")

    def test_live_bridge_uses_short_lived_job_ticket(self):
        recording = self.upload_recording()
        job_id = self.create_job(recording).data["id"]
        runner = self.runner_client()
        claim = runner.post(
            "/api/internal/validation-runners/cpu-test/claim/",
            {"capabilities": ["bag_replay"]},
            format="json",
        )
        heartbeat = runner.post(
            f"/api/internal/validation-runners/cpu-test/jobs/{job_id}/heartbeat/",
            {
                "lease_token": claim.data["lease_token"],
                "state": "running",
                "progress_percent": 30,
                "live_bridge_url": "ws://10.20.0.12:8812",
            },
            format="json",
        )
        self.assertEqual(heartbeat.status_code, 200, heartbeat.data)

        issued = self.client.post(f"/api/validation-jobs/{job_id}/live-ticket/", {}, format="json")
        gateway = APIClient()
        gateway.credentials(HTTP_X_VALIDATION_GATEWAY_TOKEN="gateway-secret")
        verified = gateway.post(
            "/api/internal/validation-live-ticket/verify/",
            {"ticket": issued.data["ticket"]},
            format="json",
        )

        self.assertEqual(issued.status_code, 200, issued.data)
        self.assertNotIn("inspection_token", issued.data["ticket"])
        self.assertEqual(verified.status_code, 200, verified.data)
        self.assertEqual(verified.data["job_id"], job_id)
        self.assertEqual(verified.data["live_bridge_url"], "ws://10.20.0.12:8812")

        denied = APIClient().post(
            "/api/internal/validation-live-ticket/verify/",
            {"ticket": issued.data["ticket"]},
            format="json",
        )
        self.assertIn(denied.status_code, {401, 403})
