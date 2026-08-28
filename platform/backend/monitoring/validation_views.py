from __future__ import annotations

import json
import re
import uuid
from pathlib import Path

from django.conf import settings
from django.core import signing
from django.contrib.auth import get_user_model
from django.db import transaction
from django.http import FileResponse, HttpResponse, HttpResponseRedirect, StreamingHttpResponse
from django.shortcuts import get_object_or_404
from django.utils import timezone
from rest_framework import status
from rest_framework.permissions import AllowAny
from rest_framework.response import Response
from rest_framework.views import APIView

from .models import (
    GoldenBaseline,
    Robot,
    TaskExecution,
    ValidationArtifact,
    ValidationJob,
    ValidationProfile,
    ValidationRecording,
    ValidationRunner,
)
from .permissions import IsValidationGateway, IsValidationRunner
from .serializers import (
    GoldenBaselineSerializer,
    ValidationJobSerializer,
    ValidationProfileSerializer,
    ValidationRecordingSerializer,
    ValidationRunnerSerializer,
)
from .services.validation_object_store import ObjectStoreError, object_store
from .services.validation_service import ValidationJobService, ValidationStateError


def _parse_json_field(value, default):
    if value in (None, ""):
        return default
    if isinstance(value, (dict, list)):
        return value
    try:
        return json.loads(value)
    except (TypeError, ValueError) as exc:
        raise ValidationStateError("JSON 字段格式无效") from exc


def _safe_name(value: str, fallback: str) -> str:
    normalized = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value or "").strip()).strip("._")
    return (normalized or fallback)[:180]


def _range_response(request, path: Path, *, content_type: str, download_name: str):
    size = path.stat().st_size
    range_header = request.headers.get("Range", "")
    match = re.fullmatch(r"bytes=(\d*)-(\d*)", range_header.strip()) if range_header else None
    if not match:
        response = FileResponse(path.open("rb"), content_type=content_type, filename=download_name)
        response["Accept-Ranges"] = "bytes"
        response["Content-Length"] = str(size)
        return response

    start_text, end_text = match.groups()
    if not start_text and not end_text:
        return HttpResponse(status=416, headers={"Content-Range": f"bytes */{size}"})
    if start_text:
        start = int(start_text)
        end = min(int(end_text) if end_text else size - 1, size - 1)
    else:
        suffix = min(int(end_text), size)
        start, end = size - suffix, size - 1
    if start >= size or end < start:
        return HttpResponse(status=416, headers={"Content-Range": f"bytes */{size}"})
    length = end - start + 1

    def chunks():
        with path.open("rb") as stream:
            stream.seek(start)
            remaining = length
            while remaining:
                data = stream.read(min(1024 * 1024, remaining))
                if not data:
                    break
                remaining -= len(data)
                yield data

    response = StreamingHttpResponse(chunks(), status=206, content_type=content_type)
    response["Accept-Ranges"] = "bytes"
    response["Content-Range"] = f"bytes {start}-{end}/{size}"
    response["Content-Length"] = str(length)
    response["Content-Disposition"] = f'inline; filename="{_safe_name(download_name, "artifact.bin")}"'
    return response


class ValidationRecordingListCreateView(APIView):
    def get(self, request):
        queryset = ValidationRecording.objects.select_related("robot", "task_execution")
        state = request.query_params.get("state", "").strip()
        robot_id = request.query_params.get("robot_id", "").strip()
        if state:
            queryset = queryset.filter(state=state)
        if robot_id:
            queryset = queryset.filter(robot_id=robot_id)
        return Response(ValidationRecordingSerializer(queryset[:200], many=True).data)

    def post(self, request):
        uploaded = request.FILES.get("file")
        if uploaded is None:
            return Response(
                {"detail": "本地上传需要 file；S3 大文件请使用 uploads/initiate 接口"},
                status=status.HTTP_400_BAD_REQUEST,
            )
        if settings.VALIDATION_OBJECT_STORE_BACKEND != "local":
            return Response(
                {"detail": "S3 模式禁止文件经过 Django，请使用分片直传接口"},
                status=status.HTTP_409_CONFLICT,
            )
        label = str(request.data.get("label") or uploaded.name or "recording").strip()[:160]
        storage_format = str(request.data.get("storage_format") or "mcap")
        if storage_format != "mcap":
            return Response({"detail": "网页上传只接受单文件 MCAP"}, status=status.HTTP_400_BAD_REQUEST)
        robot = get_object_or_404(Robot, pk=request.data["robot"]) if request.data.get("robot") else None
        execution = (
            get_object_or_404(TaskExecution, pk=request.data["task_execution"])
            if request.data.get("task_execution")
            else None
        )
        recording = ValidationRecording.objects.create(
            robot=robot,
            task_execution=execution,
            label=label,
            storage_format="mcap",
            state="uploading",
            object_key=f"recordings/{uuid.uuid4()}/{_safe_name(uploaded.name, 'source.mcap')}",
            topic_manifest=_parse_json_field(request.data.get("topic_manifest"), []),
            recording_manifest=_parse_json_field(request.data.get("recording_manifest"), {}),
        )
        try:
            size, digest = object_store.save_stream(recording.object_key, uploaded.file)
        except Exception as exc:
            recording.state = "invalid"
            recording.invalid_reason = str(exc)
            recording.save(update_fields=["state", "invalid_reason", "updated_at"])
            raise
        declared_sha = str(request.data.get("sha256") or "").lower()
        if declared_sha and declared_sha != digest:
            object_store.delete(recording.object_key)
            recording.state = "invalid"
            recording.invalid_reason = "SHA-256 与声明值不一致"
        else:
            recording.state = "ready"
            recording.sha256 = digest
            recording.size_bytes = size
            recording.uploaded_at = timezone.now()
        recording.save()
        response_status = status.HTTP_201_CREATED if recording.state == "ready" else status.HTTP_400_BAD_REQUEST
        return Response(ValidationRecordingSerializer(recording).data, status=response_status)


class ValidationRecordingUploadInitiateView(APIView):
    def post(self, request):
        if settings.VALIDATION_OBJECT_STORE_BACKEND != "s3":
            return Response({"detail": "分片上传只在 S3 模式启用"}, status=status.HTTP_409_CONFLICT)
        label = str(request.data.get("label") or "recording").strip()[:160]
        file_name = _safe_name(request.data.get("file_name"), "source.mcap")
        robot = get_object_or_404(Robot, pk=request.data["robot"]) if request.data.get("robot") else None
        execution = (
            get_object_or_404(TaskExecution, pk=request.data["task_execution"])
            if request.data.get("task_execution")
            else None
        )
        recording = ValidationRecording.objects.create(
            robot=robot,
            task_execution=execution,
            label=label,
            storage_format="mcap",
            state="uploading",
            object_key=f"recordings/{uuid.uuid4()}/{file_name}",
            sha256=str(request.data.get("sha256") or "").lower()[:64],
            topic_manifest=request.data.get("topic_manifest") or [],
            recording_manifest=request.data.get("recording_manifest") or {},
        )
        try:
            recording.upload_id = object_store.initiate_multipart(recording.object_key, "application/octet-stream")
            recording.save(update_fields=["upload_id", "updated_at"])
        except ObjectStoreError as exc:
            recording.delete()
            return Response({"detail": str(exc)}, status=status.HTTP_503_SERVICE_UNAVAILABLE)
        return Response(
            {"recording": ValidationRecordingSerializer(recording).data, "part_size_bytes": 64 * 1024 * 1024},
            status=status.HTTP_201_CREATED,
        )


class ValidationRecordingUploadPartsView(APIView):
    def post(self, request, recording_id):
        recording = get_object_or_404(ValidationRecording, id=recording_id, state="uploading")
        part_numbers = request.data.get("part_numbers") or []
        try:
            numbers = [int(value) for value in part_numbers]
            urls = object_store.presign_parts(recording.object_key, recording.upload_id, numbers)
        except (ValueError, ObjectStoreError) as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_400_BAD_REQUEST)
        return Response({"parts": urls})


class ValidationRecordingUploadCompleteView(APIView):
    def post(self, request, recording_id):
        recording = get_object_or_404(ValidationRecording, id=recording_id, state="uploading")
        try:
            info = object_store.complete_multipart(
                recording.object_key, recording.upload_id, request.data.get("parts") or []
            )
        except ObjectStoreError as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_400_BAD_REQUEST)
        recording.size_bytes = info["size_bytes"]
        recording.state = "ready"
        recording.upload_id = ""
        recording.uploaded_at = timezone.now()
        recording.save()
        return Response(ValidationRecordingSerializer(recording).data)


class ValidationProfileListView(APIView):
    def get(self, request):
        return Response(ValidationProfileSerializer(ValidationProfile.objects.filter(enabled=True), many=True).data)


class ValidationRunnerListView(APIView):
    def get(self, request):
        return Response(ValidationRunnerSerializer(ValidationRunner.objects.all(), many=True).data)


class ValidationJobListCreateView(APIView):
    def get(self, request):
        queryset = ValidationJob.objects.select_related("recording", "profile", "runner")
        state = request.query_params.get("state", "").strip()
        if state:
            queryset = queryset.filter(state=state)
        return Response(ValidationJobSerializer(queryset[:200], many=True).data)

    def post(self, request):
        recording = get_object_or_404(ValidationRecording, id=request.data.get("recording_id"))
        profile = get_object_or_404(ValidationProfile, id=request.data.get("profile_id"))
        if profile.mode == "matrix_scenario" and not ValidationRunner.objects.filter(
            kind="matrix", state__in=["idle", "busy"]
        ).exists():
            return Response(
                {"detail": "MATRiX/UE GPU Runner 尚未部署", "code": "RUNNER_UNAVAILABLE"},
                status=status.HTTP_409_CONFLICT,
            )
        requested_config = request.data.get("requested_config") or {}
        baseline = None
        if request.data.get("baseline_job_id"):
            baseline = get_object_or_404(ValidationJob, id=request.data["baseline_job_id"])
        else:
            golden = GoldenBaseline.objects.filter(
                profile=profile,
                map_hash=str(requested_config.get("map_hash") or ""),
                route_hash=str(requested_config.get("route_hash") or ""),
                active=True,
            ).select_related("job").first()
            baseline = golden.job if golden else None
        try:
            job, created = ValidationJobService.create_job(
                recording=recording,
                profile=profile,
                requested_by=request.user,
                idempotency_key=request.data.get("idempotency_key", ""),
                baseline_job=baseline,
                requested_config=requested_config,
            )
        except ValidationStateError as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_409_CONFLICT)
        return Response(
            ValidationJobSerializer(job).data,
            status=status.HTTP_201_CREATED if created else status.HTTP_200_OK,
        )


class ValidationJobDetailView(APIView):
    def get(self, request, job_id):
        job = get_object_or_404(
            ValidationJob.objects.select_related("recording", "profile", "runner").prefetch_related(
                "check_results", "artifacts", "attempts__runner"
            ),
            id=job_id,
        )
        return Response(ValidationJobSerializer(job).data)


class ValidationJobCancelView(APIView):
    def post(self, request, job_id):
        job = get_object_or_404(ValidationJob, id=job_id)
        job = ValidationJobService.request_cancel(job)
        return Response(ValidationJobSerializer(job).data)


class ValidationJobReportView(APIView):
    def get(self, request, job_id):
        job = get_object_or_404(
            ValidationJob.objects.select_related("recording", "profile", "baseline_job").prefetch_related(
                "check_results", "artifacts", "attempts__runner"
            ),
            id=job_id,
        )
        return Response(ValidationJobSerializer(job).data)


class ValidationJobApproveBaselineView(APIView):
    def post(self, request, job_id):
        job = get_object_or_404(ValidationJob, id=job_id)
        try:
            baseline = ValidationJobService.approve_baseline(job, request.user)
        except ValidationStateError as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_409_CONFLICT)
        return Response(GoldenBaselineSerializer(baseline).data, status=status.HTTP_201_CREATED)


class ValidationJobLiveTicketView(APIView):
    def post(self, request, job_id):
        job = get_object_or_404(ValidationJob, id=job_id)
        if job.state not in {"staging", "running", "analyzing", "uploading", "cancelling"}:
            return Response({"detail": "作业当前没有实时话题"}, status=status.HTTP_409_CONFLICT)
        if not job.live_bridge_url:
            return Response({"detail": "Runner 尚未发布实时桥接地址"}, status=status.HTTP_409_CONFLICT)
        ticket = signing.dumps(
            {"job": str(job.id), "user": request.user.pk},
            salt="validation-live-ticket",
            compress=True,
        )
        return Response({"ticket": ticket, "expires_in": settings.VALIDATION_LIVE_TICKET_TTL_SECONDS})


class ValidationLiveTicketVerifyView(APIView):
    permission_classes = [IsValidationGateway]

    def post(self, request):
        try:
            payload = signing.loads(
                str(request.data.get("ticket") or ""),
                salt="validation-live-ticket",
                max_age=settings.VALIDATION_LIVE_TICKET_TTL_SECONDS,
            )
            job_id = uuid.UUID(str(payload["job"]))
            user_id = int(payload["user"])
        except (KeyError, TypeError, ValueError, signing.BadSignature, signing.SignatureExpired):
            return Response({"detail": "实时票据无效或已过期"}, status=status.HTTP_403_FORBIDDEN)
        if not get_user_model().objects.filter(pk=user_id, is_active=True).exists():
            return Response({"detail": "用户已失效"}, status=status.HTTP_403_FORBIDDEN)
        job = get_object_or_404(ValidationJob, id=job_id)
        if job.state not in {"staging", "running", "analyzing", "uploading", "cancelling"}:
            return Response({"detail": "作业当前没有实时话题"}, status=status.HTTP_409_CONFLICT)
        return Response({"job_id": str(job.id), "state": job.state, "live_bridge_url": job.live_bridge_url})


class ValidationArtifactDownloadView(APIView):
    def get(self, request, job_id, artifact_id):
        artifact = get_object_or_404(ValidationArtifact, id=artifact_id, job_id=job_id)
        signed = object_store.download_url(artifact.object_key)
        if signed:
            return HttpResponseRedirect(signed)
        return _range_response(
            request,
            object_store.local_path(artifact.object_key),
            content_type=artifact.content_type,
            download_name=artifact.name,
        )


class ValidationArtifactSignedDownloadView(APIView):
    permission_classes = [AllowAny]

    def get(self, request, artifact_id):
        token = request.query_params.get("token", "")
        try:
            unsigned = signing.TimestampSigner(salt="validation-artifact").unsign(
                token, max_age=settings.VALIDATION_SIGNED_URL_TTL_SECONDS
            )
        except signing.BadSignature:
            return Response({"detail": "制品链接无效或已过期"}, status=status.HTTP_403_FORBIDDEN)
        if unsigned != str(artifact_id):
            return Response({"detail": "制品链接目标不匹配"}, status=status.HTTP_403_FORBIDDEN)
        artifact = get_object_or_404(ValidationArtifact, id=artifact_id)
        signed = object_store.download_url(artifact.object_key)
        if signed:
            return HttpResponseRedirect(signed)
        return _range_response(
            request,
            object_store.local_path(artifact.object_key),
            content_type=artifact.content_type,
            download_name=artifact.name,
        )


class ValidationRunnerClaimView(APIView):
    permission_classes = [IsValidationRunner]

    def post(self, request, runner_id):
        runner, _ = ValidationRunner.objects.update_or_create(
            id=runner_id,
            defaults={
                "display_name": str(request.data.get("display_name") or runner_id)[:128],
                "kind": str(request.data.get("kind") or "cpu")[:16],
                "capabilities": request.data.get("capabilities") or ["bag_replay"],
                "version": str(request.data.get("version") or "")[:128],
                "private_address": str(request.data.get("private_address") or "")[:256],
                "last_seen_at": timezone.now(),
            },
        )
        job = ValidationJobService.claim(runner)
        if job is None:
            return Response(status=status.HTTP_204_NO_CONTENT)
        source_url = object_store.download_url(job.recording.object_key)
        if not source_url:
            source_url = request.build_absolute_uri(
                f"/api/internal/validation-jobs/{job.id}/source/"
            )
        baseline_summary = job.baseline_job.summary if job.baseline_job_id else None
        return Response(
            {
                "id": str(job.id),
                "lease_token": str(job.lease_token),
                "mode": job.mode,
                "source": {
                    "url": source_url,
                    "sha256": job.recording.sha256,
                    "size_bytes": job.recording.size_bytes,
                    "storage_format": job.recording.storage_format,
                },
                "profile": job.resolved_config,
                "requested_config": job.requested_config,
                "baseline_summary": baseline_summary,
                "cancel_requested": job.state == "cancelling",
            }
        )


class ValidationRunnerHeartbeatView(APIView):
    permission_classes = [IsValidationRunner]

    def post(self, request, runner_id, job_id):
        runner = get_object_or_404(ValidationRunner, id=runner_id)
        try:
            job = ValidationJobService.heartbeat(job_id=job_id, runner=runner, payload=request.data)
        except ValidationStateError as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_409_CONFLICT)
        return Response({"state": job.state, "cancel_requested": job.state == "cancelling"})


class ValidationRunnerCompleteView(APIView):
    permission_classes = [IsValidationRunner]

    def post(self, request, runner_id, job_id):
        runner = get_object_or_404(ValidationRunner, id=runner_id)
        try:
            job = ValidationJobService.complete(job_id=job_id, runner=runner, payload=request.data)
        except ValidationStateError as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_409_CONFLICT)
        return Response({"state": job.state, "verdict": job.verdict})


class ValidationRunnerSourceView(APIView):
    permission_classes = [IsValidationRunner]

    def get(self, request, job_id):
        job = get_object_or_404(ValidationJob.objects.select_related("recording"), id=job_id)
        return _range_response(
            request,
            object_store.local_path(job.recording.object_key),
            content_type="application/octet-stream",
            download_name=f"{job.recording.id}.mcap",
        )


class ValidationRunnerArtifactUploadView(APIView):
    permission_classes = [IsValidationRunner]

    def post(self, request, runner_id, job_id):
        runner = get_object_or_404(ValidationRunner, id=runner_id)
        job = get_object_or_404(ValidationJob, id=job_id)
        try:
            ValidationJobService._validate_lease(job, runner, request.data.get("lease_token", ""))
        except ValidationStateError as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_409_CONFLICT)
        uploaded = request.FILES.get("file")
        if uploaded is None:
            return Response({"detail": "file 不能为空"}, status=status.HTTP_400_BAD_REQUEST)
        if settings.VALIDATION_OBJECT_STORE_BACKEND != "local":
            return Response(
                {"detail": "S3 模式请使用 artifact-presign 接口直传"}, status=status.HTTP_409_CONFLICT
            )
        role = str(request.data.get("role") or "evidence")
        if role not in {choice[0] for choice in ValidationArtifact.ROLE_CHOICES}:
            return Response({"detail": "artifact role 无效"}, status=status.HTTP_400_BAD_REQUEST)
        name = _safe_name(request.data.get("name") or uploaded.name, "artifact.bin")
        object_key = f"jobs/{job.id}/{role}/{uuid.uuid4()}-{name}"
        size, digest = object_store.save_stream(object_key, uploaded.file)
        artifact = ValidationArtifact.objects.create(
            job=job,
            role=role,
            name=name,
            object_key=object_key,
            sha256=digest,
            size_bytes=size,
            content_type=uploaded.content_type or "application/octet-stream",
        )
        return Response({"artifact_id": str(artifact.id), "sha256": digest, "size_bytes": size}, status=201)


class ValidationRunnerArtifactPresignView(APIView):
    permission_classes = [IsValidationRunner]

    def post(self, request, runner_id, job_id):
        runner = get_object_or_404(ValidationRunner, id=runner_id)
        job = get_object_or_404(ValidationJob, id=job_id)
        try:
            ValidationJobService._validate_lease(job, runner, request.data.get("lease_token", ""))
        except ValidationStateError as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_409_CONFLICT)
        role = str(request.data.get("role") or "evidence")
        if role not in {choice[0] for choice in ValidationArtifact.ROLE_CHOICES}:
            return Response({"detail": "artifact role 无效"}, status=status.HTTP_400_BAD_REQUEST)
        name = _safe_name(request.data.get("name"), "artifact.bin")
        content_type = str(request.data.get("content_type") or "application/octet-stream")[:128]
        artifact = ValidationArtifact.objects.create(
            job=job,
            role=role,
            name=name,
            object_key=f"jobs/{job.id}/{role}/{uuid.uuid4()}-{name}",
            content_type=content_type,
            metadata={"pending": True},
        )
        try:
            url = object_store.upload_url(artifact.object_key, content_type)
        except ObjectStoreError as exc:
            artifact.delete()
            return Response({"detail": str(exc)}, status=status.HTTP_503_SERVICE_UNAVAILABLE)
        return Response({"artifact_id": str(artifact.id), "url": url}, status=201)


class ValidationRunnerArtifactFinalizeView(APIView):
    permission_classes = [IsValidationRunner]

    def post(self, request, runner_id, job_id, artifact_id):
        runner = get_object_or_404(ValidationRunner, id=runner_id)
        job = get_object_or_404(ValidationJob, id=job_id)
        artifact = get_object_or_404(ValidationArtifact, id=artifact_id, job=job)
        try:
            ValidationJobService._validate_lease(job, runner, request.data.get("lease_token", ""))
            info = object_store.head(artifact.object_key)
        except (ValidationStateError, ObjectStoreError) as exc:
            return Response({"detail": str(exc)}, status=status.HTTP_409_CONFLICT)
        artifact.size_bytes = info["size_bytes"]
        artifact.sha256 = str(request.data.get("sha256") or "").lower()[:64]
        artifact.metadata = request.data.get("metadata") or {}
        artifact.save()
        return Response({"artifact_id": str(artifact.id), "size_bytes": artifact.size_bytes})
