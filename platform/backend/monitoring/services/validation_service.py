from __future__ import annotations

import uuid
from datetime import timedelta

from django.conf import settings
from django.db import transaction
from django.utils import timezone

from ..models import (
    GoldenBaseline,
    ValidationArtifact,
    ValidationAttempt,
    ValidationCheckResult,
    ValidationJob,
    ValidationRunner,
)


class ValidationStateError(ValueError):
    pass


def compute_verdict(checks: list[dict], *, has_baseline: bool) -> str:
    statuses = {str(check.get("status") or "NOT_EVALUATED").upper() for check in checks}
    if "FAIL" in statuses:
        return "FAIL"
    if "WARN" in statuses or not has_baseline or "NOT_EVALUATED" in statuses:
        return "WARN"
    return "PASS"


class ValidationJobService:
    @classmethod
    @transaction.atomic
    def create_job(
        cls,
        *,
        recording,
        profile,
        requested_by,
        idempotency_key: str = "",
        baseline_job=None,
        requested_config: dict | None = None,
    ) -> tuple[ValidationJob, bool]:
        if recording.state != "ready":
            raise ValidationStateError("录制尚未上传并校验为 ready")
        if not profile.enabled:
            raise ValidationStateError("检查 Profile 已停用")
        if baseline_job and (
            baseline_job.profile_id != profile.id or baseline_job.verdict not in {"PASS", "WARN"}
        ):
            raise ValidationStateError("黄金基线必须来自同一 Profile 的可用作业")
        key = str(idempotency_key or "").strip()[:128]
        if key:
            existing = ValidationJob.objects.filter(
                requested_by=requested_by, idempotency_key=key
            ).first()
            if existing:
                return existing, False
        job = ValidationJob.objects.create(
            recording=recording,
            profile=profile,
            baseline_job=baseline_job,
            mode=profile.mode,
            requested_by=requested_by,
            idempotency_key=key,
            requested_config=requested_config or {},
            resolved_config={
                "profile_name": profile.name,
                "profile_version": profile.version,
                "required_topics": profile.required_topics,
                "replay_topics": profile.replay_topics,
                "output_topics": profile.output_topics,
                "thresholds": profile.thresholds,
                "runner_config": profile.runner_config,
            },
        )
        return job, True

    @classmethod
    @transaction.atomic
    def reclaim_expired(cls) -> int:
        now = timezone.now()
        reclaimed = 0
        jobs = ValidationJob.objects.select_for_update().filter(
            state__in=["staging", "running", "analyzing", "uploading", "cancelling"],
            lease_expires_at__lt=now,
        )
        for job in jobs:
            attempt = job.attempts.filter(number=job.attempt_count).first()
            if attempt:
                attempt.state = "infra_error"
                attempt.finished_at = now
                attempt.error_code = "RUNNER_LEASE_EXPIRED"
                attempt.error_message = "Runner 心跳租约已过期"
                attempt.save(update_fields=["state", "finished_at", "error_code", "error_message", "updated_at"])
            if job.state == "cancelling":
                job.state = "cancelled"
                job.finished_at = now
            elif job.attempt_count < settings.VALIDATION_MAX_ATTEMPTS:
                job.state = "queued"
                job.runner = None
                job.lease_token = None
                job.lease_expires_at = None
                job.progress_percent = 0
            else:
                job.state = "infra_error"
                job.error_code = "RUNNER_LEASE_EXPIRED"
                job.error_message = "Runner 心跳租约已过期，重试次数已耗尽"
                job.finished_at = now
            job.save()
            reclaimed += 1
        return reclaimed

    @classmethod
    @transaction.atomic
    def claim(cls, runner: ValidationRunner) -> ValidationJob | None:
        cls.reclaim_expired()
        supported_modes = set(runner.capabilities or ["bag_replay"])
        job = (
            ValidationJob.objects.select_for_update(skip_locked=True)
            .select_related("recording", "profile", "baseline_job")
            .filter(state="queued", mode__in=supported_modes)
            .order_by("queued_at")
            .first()
        )
        now = timezone.now()
        runner.last_seen_at = now
        if not job:
            runner.state = "idle"
            runner.save(update_fields=["last_seen_at", "state", "updated_at"])
            return None
        lease_token = uuid.uuid4()
        job.runner = runner
        job.state = "staging"
        job.started_at = job.started_at or now
        job.attempt_count += 1
        job.lease_token = lease_token
        job.lease_expires_at = now + timedelta(seconds=settings.VALIDATION_RUNNER_LEASE_SECONDS)
        job.error_code = ""
        job.error_message = ""
        job.save()
        ValidationAttempt.objects.create(
            job=job,
            number=job.attempt_count,
            runner=runner,
            state="staging",
            lease_token=lease_token,
        )
        runner.state = "busy"
        runner.save(update_fields=["last_seen_at", "state", "updated_at"])
        return job

    @staticmethod
    def _validate_lease(job: ValidationJob, runner: ValidationRunner, lease_token: str) -> None:
        if job.runner_id != runner.id or str(job.lease_token or "") != str(lease_token or ""):
            raise ValidationStateError("作业租约不匹配")
        if job.state in ValidationJob.TERMINAL_STATES:
            raise ValidationStateError("作业已结束")

    @classmethod
    @transaction.atomic
    def heartbeat(cls, *, job_id, runner: ValidationRunner, payload: dict) -> ValidationJob:
        job = ValidationJob.objects.select_for_update().get(id=job_id)
        cls._validate_lease(job, runner, payload.get("lease_token", ""))
        state = str(payload.get("state") or job.state)
        if state not in {"staging", "running", "analyzing", "uploading", "cancelling"}:
            raise ValidationStateError("Runner 上报了非法作业状态")
        progress = int(payload.get("progress_percent", job.progress_percent))
        if progress < job.progress_percent or progress > 100:
            raise ValidationStateError("作业进度必须单调且不超过 100")
        job.state = "cancelling" if job.state == "cancelling" else state
        job.progress_percent = progress
        job.lease_expires_at = timezone.now() + timedelta(seconds=settings.VALIDATION_RUNNER_LEASE_SECONDS)
        job.live_bridge_url = str(payload.get("live_bridge_url") or job.live_bridge_url)[:512]
        job.stack_git_sha = str(payload.get("stack_git_sha") or job.stack_git_sha)[:64]
        job.stack_image_digest = str(payload.get("stack_image_digest") or job.stack_image_digest)[:160]
        job.save()
        runner.last_seen_at = timezone.now()
        runner.state = "busy"
        runner.save(update_fields=["last_seen_at", "state", "updated_at"])
        attempt = job.attempts.get(number=job.attempt_count)
        attempt.state = job.state
        attempt.runtime_metrics = payload.get("runtime_metrics") or attempt.runtime_metrics
        attempt.save(update_fields=["state", "runtime_metrics", "updated_at"])
        return job

    @classmethod
    @transaction.atomic
    def complete(cls, *, job_id, runner: ValidationRunner, payload: dict) -> ValidationJob:
        job = ValidationJob.objects.select_for_update().get(id=job_id)
        cls._validate_lease(job, runner, payload.get("lease_token", ""))
        outcome = str(payload.get("outcome") or "completed")
        now = timezone.now()
        attempt = job.attempts.get(number=job.attempt_count)
        if outcome == "cancelled" or job.state == "cancelling":
            job.state = "cancelled"
            job.verdict = "NOT_EVALUATED"
            attempt.state = "cancelled"
        elif outcome == "infra_error":
            attempt.state = "infra_error"
            attempt.error_code = str(payload.get("error_code") or "RUNNER_FAILED")[:64]
            attempt.error_message = str(payload.get("error_message") or "Runner 执行失败")
            if job.attempt_count < settings.VALIDATION_MAX_ATTEMPTS:
                job.state = "queued"
                job.runner = None
                job.lease_token = None
                job.lease_expires_at = None
                job.progress_percent = 0
                attempt.finished_at = now
                attempt.save()
                runner.state = "idle"
                runner.last_seen_at = now
                runner.save(update_fields=["state", "last_seen_at", "updated_at"])
                job.save()
                return job
            job.state = "infra_error"
            job.verdict = "NOT_EVALUATED"
            job.error_code = attempt.error_code
            job.error_message = attempt.error_message
        else:
            checks = payload.get("checks") or []
            if not isinstance(checks, list) or len(checks) > 500:
                raise ValidationStateError("Runner 检查结果必须是最多 500 项的列表")
            allowed_statuses = {"PASS", "WARN", "FAIL", "NOT_EVALUATED"}
            seen_rule_ids = set()
            normalized_checks = []
            for raw in checks:
                if not isinstance(raw, dict):
                    raise ValidationStateError("Runner 检查结果项格式无效")
                rule_id = str(raw.get("rule_id") or "unknown")[:128]
                check_status = str(raw.get("status") or "NOT_EVALUATED").upper()
                if check_status not in allowed_statuses:
                    raise ValidationStateError("Runner 上报了非法检查状态")
                if rule_id in seen_rule_ids:
                    raise ValidationStateError("Runner 上报了重复 rule_id")
                seen_rule_ids.add(rule_id)
                normalized_checks.append((raw, rule_id, check_status))
            ValidationCheckResult.objects.filter(job=job).delete()
            for raw, rule_id, check_status in normalized_checks:
                ValidationCheckResult.objects.create(
                    job=job,
                    rule_id=rule_id,
                    title=str(raw.get("title") or raw.get("rule_id") or "未命名检查")[:256],
                    category=str(raw.get("category") or "general")[:64],
                    status=check_status,
                    severity=str(raw.get("severity") or "error")[:16],
                    hard_failure=bool(raw.get("hard_failure", False)),
                    metric_name=str(raw.get("metric_name") or "")[:128],
                    actual_value=raw.get("actual_value"),
                    expected_value=raw.get("expected_value"),
                    start_time_ns=raw.get("start_time_ns"),
                    end_time_ns=raw.get("end_time_ns"),
                    topics=raw.get("topics") or [],
                    evidence=raw.get("evidence") or {},
                    message=str(raw.get("message") or ""),
                )
            job.state = "completed"
            job.verdict = compute_verdict(checks, has_baseline=bool(job.baseline_job_id))
            job.progress_percent = 100
            job.summary = payload.get("summary") or {}
            job.stack_git_sha = str(payload.get("stack_git_sha") or job.stack_git_sha)[:64]
            job.stack_image_digest = str(payload.get("stack_image_digest") or job.stack_image_digest)[:160]
            attempt.state = "completed"
            attempt.runtime_metrics = payload.get("runtime_metrics") or {}
        job.finished_at = now if job.state in ValidationJob.TERMINAL_STATES else None
        job.lease_expires_at = None if job.state in ValidationJob.TERMINAL_STATES else job.lease_expires_at
        job.save()
        attempt.finished_at = now
        attempt.save()
        runner.state = "idle"
        runner.last_seen_at = now
        runner.save(update_fields=["state", "last_seen_at", "updated_at"])
        return job

    @classmethod
    @transaction.atomic
    def request_cancel(cls, job: ValidationJob) -> ValidationJob:
        job = ValidationJob.objects.select_for_update().get(id=job.id)
        if job.state in ValidationJob.TERMINAL_STATES:
            return job
        now = timezone.now()
        if job.state == "queued":
            job.state = "cancelled"
            job.finished_at = now
            job.verdict = "NOT_EVALUATED"
        else:
            job.state = "cancelling"
            job.cancel_requested_at = now
        job.save()
        return job

    @classmethod
    @transaction.atomic
    def approve_baseline(cls, job: ValidationJob, user) -> GoldenBaseline:
        job = ValidationJob.objects.select_for_update().get(id=job.id)
        if job.state != "completed" or job.verdict == "FAIL":
            raise ValidationStateError("只有未出现失败项的已完成作业才能批准为黄金基线")
        config = job.requested_config or {}
        GoldenBaseline.objects.filter(
            profile=job.profile,
            map_hash=str(config.get("map_hash") or ""),
            route_hash=str(config.get("route_hash") or ""),
            active=True,
        ).update(active=False)
        baseline, _ = GoldenBaseline.objects.update_or_create(
            job=job,
            defaults={
                "profile": job.profile,
                "map_hash": str(config.get("map_hash") or "")[:64],
                "route_hash": str(config.get("route_hash") or "")[:64],
                "stack_image_digest": job.stack_image_digest or "unversioned",
                "active": True,
                "approved_by": user,
            },
        )
        return baseline
