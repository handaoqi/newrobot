"""Bounded retention jobs for high-volume operational records."""

from __future__ import annotations

import time
from dataclasses import dataclass

from django.conf import settings
from django.db import transaction
from django.utils import timezone

from ..models import InboundMessage, SystemLog
from .sqlite_retry import checkpoint_sqlite_wal, with_sqlite_lock_retry


@dataclass(frozen=True)
class InboundMessagePruneResult:
    processed_deleted: int = 0
    failed_deleted: int = 0
    stale_pending_expired: int = 0

    @property
    def deleted(self) -> int:
        return self.processed_deleted + self.failed_deleted

    def plus(self, other: "InboundMessagePruneResult") -> "InboundMessagePruneResult":
        return InboundMessagePruneResult(
            processed_deleted=self.processed_deleted + other.processed_deleted,
            failed_deleted=self.failed_deleted + other.failed_deleted,
            stale_pending_expired=self.stale_pending_expired + other.stale_pending_expired,
        )


def last_scheduled_weekly_slot(now=None):
    """Return the most recent weekly cleanup instant in the configured local TZ."""

    now = timezone.localtime(now or timezone.now())
    weekday = int(getattr(settings, "INBOUND_MESSAGE_WEEKLY_CLEANUP_WEEKDAY", 0)) % 7
    hour = min(23, max(0, int(getattr(settings, "INBOUND_MESSAGE_WEEKLY_CLEANUP_HOUR", 3))))
    candidate = now.replace(hour=hour, minute=0, second=0, microsecond=0)
    days_back = (now.weekday() - weekday) % 7
    if days_back == 0 and now < candidate:
        days_back = 7
    return candidate - timezone.timedelta(days=days_back)


def weekly_cleanup_due(now=None, last_run=None) -> bool:
    """True when the scheduler is inside this week's cleanup window and has not run it yet."""

    if not getattr(settings, "INBOUND_MESSAGE_WEEKLY_CLEANUP_ENABLED", True):
        return False
    now = now or timezone.now()
    slot = last_scheduled_weekly_slot(now)
    if now < slot:
        return False
    if last_run is not None and last_run >= slot:
        return False
    grace_hours = max(1, int(getattr(settings, "INBOUND_MESSAGE_WEEKLY_CLEANUP_GRACE_HOURS", 24)))
    return now <= slot + timezone.timedelta(hours=grace_hours)


class InboundMessageRetentionService:
    """Delete old, terminal MQTT audit messages in small SQLite-safe batches.

    ``InboundMessage`` keeps a complete packet only for de-duplication and
    short-term forensic review.  Its business effects have already been
    materialised into telemetry, task, command, and alert tables, so processed
    packets must not accumulate forever.  Failed packets use a longer window;
    pending packets are never touched by this job until they are marked stale.
    """

    TERMINAL_STATUSES = ("processed", "ignored")

    @classmethod
    def prune_once(
        cls,
        *,
        now=None,
        retention_days: int | None = None,
        failed_retention_days: int | None = None,
        batch_size: int | None = None,
        dry_run: bool = False,
        expire_stale_pending: bool = False,
    ) -> InboundMessagePruneResult:
        now = now or timezone.now()
        retention_days = cls._days(
            retention_days,
            "INBOUND_MESSAGE_RETENTION_DAYS",
        )
        failed_retention_days = cls._days(
            failed_retention_days,
            "INBOUND_MESSAGE_FAILED_RETENTION_DAYS",
        )
        batch_size = max(
            1,
            int(batch_size if batch_size is not None else getattr(
                settings, "INBOUND_MESSAGE_CLEANUP_BATCH_SIZE", 2_000
            )),
        )
        stale_pending_expired = 0
        if expire_stale_pending:
            stale_pending_expired = cls.expire_stale_pending(now=now, dry_run=dry_run)

        processed_deleted = cls._prune_statuses(
            statuses=cls.TERMINAL_STATUSES,
            cutoff=cls._cutoff(now, retention_days),
            batch_size=batch_size,
            dry_run=dry_run,
        )
        failed_deleted = cls._prune_statuses(
            statuses=("failed",),
            cutoff=cls._cutoff(now, failed_retention_days),
            batch_size=batch_size,
            dry_run=dry_run,
        )
        return InboundMessagePruneResult(
            processed_deleted=processed_deleted,
            failed_deleted=failed_deleted,
            stale_pending_expired=stale_pending_expired,
        )

    @classmethod
    def prune_until_done(
        cls,
        *,
        now=None,
        max_batches: int | None = None,
        time_budget_seconds: float | None = None,
        checkpoint: bool = False,
        **kwargs,
    ) -> tuple[InboundMessagePruneResult, int]:
        now = now or timezone.now()
        max_batches = max(
            1,
            int(max_batches if max_batches is not None else getattr(
                settings, "INBOUND_MESSAGE_WEEKLY_CLEANUP_MAX_BATCHES", 10_000
            )),
        )
        if time_budget_seconds is None:
            time_budget_seconds = float(
                getattr(settings, "INBOUND_MESSAGE_WEEKLY_CLEANUP_TIME_BUDGET_SECONDS", 600)
            )
        started = time.monotonic()
        total = InboundMessagePruneResult()
        batches = 0
        expire_stale = kwargs.pop("expire_stale_pending", True)
        dry_run = bool(kwargs.get("dry_run"))
        for index in range(max_batches):
            if time_budget_seconds and time.monotonic() - started >= time_budget_seconds:
                break
            result = cls.prune_once(
                now=now,
                expire_stale_pending=bool(expire_stale) and index == 0,
                **kwargs,
            )
            batches += 1
            total = total.plus(result)
            if dry_run or result.deleted == 0:
                break
        if checkpoint and not dry_run:
            checkpoint_sqlite_wal()
        return total, batches

    @classmethod
    def expire_stale_pending(cls, *, now=None, dry_run: bool = False) -> int:
        now = now or timezone.now()
        days = int(getattr(settings, "INBOUND_MESSAGE_STALE_PENDING_DAYS", 7))
        if days <= 0:
            return 0
        queryset = InboundMessage.objects.filter(
            process_status="pending",
            received_at__lt=now - timezone.timedelta(days=days),
        )
        if dry_run:
            return queryset.count()

        def mark_failed():
            with transaction.atomic():
                return queryset.update(
                    process_status="failed",
                    error_message="stale pending expired by retention",
                    processed_at=now,
                )

        return with_sqlite_lock_retry(mark_failed, label="inbound stale pending expiry")

    @staticmethod
    def _days(value: int | None, setting: str) -> int:
        return int(value if value is not None else getattr(settings, setting, 0))

    @staticmethod
    def _cutoff(now, days: int):
        # Zero deliberately disables that category.  It is safer than an
        # accidental "delete everything" from a missing environment variable.
        if days <= 0:
            return None
        return now - timezone.timedelta(days=days)

    @staticmethod
    def _prune_statuses(*, statuses: tuple[str, ...], cutoff, batch_size: int, dry_run: bool) -> int:
        if cutoff is None:
            return 0
        queryset = InboundMessage.objects.filter(
            process_status__in=statuses,
            received_at__lt=cutoff,
        ).order_by("received_at")
        if dry_run:
            return queryset.count()
        def delete_batch():
            with transaction.atomic():
                message_ids = list(queryset.values_list("message_id", flat=True)[:batch_size])
                if not message_ids:
                    return 0
                deleted, _ = InboundMessage.objects.filter(message_id__in=message_ids).delete()
                return deleted

        return with_sqlite_lock_retry(delete_batch, label="inbound message retention")


@dataclass(frozen=True)
class SystemLogPruneResult:
    debug_deleted: int = 0
    info_deleted: int = 0
    warning_error_deleted: int = 0

    @property
    def deleted(self) -> int:
        return self.debug_deleted + self.info_deleted + self.warning_error_deleted


class SystemLogRetentionService:
    """Apply the level-specific system-log retention policy in bounded batches."""

    @classmethod
    def prune_once(cls, *, now=None, batch_size: int | None = None, dry_run: bool = False):
        now = now or timezone.now()
        batch_size = max(1, int(batch_size or getattr(settings, "SYSTEM_LOG_CLEANUP_BATCH_SIZE", 2_000)))
        groups = (
            (("DEBUG",), int(getattr(settings, "SYSTEM_LOG_DEBUG_RETENTION_DAYS", 7))),
            (("INFO",), int(getattr(settings, "SYSTEM_LOG_INFO_RETENTION_DAYS", 30))),
            (("WARNING", "ERROR"), int(getattr(settings, "SYSTEM_LOG_WARNING_ERROR_RETENTION_DAYS", 180))),
        )
        counts = []
        for levels, days in groups:
            queryset = SystemLog.objects.filter(
                level__in=levels,
                occurred_at__lt=now - timezone.timedelta(days=max(1, days)),
            ).order_by("occurred_at")
            if dry_run:
                counts.append(queryset.count())
                continue
            def delete_batch():
                with transaction.atomic():
                    ids = list(queryset.values_list("id", flat=True)[:batch_size])
                    deleted, _ = SystemLog.objects.filter(id__in=ids).delete() if ids else (0, {})
                    return deleted

            deleted = with_sqlite_lock_retry(delete_batch, label="system log retention")
            counts.append(deleted)
        return SystemLogPruneResult(*counts)
