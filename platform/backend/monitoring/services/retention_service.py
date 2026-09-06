"""Bounded retention jobs for high-volume operational records."""

from __future__ import annotations

from dataclasses import dataclass

from django.conf import settings
from django.db import transaction
from django.utils import timezone

from ..models import InboundMessage, SystemLog


@dataclass(frozen=True)
class InboundMessagePruneResult:
    processed_deleted: int = 0
    failed_deleted: int = 0

    @property
    def deleted(self) -> int:
        return self.processed_deleted + self.failed_deleted


class InboundMessageRetentionService:
    """Delete old, terminal MQTT audit messages in small SQLite-safe batches.

    ``InboundMessage`` keeps a complete packet only for de-duplication and
    short-term forensic review.  Its business effects have already been
    materialised into telemetry, task, command, and alert tables, so processed
    packets must not accumulate forever.  Failed packets use a longer window;
    pending packets are never touched by this job.
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
        )

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
        with transaction.atomic():
            message_ids = list(queryset.values_list("message_id", flat=True)[:batch_size])
            if not message_ids:
                return 0
            deleted, _ = InboundMessage.objects.filter(message_id__in=message_ids).delete()
        return deleted


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
            with transaction.atomic():
                ids = list(queryset.values_list("id", flat=True)[:batch_size])
                deleted, _ = SystemLog.objects.filter(id__in=ids).delete() if ids else (0, {})
            counts.append(deleted)
        return SystemLogPruneResult(*counts)
