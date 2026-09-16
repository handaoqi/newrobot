"""Bounded retention jobs for high-volume operational records."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from datetime import datetime, time as datetime_time, timedelta
from decimal import Decimal

from django.conf import settings
from django.db import transaction
from django.utils import timezone

from ..models import (
    InboundMessage,
    RobotTelemetry,
    RobotTelemetryDailySummary,
    SystemLog,
    TaskExecution,
    TrajectoryPoint,
)
from .sqlite_retry import checkpoint_sqlite_wal, with_sqlite_lock_retry


@dataclass(frozen=True)
class InboundMessagePruneResult:
    processed_deleted: int = 0
    failed_deleted: int = 0
    stale_pending_expired: int = 0
    telemetry_deleted: int = 0
    operational_deleted: int = 0

    @property
    def deleted(self) -> int:
        return self.processed_deleted + self.failed_deleted

    def plus(self, other: "InboundMessagePruneResult") -> "InboundMessagePruneResult":
        return InboundMessagePruneResult(
            processed_deleted=self.processed_deleted + other.processed_deleted,
            failed_deleted=self.failed_deleted + other.failed_deleted,
            stale_pending_expired=self.stale_pending_expired + other.stale_pending_expired,
            telemetry_deleted=self.telemetry_deleted + other.telemetry_deleted,
            operational_deleted=self.operational_deleted + other.operational_deleted,
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
        telemetry_retention_days: int | None = None,
        operational_retention_days: int | None = None,
        failed_retention_days: int | None = None,
        batch_size: int | None = None,
        dry_run: bool = False,
        expire_stale_pending: bool = False,
    ) -> InboundMessagePruneResult:
        now = now or timezone.now()
        # ``retention_days`` remains a command/API compatibility override.  In
        # normal scheduled runs, high-volume status packets and lower-volume
        # operational events use separate windows.
        if telemetry_retention_days is None:
            telemetry_retention_days = (
                int(retention_days)
                if retention_days is not None
                else cls._days(None, "INBOUND_MESSAGE_TELEMETRY_RETENTION_DAYS")
            )
        if operational_retention_days is None:
            operational_retention_days = (
                int(retention_days)
                if retention_days is not None
                else cls._days(None, "INBOUND_MESSAGE_OPERATIONAL_RETENTION_DAYS")
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

        telemetry_deleted = cls._prune_statuses(
            statuses=cls.TERMINAL_STATUSES,
            cutoff=cls._cutoff(now, telemetry_retention_days),
            batch_size=batch_size,
            dry_run=dry_run,
            message_types=("telemetry.status",),
        )
        operational_deleted = cls._prune_statuses(
            statuses=cls.TERMINAL_STATUSES,
            cutoff=cls._cutoff(now, operational_retention_days),
            batch_size=batch_size,
            dry_run=dry_run,
            exclude_message_types=("telemetry.status",),
        )
        failed_deleted = cls._prune_statuses(
            statuses=("failed",),
            cutoff=cls._cutoff(now, failed_retention_days),
            batch_size=batch_size,
            dry_run=dry_run,
        )
        return InboundMessagePruneResult(
            processed_deleted=telemetry_deleted + operational_deleted,
            failed_deleted=failed_deleted,
            stale_pending_expired=stale_pending_expired,
            telemetry_deleted=telemetry_deleted,
            operational_deleted=operational_deleted,
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
    def _prune_statuses(
        *,
        statuses: tuple[str, ...],
        cutoff,
        batch_size: int,
        dry_run: bool,
        message_types: tuple[str, ...] | None = None,
        exclude_message_types: tuple[str, ...] | None = None,
    ) -> int:
        if cutoff is None:
            return 0
        queryset = InboundMessage.objects.filter(
            process_status__in=statuses,
            received_at__lt=cutoff,
        )
        if message_types:
            queryset = queryset.filter(message_type__in=message_types)
        if exclude_message_types:
            queryset = queryset.exclude(message_type__in=exclude_message_types)
        queryset = queryset.order_by("received_at")
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
class RobotTelemetryPruneResult:
    details_deleted: int = 0
    summaries_written: int = 0

    def plus(self, other: "RobotTelemetryPruneResult") -> "RobotTelemetryPruneResult":
        return RobotTelemetryPruneResult(
            details_deleted=self.details_deleted + other.details_deleted,
            summaries_written=self.summaries_written + other.summaries_written,
        )


class RobotTelemetryRetentionService:
    """Archive completed local days, then remove their detailed telemetry."""

    @classmethod
    def prune_once(
        cls,
        *,
        now=None,
        retention_days: int | None = None,
        max_days: int | None = None,
        dry_run: bool = False,
    ) -> RobotTelemetryPruneResult:
        now = now or timezone.now()
        retention_days = max(
            1,
            int(
                retention_days
                if retention_days is not None
                else getattr(settings, "ROBOT_TELEMETRY_RETENTION_DAYS", 14)
            ),
        )
        max_days = max(
            1,
            int(
                max_days
                if max_days is not None
                else getattr(settings, "ROBOT_TELEMETRY_CLEANUP_DAYS_PER_RUN", 1)
            ),
        )
        local_tz = timezone.get_current_timezone()
        cutoff_day = timezone.localdate(now) - timedelta(days=retention_days)
        cutoff = timezone.make_aware(datetime.combine(cutoff_day, datetime_time.min), local_tz)
        eligible = RobotTelemetry.objects.filter(reported_at__lt=cutoff)
        if dry_run:
            return RobotTelemetryPruneResult(details_deleted=eligible.count())

        total = RobotTelemetryPruneResult()
        for _ in range(max_days):
            candidate = eligible.order_by("reported_at").values_list("robot_id", "reported_at").first()
            if candidate is None:
                break
            robot_id, reported_at = candidate
            day = timezone.localtime(reported_at, local_tz).date()
            result = with_sqlite_lock_retry(
                lambda robot_id=robot_id, day=day: cls._archive_and_delete_day(
                    robot_id=robot_id,
                    day=day,
                    local_tz=local_tz,
                ),
                label="robot telemetry daily archive",
            )
            total = total.plus(result)
        return total

    @classmethod
    def prune_until_done(
        cls,
        *,
        now=None,
        max_days: int | None = None,
        time_budget_seconds: float | None = None,
        checkpoint: bool = False,
        **kwargs,
    ) -> tuple[RobotTelemetryPruneResult, int]:
        now = now or timezone.now()
        max_days = max(
            1,
            int(
                max_days
                if max_days is not None
                else getattr(settings, "ROBOT_TELEMETRY_WEEKLY_CLEANUP_MAX_DAYS", 10_000)
            ),
        )
        if time_budget_seconds is None:
            time_budget_seconds = float(
                getattr(settings, "ROBOT_TELEMETRY_WEEKLY_CLEANUP_TIME_BUDGET_SECONDS", 600)
            )
        started = time.monotonic()
        total = RobotTelemetryPruneResult()
        batches = 0
        dry_run = bool(kwargs.get("dry_run"))
        for _ in range(max_days):
            if time_budget_seconds and time.monotonic() - started >= time_budget_seconds:
                break
            result = cls.prune_once(now=now, max_days=1, **kwargs)
            batches += 1
            total = total.plus(result)
            if dry_run or result.details_deleted == 0:
                break
        if checkpoint and not dry_run:
            checkpoint_sqlite_wal()
        return total, batches

    @staticmethod
    def _haversine_km(lat1, lon1, lat2, lon2) -> float:
        radius_km = 6371.0088
        phi1, phi2 = math.radians(float(lat1)), math.radians(float(lat2))
        delta_phi = math.radians(float(lat2) - float(lat1))
        delta_lambda = math.radians(float(lon2) - float(lon1))
        value = (
            math.sin(delta_phi / 2) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        )
        return radius_km * 2 * math.atan2(math.sqrt(value), math.sqrt(max(0.0, 1 - value)))

    @classmethod
    def _archive_and_delete_day(cls, *, robot_id: int, day, local_tz) -> RobotTelemetryPruneResult:
        day_start = timezone.make_aware(datetime.combine(day, datetime_time.min), local_tz)
        day_end = day_start + timedelta(days=1)
        with transaction.atomic():
            queryset = RobotTelemetry.objects.select_for_update().filter(
                robot_id=robot_id,
                reported_at__gte=day_start,
                reported_at__lt=day_end,
            )
            rows = list(
                queryset.order_by("reported_at", "id").values_list(
                    "reported_at", "latitude", "longitude"
                )
            )
            if not rows:
                return RobotTelemetryPruneResult()

            distance_km = 0.0
            first_point = None
            last_point = None
            previous = None
            for _, latitude, longitude in rows:
                if latitude is None or longitude is None:
                    continue
                point = (latitude, longitude)
                if first_point is None:
                    first_point = point
                if previous is not None:
                    distance_km += cls._haversine_km(*previous, *point)
                previous = point
                last_point = point

            first_reported_at = rows[0][0]
            last_reported_at = rows[-1][0]
            summary = RobotTelemetryDailySummary.objects.select_for_update().filter(
                robot_id=robot_id, day=day
            ).first()
            if summary is not None:
                if summary.last_reported_at <= first_reported_at and summary.last_latitude is not None and first_point:
                    distance_km += cls._haversine_km(
                        summary.last_latitude, summary.last_longitude, *first_point
                    )
                elif last_reported_at <= summary.first_reported_at and last_point and summary.first_latitude is not None:
                    distance_km += cls._haversine_km(
                        *last_point, summary.first_latitude, summary.first_longitude
                    )
                first_is_new = first_reported_at < summary.first_reported_at
                last_is_new = last_reported_at > summary.last_reported_at
                first_reported_at = min(first_reported_at, summary.first_reported_at)
                last_reported_at = max(last_reported_at, summary.last_reported_at)
                summary.sample_count += len(rows)
                summary.first_reported_at = first_reported_at
                summary.last_reported_at = last_reported_at
                summary.active_seconds = max(0, int((last_reported_at - first_reported_at).total_seconds()))
                summary.distance_km += Decimal(str(round(distance_km, 6)))
                if first_is_new and first_point:
                    summary.first_latitude, summary.first_longitude = first_point
                if last_is_new and last_point:
                    summary.last_latitude, summary.last_longitude = last_point
                summary.save()
            else:
                RobotTelemetryDailySummary.objects.create(
                    robot_id=robot_id,
                    day=day,
                    sample_count=len(rows),
                    first_reported_at=first_reported_at,
                    last_reported_at=last_reported_at,
                    active_seconds=max(0, int((last_reported_at - first_reported_at).total_seconds())),
                    distance_km=Decimal(str(round(distance_km, 6))),
                    first_latitude=first_point[0] if first_point else None,
                    first_longitude=first_point[1] if first_point else None,
                    last_latitude=last_point[0] if last_point else None,
                    last_longitude=last_point[1] if last_point else None,
                )
            deleted, _ = queryset.delete()
            return RobotTelemetryPruneResult(details_deleted=deleted, summaries_written=1)


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


@dataclass(frozen=True)
class TaskKeyframePruneResult:
    deleted: int = 0


class TaskKeyframeRetentionService:
    """Bound recent task-position/keyframe diagnostics without touching active tasks."""

    @classmethod
    def prune_once(
        cls,
        *,
        now=None,
        retention_days: int | None = None,
        batch_size: int | None = None,
        dry_run: bool = False,
    ) -> TaskKeyframePruneResult:
        now = now or timezone.now()
        retention_days = max(
            1,
            int(
                retention_days
                if retention_days is not None
                else getattr(settings, "TASK_KEYFRAME_RETENTION_DAYS", 14)
            ),
        )
        batch_size = max(
            1,
            int(
                batch_size
                if batch_size is not None
                else getattr(settings, "TASK_KEYFRAME_CLEANUP_BATCH_SIZE", 2_000)
            ),
        )
        eligible = TrajectoryPoint.objects.filter(
            sampled_at__lt=now - timedelta(days=retention_days),
        ).exclude(task_execution__state__in=TaskExecution.ACTIVE_STATES).order_by("sampled_at")
        if dry_run:
            return TaskKeyframePruneResult(deleted=eligible.count())

        def delete_batch():
            with transaction.atomic():
                ids = list(eligible.values_list("id", flat=True)[:batch_size])
                deleted, _ = TrajectoryPoint.objects.filter(id__in=ids).delete() if ids else (0, {})
                return TaskKeyframePruneResult(deleted=deleted)

        return with_sqlite_lock_retry(delete_batch, label="task keyframe retention")
