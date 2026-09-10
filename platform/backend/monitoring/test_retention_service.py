from __future__ import annotations

import uuid
from datetime import datetime
from zoneinfo import ZoneInfo

from django.test import SimpleTestCase, TestCase, override_settings
from django.utils import timezone

from .models import InboundMessage, Robot, SystemLog
from .services.retention_service import (
    InboundMessageRetentionService,
    SystemLogRetentionService,
    weekly_cleanup_due,
)


@override_settings(
    INBOUND_MESSAGE_RETENTION_DAYS=30,
    INBOUND_MESSAGE_FAILED_RETENTION_DAYS=180,
    INBOUND_MESSAGE_CLEANUP_BATCH_SIZE=2,
    INBOUND_MESSAGE_STALE_PENDING_DAYS=7,
)
class InboundMessageRetentionTests(TestCase):
    def setUp(self):
        self.robot = Robot.objects.create(code="retention-rx", name="Retention RX", location="site", area="site")
        self.now = timezone.now()

    def message(self, *, status: str, age_days: int):
        return InboundMessage.objects.create(
            message_id=uuid.uuid4(),
            robot=self.robot,
            session_id=uuid.uuid4(),
            message_type="telemetry.status",
            topic="robots/retention-rx/telemetry",
            process_status=status,
            received_at=self.now - timezone.timedelta(days=age_days),
            raw_payload={"large": "payload"},
        )

    def test_prunes_only_old_terminal_messages_in_bounded_batches(self):
        old_processed = self.message(status="processed", age_days=31)
        old_ignored = self.message(status="ignored", age_days=31)
        retained_new = self.message(status="processed", age_days=29)
        retained_pending = self.message(status="pending", age_days=365)
        retained_failed = self.message(status="failed", age_days=179)
        old_failed = self.message(status="failed", age_days=181)

        result = InboundMessageRetentionService.prune_once(now=self.now)

        self.assertEqual(result.processed_deleted, 2)
        self.assertEqual(result.failed_deleted, 1)
        self.assertFalse(InboundMessage.objects.filter(pk=old_processed.pk).exists())
        self.assertFalse(InboundMessage.objects.filter(pk=old_ignored.pk).exists())
        self.assertFalse(InboundMessage.objects.filter(pk=old_failed.pk).exists())
        self.assertTrue(InboundMessage.objects.filter(pk=retained_new.pk).exists())
        self.assertTrue(InboundMessage.objects.filter(pk=retained_pending.pk).exists())
        self.assertTrue(InboundMessage.objects.filter(pk=retained_failed.pk).exists())

    def test_zero_retention_days_disables_a_category(self):
        retained = self.message(status="processed", age_days=365)

        result = InboundMessageRetentionService.prune_once(
            now=self.now,
            retention_days=0,
            failed_retention_days=0,
        )

        self.assertEqual(result.deleted, 0)
        self.assertTrue(InboundMessage.objects.filter(pk=retained.pk).exists())

    def test_dry_run_reports_eligible_messages_without_deleting(self):
        old_message = self.message(status="processed", age_days=31)

        result = InboundMessageRetentionService.prune_once(now=self.now, dry_run=True)

        self.assertEqual(result.processed_deleted, 1)
        self.assertTrue(InboundMessage.objects.filter(pk=old_message.pk).exists())

    def test_prune_until_done_drains_all_eligible_batches(self):
        for _ in range(5):
            self.message(status="processed", age_days=31)

        result, batches = InboundMessageRetentionService.prune_until_done(
            now=self.now,
            expire_stale_pending=False,
            checkpoint=False,
        )

        self.assertEqual(result.processed_deleted, 5)
        self.assertGreaterEqual(batches, 3)
        self.assertFalse(InboundMessage.objects.filter(process_status="processed").exists())

    def test_expire_stale_pending_marks_old_rows_failed(self):
        stale = self.message(status="pending", age_days=8)
        fresh = self.message(status="pending", age_days=1)

        count = InboundMessageRetentionService.expire_stale_pending(now=self.now)

        self.assertEqual(count, 1)
        stale.refresh_from_db()
        fresh.refresh_from_db()
        self.assertEqual(stale.process_status, "failed")
        self.assertEqual(fresh.process_status, "pending")


@override_settings(
    SYSTEM_LOG_DEBUG_RETENTION_DAYS=7,
    SYSTEM_LOG_INFO_RETENTION_DAYS=30,
    SYSTEM_LOG_WARNING_ERROR_RETENTION_DAYS=180,
    SYSTEM_LOG_CLEANUP_BATCH_SIZE=20,
)
class SystemLogRetentionTests(TestCase):
    def setUp(self):
        self.robot = Robot.objects.create(code="syslog-retention-rx", name="System Log RX")
        self.now = timezone.now()

    def log(self, level, age_days):
        return SystemLog.objects.create(
            robot=self.robot, level=level, module="system",
            event_code="retention.test", message="retention test",
            occurred_at=self.now - timezone.timedelta(days=age_days),
        )

    def test_level_specific_windows(self):
        old_debug = self.log("DEBUG", 8)
        new_debug = self.log("DEBUG", 6)
        old_info = self.log("INFO", 31)
        new_info = self.log("INFO", 29)
        old_error = self.log("ERROR", 181)
        new_warning = self.log("WARNING", 179)

        result = SystemLogRetentionService.prune_once(now=self.now)

        self.assertEqual((result.debug_deleted, result.info_deleted, result.warning_error_deleted), (1, 1, 1))
        self.assertFalse(SystemLog.objects.filter(pk__in=[old_debug.pk, old_info.pk, old_error.pk]).exists())
        self.assertEqual(SystemLog.objects.filter(pk__in=[new_debug.pk, new_info.pk, new_warning.pk]).count(), 3)


@override_settings(
    INBOUND_MESSAGE_WEEKLY_CLEANUP_ENABLED=True,
    INBOUND_MESSAGE_WEEKLY_CLEANUP_WEEKDAY=0,
    INBOUND_MESSAGE_WEEKLY_CLEANUP_HOUR=3,
    INBOUND_MESSAGE_WEEKLY_CLEANUP_GRACE_HOURS=24,
)
class WeeklyCleanupDueTests(SimpleTestCase):
    tz = ZoneInfo("Asia/Shanghai")

    def test_due_on_monday_after_scheduled_hour(self):
        now = datetime(2026, 9, 7, 4, 0, tzinfo=self.tz)
        self.assertTrue(weekly_cleanup_due(now=now, last_run=None))

    def test_not_due_before_scheduled_hour(self):
        now = datetime(2026, 9, 7, 2, 59, tzinfo=self.tz)
        self.assertFalse(weekly_cleanup_due(now=now, last_run=None))

    def test_not_due_after_grace_window(self):
        now = datetime(2026, 9, 8, 4, 1, tzinfo=self.tz)
        self.assertFalse(weekly_cleanup_due(now=now, last_run=None))

    def test_not_due_when_already_run_this_slot(self):
        now = datetime(2026, 9, 7, 10, 0, tzinfo=self.tz)
        last = datetime(2026, 9, 7, 3, 5, tzinfo=self.tz)
        self.assertFalse(weekly_cleanup_due(now=now, last_run=last))
