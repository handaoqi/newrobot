from __future__ import annotations

import uuid
from datetime import datetime
from decimal import Decimal
from zoneinfo import ZoneInfo

from django.test import SimpleTestCase, TestCase, override_settings
from django.utils import timezone

from .message_handlers import _telemetry_audit_payload
from .models import (
    InboundMessage,
    MapData,
    PatrolRoute,
    PatrolTask,
    Robot,
    RobotTelemetry,
    RobotTelemetryDailySummary,
    SystemLog,
    TaskExecution,
    TrajectoryPoint,
)
from .protocol import parse_message
from .services.retention_service import (
    InboundMessageRetentionService,
    RobotTelemetryRetentionService,
    SystemLogRetentionService,
    TaskKeyframeRetentionService,
    weekly_cleanup_due,
)


@override_settings(
    INBOUND_MESSAGE_RETENTION_DAYS=30,
    INBOUND_MESSAGE_TELEMETRY_RETENTION_DAYS=30,
    INBOUND_MESSAGE_OPERATIONAL_RETENTION_DAYS=30,
    INBOUND_MESSAGE_FAILED_RETENTION_DAYS=180,
    INBOUND_MESSAGE_CLEANUP_BATCH_SIZE=2,
    INBOUND_MESSAGE_STALE_PENDING_DAYS=7,
)
class InboundMessageRetentionTests(TestCase):
    def setUp(self):
        self.robot = Robot.objects.create(code="retention-rx", name="Retention RX", location="site", area="site")
        self.now = timezone.now()

    def message(self, *, status: str, age_days: int, message_type: str = "telemetry.status"):
        return InboundMessage.objects.create(
            message_id=uuid.uuid4(),
            robot=self.robot,
            session_id=uuid.uuid4(),
            message_type=message_type,
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
        INBOUND_MESSAGE_TELEMETRY_RETENTION_DAYS=3,
        INBOUND_MESSAGE_OPERATIONAL_RETENTION_DAYS=30,
    )
    def test_uses_short_status_and_long_operational_windows(self):
        old_status = self.message(status="processed", age_days=4)
        retained_event = self.message(status="processed", age_days=4, message_type="task.progress")
        old_event = self.message(status="processed", age_days=31, message_type="alert.event")

        result = InboundMessageRetentionService.prune_once(now=self.now, batch_size=10)

        self.assertEqual(result.telemetry_deleted, 1)
        self.assertEqual(result.operational_deleted, 1)
        self.assertFalse(InboundMessage.objects.filter(pk__in=[old_status.pk, old_event.pk]).exists())
        self.assertTrue(InboundMessage.objects.filter(pk=retained_event.pk).exists())


@override_settings(INBOUND_TELEMETRY_FULL_PAYLOAD_SAMPLE_EVERY=150)
class TelemetryAuditPayloadTests(SimpleTestCase):
    def envelope(self, message_id: int):
        value = {
            "protocol_version": "1.0",
            "message_id": str(uuid.UUID(int=message_id)),
            "message_type": "telemetry.status",
            "robot_id": "retention-rx",
            "session_id": str(uuid.uuid4()),
            "sent_at": "2026-09-11T06:00:00Z",
            "trace_id": str(uuid.uuid4()),
            "sequence": 1,
            "payload": {
                "sampled_at": "2026-09-11T06:00:00Z",
                "state_version": 1,
                "pose": {"x": 1, "y": 2},
                "localization": {"status": "healthy", "raw_rtk": {"large": "value"}},
                "mapping": {"state": "idle", "post_save_validation": {"large": "value"}},
                "navigation": {"actual_forward_speed_mps": 0.1, "global_plan": [1, 2, 3]},
                "sensors": {"lidar": {"healthy": True}},
                "power_mode": {"mode": "normal", "services": {"large": "value"}},
            },
        }
        return parse_message(value)

    def test_compact_status_removes_large_repeated_sections(self):
        archived = _telemetry_audit_payload(self.envelope(1))

        self.assertEqual(archived["_archive"]["payload_mode"], "compact")
        self.assertNotIn("post_save_validation", archived["payload"]["mapping"])
        self.assertNotIn("raw_rtk", archived["payload"]["localization"])
        self.assertNotIn("global_plan", archived["payload"]["navigation"])
        self.assertNotIn("services", archived["payload"]["power_mode"])

    def test_deterministic_sample_and_failure_keep_full_payload(self):
        sampled = _telemetry_audit_payload(self.envelope(0))
        failed = _telemetry_audit_payload(self.envelope(1), preserve_full=True)

        self.assertEqual(sampled["_archive"]["payload_mode"], "full_sample")
        self.assertIn("post_save_validation", sampled["payload"]["mapping"])
        self.assertNotIn("_archive", failed)
        self.assertIn("post_save_validation", failed["payload"]["mapping"])


@override_settings(ROBOT_TELEMETRY_RETENTION_DAYS=14, ROBOT_TELEMETRY_CLEANUP_DAYS_PER_RUN=1)
class RobotTelemetryRetentionTests(TestCase):
    def setUp(self):
        self.robot = Robot.objects.create(code="telemetry-retention-rx", name="Telemetry Retention RX")
        self.now = datetime(2026, 9, 11, 14, 0, tzinfo=ZoneInfo("Asia/Shanghai"))

    def telemetry(self, sequence: int, reported_at, latitude, longitude):
        return RobotTelemetry.objects.create(
            robot=self.robot,
            sequence_id=str(sequence),
            position_name="site",
            latitude=Decimal(str(latitude)),
            longitude=Decimal(str(longitude)),
            reported_at=reported_at,
        )

    def test_archives_complete_day_before_deleting_details(self):
        old_day = self.now - timezone.timedelta(days=16)
        first = self.telemetry(1, old_day.replace(hour=9, minute=0), 31.200000, 121.500000)
        second = self.telemetry(2, old_day.replace(hour=9, minute=10), 31.201000, 121.500000)
        recent = self.telemetry(3, self.now - timezone.timedelta(days=2), 31.202000, 121.500000)

        result = RobotTelemetryRetentionService.prune_once(now=self.now)

        self.assertEqual(result.details_deleted, 2)
        self.assertEqual(result.summaries_written, 1)
        self.assertFalse(RobotTelemetry.objects.filter(pk__in=[first.pk, second.pk]).exists())
        self.assertTrue(RobotTelemetry.objects.filter(pk=recent.pk).exists())
        summary = RobotTelemetryDailySummary.objects.get(robot=self.robot, day=old_day.date())
        self.assertEqual(summary.sample_count, 2)
        self.assertEqual(summary.active_seconds, 600)
        self.assertGreater(summary.distance_km, 0)

    def test_dry_run_does_not_create_summary_or_delete(self):
        self.telemetry(1, self.now - timezone.timedelta(days=20), 31.2, 121.5)

        result = RobotTelemetryRetentionService.prune_once(now=self.now, dry_run=True)

        self.assertEqual(result.details_deleted, 1)
        self.assertEqual(RobotTelemetry.objects.count(), 1)
        self.assertFalse(RobotTelemetryDailySummary.objects.exists())


@override_settings(TASK_KEYFRAME_RETENTION_DAYS=14, TASK_KEYFRAME_CLEANUP_BATCH_SIZE=2)
class TaskKeyframeRetentionTests(TestCase):
    def setUp(self):
        self.robot = Robot.objects.create(code="task-kf-rx", name="Task keyframe RX")
        self.map_data = MapData.objects.create(name="task keyframe map", robot=self.robot)
        self.route = PatrolRoute.objects.create(name="task keyframe route", map_data=self.map_data, robot=self.robot)
        self.now = timezone.now()

    def execution(self, *, state: str):
        task = PatrolTask.objects.create(
            name=f"task keyframe {state}",
            robot=self.robot,
            route=self.route,
            route_name=self.route.name,
            scheduled_start=self.now,
            scheduled_end=self.now + timezone.timedelta(hours=1),
        )
        return TaskExecution.objects.create(task=task, robot=self.robot, route=self.route, map_data=self.map_data, state=state)

    def point(self, execution, *, seq: int, age_days: int):
        return TrajectoryPoint.objects.create(
            robot=self.robot,
            task_execution=execution,
            seq=seq,
            sampled_at=self.now - timezone.timedelta(days=age_days),
            x=Decimal("1.0"), y=Decimal("2.0"), yaw=Decimal("0.0"),
            speed_mps=Decimal("0.0"), localization_status="normal", batch_id=uuid.uuid4(),
            keyframe={"ndt": {"matching_error": 0.01}},
        )

    def test_prunes_old_completed_rows_but_keeps_recent_and_active_execution(self):
        old_completed = self.point(self.execution(state="completed"), seq=0, age_days=15)
        active = self.point(self.execution(state="running"), seq=0, age_days=15)
        recent = self.point(self.execution(state="failed"), seq=0, age_days=13)

        result = TaskKeyframeRetentionService.prune_once(now=self.now)

        self.assertEqual(result.deleted, 1)
        self.assertFalse(TrajectoryPoint.objects.filter(pk=old_completed.pk).exists())
        self.assertTrue(TrajectoryPoint.objects.filter(pk=active.pk).exists())
        self.assertTrue(TrajectoryPoint.objects.filter(pk=recent.pk).exists())

    def test_dry_run_does_not_delete_rows(self):
        old = self.point(self.execution(state="failed"), seq=0, age_days=15)

        result = TaskKeyframeRetentionService.prune_once(now=self.now, dry_run=True)

        self.assertEqual(result.deleted, 1)
        self.assertTrue(TrajectoryPoint.objects.filter(pk=old.pk).exists())


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
