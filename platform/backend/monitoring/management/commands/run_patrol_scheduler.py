from __future__ import annotations

import logging
import time

from django.conf import settings
from django.core.management.base import BaseCommand
from django.utils import timezone

from monitoring.services.retention_service import (
    InboundMessageRetentionService,
    SystemLogRetentionService,
    weekly_cleanup_due,
)
from monitoring.services.schedule_service import ScheduleService

LOGGER = logging.getLogger(__name__)


class Command(BaseCommand):
    help = "Run patrol calendar scheduler."

    def add_arguments(self, parser):
        parser.add_argument("--interval", type=int, default=10, help="Scan interval in seconds.")
        parser.add_argument("--once", action="store_true", help="Run one scan and exit.")

    def handle(self, *args, **options):
        interval = max(1, int(options["interval"]))
        next_retention_run = timezone.now()
        last_weekly_run = None
        while True:
            now = timezone.now()
            planned_minute = ScheduleService.local_minute()
            try:
                runs = ScheduleService.dispatch_due(planned_minute)
                if runs:
                    self.stdout.write(
                        f"{timezone.localtime()} dispatched calendar runs: {', '.join(str(run.id) for run in runs)}"
                    )
            except Exception:
                LOGGER.exception("patrol scheduler scan failed")
            if now >= next_retention_run:
                self._run_hourly_retention(now)
                next_retention_run = now + timezone.timedelta(
                    seconds=getattr(settings, "INBOUND_MESSAGE_CLEANUP_INTERVAL_SECONDS", 3600)
                )
            if weekly_cleanup_due(now=now, last_run=last_weekly_run):
                self._run_weekly_retention(now)
                last_weekly_run = now
            if options["once"]:
                return
            time.sleep(interval)

    def _run_hourly_retention(self, now) -> None:
        try:
            result = InboundMessageRetentionService.prune_once(now=now)
            if result.deleted:
                LOGGER.info(
                    "pruned MQTT inbound packets: processed_or_ignored=%s failed=%s",
                    result.processed_deleted,
                    result.failed_deleted,
                )
        except Exception:
            LOGGER.exception("inbound message retention scan failed")
        try:
            log_result = SystemLogRetentionService.prune_once(now=now)
            if log_result.deleted:
                LOGGER.info(
                    "pruned system logs: debug=%s info=%s warning_error=%s",
                    log_result.debug_deleted,
                    log_result.info_deleted,
                    log_result.warning_error_deleted,
                )
        except Exception:
            LOGGER.exception("system log retention scan failed")

    def _run_weekly_retention(self, now) -> None:
        try:
            result, batches = InboundMessageRetentionService.prune_until_done(
                now=now,
                expire_stale_pending=True,
                checkpoint=True,
            )
            LOGGER.info(
                "weekly MQTT inbound drain: processed_or_ignored=%s failed=%s "
                "stale_pending=%s batches=%s",
                result.processed_deleted,
                result.failed_deleted,
                result.stale_pending_expired,
                batches,
            )
        except Exception:
            LOGGER.exception("weekly inbound message retention drain failed")
