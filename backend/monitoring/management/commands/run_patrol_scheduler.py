from __future__ import annotations

import logging
import time

from django.core.management.base import BaseCommand
from django.utils import timezone

from monitoring.services.schedule_service import ScheduleService

LOGGER = logging.getLogger(__name__)


class Command(BaseCommand):
    help = "Run patrol calendar scheduler."

    def add_arguments(self, parser):
        parser.add_argument("--interval", type=int, default=10, help="Scan interval in seconds.")
        parser.add_argument("--once", action="store_true", help="Run one scan and exit.")

    def handle(self, *args, **options):
        interval = max(1, int(options["interval"]))
        while True:
            planned_minute = ScheduleService.local_minute()
            try:
                runs = ScheduleService.dispatch_due(planned_minute)
                if runs:
                    self.stdout.write(
                        f"{timezone.localtime()} dispatched calendar runs: {', '.join(str(run.id) for run in runs)}"
                    )
            except Exception:
                LOGGER.exception("patrol scheduler scan failed")
            if options["once"]:
                return
            time.sleep(interval)
