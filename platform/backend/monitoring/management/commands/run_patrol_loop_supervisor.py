from __future__ import annotations

import logging
import threading

from django.core.management.base import BaseCommand

from monitoring.services.patrol_loop_service import PatrolLoopService


LOGGER = logging.getLogger(__name__)


class Command(BaseCommand):
    help = "Run the durable patrol-loop supervisor."

    def add_arguments(self, parser):
        parser.add_argument("--interval", type=float, default=1.0)
        parser.add_argument("--once", action="store_true")

    def handle(self, *args, **options):
        interval = max(0.2, float(options["interval"]))
        if options["once"]:
            count = PatrolLoopService.process_due()
            self.stdout.write(f"processed={count}")
            return
        stop_event = threading.Event()
        self.stdout.write(self.style.SUCCESS(f"patrol loop supervisor interval={interval:.1f}s"))
        try:
            while not stop_event.wait(interval):
                try:
                    PatrolLoopService.process_due()
                except Exception:
                    LOGGER.exception("patrol loop supervisor iteration failed")
        except KeyboardInterrupt:
            self.stdout.write("patrol loop supervisor stopping")
