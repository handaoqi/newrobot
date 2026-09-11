from __future__ import annotations

from django.core.management.base import BaseCommand

from monitoring.services.retention_service import RobotTelemetryPruneResult, RobotTelemetryRetentionService


class Command(BaseCommand):
    help = "Archive daily robot telemetry summaries and prune expired detail rows."

    def add_arguments(self, parser):
        parser.add_argument("--retention-days", type=int, help="Detailed telemetry retention window.")
        parser.add_argument("--max-days", type=int, default=1, help="Maximum robot/day groups to process.")
        parser.add_argument("--until-done", action="store_true", help="Drain all eligible robot/day groups.")
        parser.add_argument("--time-budget-seconds", type=float, help="Stop a drain after this many seconds.")
        parser.add_argument("--dry-run", action="store_true", help="Count eligible detail rows without changing data.")

    def handle(self, *args, **options):
        kwargs = {
            "retention_days": options["retention_days"],
            "dry_run": bool(options["dry_run"]),
        }
        if options["until_done"]:
            result, batches = RobotTelemetryRetentionService.prune_until_done(
                max_days=None if options["max_days"] == 1 else options["max_days"],
                time_budget_seconds=options["time_budget_seconds"],
                checkpoint=not options["dry_run"],
                **kwargs,
            )
        else:
            result = RobotTelemetryPruneResult()
            batches = 0
            for _ in range(max(1, int(options["max_days"]))):
                batch = RobotTelemetryRetentionService.prune_once(max_days=1, **kwargs)
                result = result.plus(batch)
                batches += 1
                if options["dry_run"] or batch.details_deleted == 0:
                    break
        mode = "eligible" if options["dry_run"] else "deleted"
        self.stdout.write(
            f"Robot telemetry retention {mode}: details={result.details_deleted}, "
            f"daily_summaries={result.summaries_written}, batches={batches}"
        )
