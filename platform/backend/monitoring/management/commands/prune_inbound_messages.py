from __future__ import annotations

from django.core.management.base import BaseCommand

from monitoring.services.retention_service import InboundMessagePruneResult, InboundMessageRetentionService


class Command(BaseCommand):
    help = "Prune old terminal MQTT inbound messages in bounded batches."

    def add_arguments(self, parser):
        parser.add_argument("--retention-days", type=int, help="Processed/ignored packet retention; 0 disables.")
        parser.add_argument("--telemetry-retention-days", type=int, help="telemetry.status retention; 0 disables.")
        parser.add_argument("--operational-retention-days", type=int, help="Other terminal packet retention; 0 disables.")
        parser.add_argument("--failed-retention-days", type=int, help="Failed packet retention; 0 disables.")
        parser.add_argument("--batch-size", type=int, help="Maximum rows per status group in one batch.")
        parser.add_argument("--max-batches", type=int, default=1, help="Number of bounded batches to run.")
        parser.add_argument(
            "--until-done",
            action="store_true",
            help="Keep pruning until no eligible rows remain or the time budget expires.",
        )
        parser.add_argument("--time-budget-seconds", type=float, help="Stop an --until-done drain after this many seconds.")
        parser.add_argument(
            "--expire-stale-pending",
            action="store_true",
            help="Mark old pending packets failed before pruning.",
        )
        parser.add_argument("--dry-run", action="store_true", help="Report eligible rows without deleting them.")

    def handle(self, *args, **options):
        kwargs = {
            "retention_days": options["retention_days"],
            "telemetry_retention_days": options["telemetry_retention_days"],
            "operational_retention_days": options["operational_retention_days"],
            "failed_retention_days": options["failed_retention_days"],
            "batch_size": options["batch_size"],
            "dry_run": bool(options["dry_run"]),
            "expire_stale_pending": bool(options["expire_stale_pending"]),
        }
        if options["until_done"]:
            result, batches = InboundMessageRetentionService.prune_until_done(
                max_batches=None if options["max_batches"] == 1 else options["max_batches"],
                time_budget_seconds=options["time_budget_seconds"],
                checkpoint=not options["dry_run"],
                **kwargs,
            )
        else:
            batches = max(1, int(options["max_batches"]))
            totals = InboundMessagePruneResult()
            ran = 0
            for _ in range(batches):
                result = InboundMessageRetentionService.prune_once(**kwargs)
                totals = totals.plus(result)
                ran += 1
                if options["dry_run"] or result.deleted == 0:
                    break
            result, batches = totals, ran
        mode = "eligible" if options["dry_run"] else "deleted"
        self.stdout.write(
            f"Inbound message retention {mode}: telemetry_status={result.telemetry_deleted}, "
            f"operational={result.operational_deleted}, processed_or_ignored={result.processed_deleted}, "
            f"failed={result.failed_deleted}, stale_pending={result.stale_pending_expired}, batches={batches}"
        )
