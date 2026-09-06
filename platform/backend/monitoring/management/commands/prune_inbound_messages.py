from __future__ import annotations

from django.core.management.base import BaseCommand

from monitoring.services.retention_service import InboundMessageRetentionService


class Command(BaseCommand):
    help = "Prune old terminal MQTT inbound messages in bounded batches."

    def add_arguments(self, parser):
        parser.add_argument("--retention-days", type=int, help="Processed/ignored packet retention; 0 disables.")
        parser.add_argument("--failed-retention-days", type=int, help="Failed packet retention; 0 disables.")
        parser.add_argument("--batch-size", type=int, help="Maximum rows per status group in one batch.")
        parser.add_argument("--max-batches", type=int, default=1, help="Number of bounded batches to run.")
        parser.add_argument("--dry-run", action="store_true", help="Report eligible rows without deleting them.")

    def handle(self, *args, **options):
        batches = max(1, int(options["max_batches"]))
        processed_deleted = failed_deleted = 0
        for _ in range(batches):
            result = InboundMessageRetentionService.prune_once(
                retention_days=options["retention_days"],
                failed_retention_days=options["failed_retention_days"],
                batch_size=options["batch_size"],
                dry_run=bool(options["dry_run"]),
            )
            processed_deleted += result.processed_deleted
            failed_deleted += result.failed_deleted
            if options["dry_run"] or result.deleted == 0:
                break
        mode = "eligible" if options["dry_run"] else "deleted"
        self.stdout.write(
            f"Inbound message retention {mode}: processed_or_ignored={processed_deleted}, "
            f"failed={failed_deleted}, batches={batches}"
        )
