from django.core.management.base import BaseCommand

from monitoring.services.retention_service import SystemLogRetentionService


class Command(BaseCommand):
    help = "Prune structured system logs using level-specific retention windows."

    def add_arguments(self, parser):
        parser.add_argument("--batch-size", type=int)
        parser.add_argument("--max-batches", type=int, default=1)
        parser.add_argument("--dry-run", action="store_true")

    def handle(self, *args, **options):
        totals = [0, 0, 0]
        for _ in range(max(1, options["max_batches"])):
            result = SystemLogRetentionService.prune_once(
                batch_size=options["batch_size"], dry_run=options["dry_run"],
            )
            totals[0] += result.debug_deleted
            totals[1] += result.info_deleted
            totals[2] += result.warning_error_deleted
            if options["dry_run"] or result.deleted == 0:
                break
        self.stdout.write(
            f"System log retention {'eligible' if options['dry_run'] else 'deleted'}: "
            f"debug={totals[0]}, info={totals[1]}, warning_error={totals[2]}"
        )
