from __future__ import annotations

from django.core.management.base import BaseCommand

from django.db.models import Q

from monitoring.models import RemoteCommand, TaskExecution
from monitoring.services.command_service import CommandService


class Command(BaseCommand):
    help = "Reconcile task.start commands with terminal task executions in bounded batches."

    def add_arguments(self, parser):
        parser.add_argument("--batch-size", type=int, default=500)
        parser.add_argument("--dry-run", action="store_true")

    def handle(self, *args, **options):
        batch_size = max(1, int(options["batch_size"]))
        dry_run = bool(options["dry_run"])
        terminal_states = tuple(CommandService.EXECUTION_COMMAND_STATUS)
        aligned = Q()
        for execution_state, command_status in CommandService.EXECUTION_COMMAND_STATUS.items():
            aligned |= Q(task_execution__state=execution_state, status=command_status)
        mismatched = (
            RemoteCommand.objects.filter(
                command_type="task.start",
                task_execution__state__in=terminal_states,
            )
            .exclude(aligned)
            .order_by("issued_at")
        )
        candidate_ids = list(
            mismatched.values_list("task_execution_id", flat=True).distinct()[:batch_size]
        )
        changed = 0
        for execution in TaskExecution.objects.filter(pk__in=candidate_ids).iterator():
            expected = CommandService.EXECUTION_COMMAND_STATUS[execution.state]
            needs_change = execution.commands.filter(command_type="task.start").exclude(
                status=expected
            ).exists()
            if not needs_change:
                continue
            if dry_run:
                changed += 1
                self.stdout.write(f"would reconcile execution={execution.id} state={execution.state}")
            else:
                changed += CommandService.reconcile_task_start_for_execution(
                    execution,
                    source="historical_repair",
                )
        self.stdout.write(
            self.style.SUCCESS(
                f"{'would change' if dry_run else 'changed'}={changed} candidates={len(candidate_ids)}"
            )
        )
