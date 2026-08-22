from __future__ import annotations

import os

from django.contrib.auth import get_user_model
from django.core.management.base import BaseCommand, CommandError
from django.db import transaction

from monitoring.bootstrap import initialize_platform_configuration
from monitoring.models import Robot


class Command(BaseCommand):
    help = "Initialize required platform configuration, operator, and primary robot"

    def add_arguments(self, parser):
        parser.add_argument("--skip-operator", action="store_true")
        parser.add_argument("--skip-robot", action="store_true")
        parser.add_argument("--with-demo-data", action="store_true")

    @transaction.atomic
    def handle(self, *args, **options):
        created = initialize_platform_configuration()
        self.stdout.write(
            "system configuration: "
            f"categories={created['categories']} "
            f"templates={created['templates']} "
            f"alert_skills={created['alert_skills']}"
        )
        if not options["skip_operator"]:
            self._initialize_operator()
        if not options["skip_robot"]:
            self._initialize_robot()
        if options["with_demo_data"]:
            from monitoring.views import ensure_demo_seed

            ensure_demo_seed(force=True)
            self.stdout.write(self.style.WARNING("optional demo data initialized"))
        self.stdout.write(self.style.SUCCESS("platform initialization complete"))

    def _initialize_operator(self):
        username = os.getenv("PLATFORM_OPERATOR_USERNAME", "operator").strip()
        password = os.getenv("PLATFORM_OPERATOR_PASSWORD", "")
        email = os.getenv("PLATFORM_OPERATOR_EMAIL", "").strip()
        if not username:
            raise CommandError("PLATFORM_OPERATOR_USERNAME cannot be empty")

        User = get_user_model()
        user = User.objects.filter(username=username).first()
        if user:
            self.stdout.write(f"operator exists: {username}")
            return
        if not password:
            raise CommandError(
                "PLATFORM_OPERATOR_PASSWORD is required when creating the first operator"
            )
        User.objects.create_user(username=username, password=password, email=email)
        self.stdout.write(f"operator created: {username}")

    def _initialize_robot(self):
        code = os.getenv("PLATFORM_ROBOT_CODE", "ZSL-1A-07").strip()
        if not code:
            raise CommandError("PLATFORM_ROBOT_CODE cannot be empty")
        robot, created = Robot.objects.get_or_create(
            code=code,
            defaults={
                "name": os.getenv("PLATFORM_ROBOT_NAME", code).strip() or code,
                "location": os.getenv("PLATFORM_ROBOT_LOCATION", "unassigned").strip(),
                "area": os.getenv("PLATFORM_ROBOT_AREA", "unassigned").strip(),
                "status": "offline",
                "mode": "standby",
                "battery_level": 0,
                "network_strength": 0,
                "stream_id": f"dog_{code}_front",
                "connection_status": "unknown",
            },
        )
        self.stdout.write(f"robot {'created' if created else 'exists'}: {robot.code}")
