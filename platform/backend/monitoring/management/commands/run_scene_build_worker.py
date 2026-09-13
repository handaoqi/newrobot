from __future__ import annotations

import time

from django.core.management.base import BaseCommand
from django.db import close_old_connections

from monitoring.services.scene_build_service import claim_scene_build, run_scene_build


class Command(BaseCommand):
    help = "Run the single-concurrency server-side street-block build worker."

    def add_arguments(self, parser):
        parser.add_argument("--once", action="store_true")

    def handle(self, *args, **options):
        while True:
            close_old_connections()
            build = claim_scene_build()
            if build is not None:
                self.stdout.write(f"building scene {build.id}")
                run_scene_build(build)
                continue
            if options["once"]:
                return
            time.sleep(1)
