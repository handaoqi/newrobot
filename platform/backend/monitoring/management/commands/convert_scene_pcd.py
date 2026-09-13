from __future__ import annotations

import json

from django.core.management.base import BaseCommand

from monitoring.services.scene_converter import convert_scene


class Command(BaseCommand):
    help = "Convert one PCD and optional reference media to a street-block GLB."

    def add_arguments(self, parser):
        parser.add_argument("--point-cloud", required=True)
        parser.add_argument("--output-dir", required=True)
        parser.add_argument("--references", nargs="*", default=[])
        parser.add_argument("--trajectory")
        parser.add_argument("--semantics")

    def handle(self, *args, **options):
        semantics = None
        if options["semantics"]:
            with open(options["semantics"], encoding="utf-8") as stream:
                semantics = json.load(stream)
        artifact, manifest = convert_scene(
            options["point_cloud"],
            options["output_dir"],
            references=options["references"],
            trajectory=options["trajectory"],
            semantics=semantics,
        )
        self.stdout.write(json.dumps({"artifact": str(artifact), "manifest": manifest}, ensure_ascii=False))
