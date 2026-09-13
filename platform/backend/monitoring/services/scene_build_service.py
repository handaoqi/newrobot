"""Run server-side street-block builds without an online robot."""

from __future__ import annotations

import json
import logging
import tempfile
from pathlib import Path

from django.core.files.base import ContentFile
from django.db import transaction
from django.utils import timezone

from ..models import MapSceneBuild
from .map_scene_service import scene_cloud_path
from .scene_converter import convert_scene, sha256_path


LOGGER = logging.getLogger(__name__)


def _description(map_data) -> dict:
    try:
        value = json.loads(map_data.description or "{}")
    except (TypeError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def claim_scene_build() -> MapSceneBuild | None:
    with transaction.atomic():
        build = (
            MapSceneBuild.objects.select_for_update()
            .filter(state="queued", config__engine="server_code")
            .order_by("created_at")
            .first()
        )
        if build is None:
            return None
        build.state = "running"
        build.stage = "staging"
        build.progress_percent = 5
        build.error_message = ""
        build.save(update_fields=["state", "stage", "progress_percent", "error_message", "updated_at"])
        return build


def _progress(build: MapSceneBuild, stage: str, percent: int) -> None:
    build.stage = stage
    build.progress_percent = percent
    build.save(update_fields=["stage", "progress_percent", "updated_at"])


def _matching_semantics(build: MapSceneBuild, cloud_checksum: str, uploaded_cloud: bool) -> tuple[dict | None, list[str]]:
    if not bool((build.config or {}).get("use_ptv3")):
        return None, []
    description = _description(build.map_data)
    scene = description.get("scene_manifest") if isinstance(description.get("scene_manifest"), dict) else {}
    semantics = scene.get("scene_semantics") if isinstance(scene.get("scene_semantics"), dict) else None
    if not semantics:
        return None, ["未找到可复用的 PTv3 结果，已使用纯代码转换"]
    if uploaded_cloud:
        source_hash = str((semantics.get("source") or {}).get("cloud_sha256") or "")
        if not source_hash or source_hash != cloud_checksum:
            return None, ["上传点云与 PTv3 来源不匹配，已忽略 PTv3 结果"]
    else:
        map_hash = str(semantics.get("map_sha256") or "")
        if map_hash and map_hash != cloud_checksum:
            return None, ["当前地图版本与 PTv3 结果不匹配，已忽略 PTv3 结果"]
    return semantics, []


def run_scene_build(build_or_id: MapSceneBuild | str) -> MapSceneBuild:
    build = build_or_id if isinstance(build_or_id, MapSceneBuild) else MapSceneBuild.objects.get(pk=build_or_id)
    build = MapSceneBuild.objects.select_related("map_data", "scene_input").get(pk=build.pk)
    try:
        scene_input = build.scene_input
        uploaded_cloud = bool(scene_input and scene_input.point_cloud)
        if uploaded_cloud:
            cloud_path = Path(scene_input.point_cloud.path)
            cloud_checksum = sha256_path(cloud_path)
        else:
            cloud_path, cloud_checksum, _ = scene_cloud_path(build.map_data, max_points=1_000_000)
        references = [item.file.path for item in scene_input.references.all()] if scene_input else []
        trajectory = scene_input.trajectory.path if scene_input and scene_input.trajectory else None
        semantics, warnings = _matching_semantics(build, cloud_checksum, uploaded_cloud)

        _progress(build, "geometry", 25)
        with tempfile.TemporaryDirectory(prefix=f"roamerx-scene-{build.id}-") as temporary:
            artifact, manifest = convert_scene(
                cloud_path,
                temporary,
                references=references,
                trajectory=trajectory,
                semantics=semantics,
                grid_resolution_m=float((build.config or {}).get("grid_resolution_m") or 0.35),
                max_points=1_000_000,
            )
            _progress(build, "exporting", 85)
            manifest["warnings"] = warnings
            manifest["engine"] = "server_code"
            manifest["semantic_source"] = "ptv3+geometry" if semantics and manifest["quality"].get("ptv3_matches") else "geometry"
            build.artifact.save(f"street-block-{build.id}.glb", ContentFile(artifact.read_bytes()), save=False)

        build.manifest = manifest
        build.metrics = manifest.get("quality", {})
        build.state = "ready"
        build.stage = "completed"
        build.progress_percent = 100
        build.error_message = ""
        build.finished_at = timezone.now()
        build.save()

        description = _description(build.map_data)
        scene = description.get("scene_manifest") if isinstance(description.get("scene_manifest"), dict) else {}
        scene["street_block"] = {
            **manifest,
            "available": True,
            "build_id": str(build.id),
            "url": f"/api/maps/{build.map_data_id}/scene-builds/{build.id}/artifact/",
        }
        description["scene_manifest"] = scene
        build.map_data.description = json.dumps(description, ensure_ascii=False)
        build.map_data.save(update_fields=["description", "updated_at"])
    except Exception as exc:
        LOGGER.exception("server scene build failed: %s", build.id)
        build.state = "failed"
        build.stage = "failed"
        build.progress_percent = min(99, build.progress_percent)
        build.error_message = str(exc)
        build.finished_at = timezone.now()
        build.save()
    return build
