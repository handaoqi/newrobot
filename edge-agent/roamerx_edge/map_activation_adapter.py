from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import os
from pathlib import Path
import json
import re
import shutil
import zipfile

import requests
import yaml

from .config import EdgeConfig
from .manual_map_cleanup import ManualMapCleanupError, build_manual_cleanup_map
from .map_coordinate import MapConstraintError, constraints_from_manifest, validate_map_constraints
from .map_package_finalize import load_map_manifest
from .map_version_pointer import MapVersionPointerError, activate_map_version
from .protocol import ProtocolError
from .safety_policy import RuntimeSafetyState


class MapActivationAdapter:
    """Switch the local map selected by the platform."""

    REQUIRED_FILES = ("map.yaml", "map.pgm", "map.pcd")
    OPTIONAL_FILES = ("map.txt", "gnss_origin.yaml", "map_manifest.json")

    def __init__(self, config: EdgeConfig, safety_state: RuntimeSafetyState, config_path: str) -> None:
        self.config = config
        self.safety_state = safety_state
        self.config_path = Path(config_path).expanduser()
        self.map_dir = Path(config.mapping.map_dir).expanduser()
        self.last_activation_error = ""
        self.applied_at = ""

    def activate(self, command: dict) -> dict:
        map_id = str(command.get("map_id", "")).strip()
        map_version = str(command.get("map_version", "")).strip()
        try:
            source_dir = self._resolve_source_dir(command)
            gnss_origin_yaml = str(command.get("gnss_origin_yaml") or "")
            if gnss_origin_yaml and not (source_dir / "gnss_origin.yaml").exists():
                if len(gnss_origin_yaml.encode("utf-8")) > 16384 or not all(
                    key in gnss_origin_yaml
                    for key in ("origin_latitude:", "origin_longitude:", "alignment_locked:")
                ):
                    raise ProtocolError("MAP_GNSS_METADATA_INVALID", "GNSS map origin metadata is invalid")
                (source_dir / "gnss_origin.yaml").write_text(gnss_origin_yaml, encoding="utf-8")
            cleanup_result = None
            if command.get("manual_edit"):
                safe_version = re.sub(r"[^A-Za-z0-9_.-]+", "_", map_version)[:96] or map_id
                edited_dir = self.map_dir / "manual_edits" / f"map_{map_id}_{safe_version}"
                cached_files = [edited_dir / name for name in self.REQUIRED_FILES]
                if all(path.exists() for path in cached_files):
                    # A manual revision is immutable for its map id/version.
                    # Reuse its complete local copy when the robot is offline
                    # from the cloud media host instead of rebuilding it.
                    cleanup_result = {"reused_cached_revision": True}
                else:
                    try:
                        cleanup_result = build_manual_cleanup_map(
                            source_dir,
                            edited_dir,
                            pgm_url=str(command.get("pgm_url") or ""),
                            yaml_url=str(command.get("yaml_url") or ""),
                            pgm_sha256=str(command.get("pgm_sha256") or ""),
                            yaml_sha256=str(command.get("yaml_sha256") or ""),
                        )
                    except ManualMapCleanupError as exc:
                        raise ProtocolError("MAP_MANUAL_CLEANUP_FAILED", str(exc)) from exc
                source_dir = edited_dir
            missing = [name for name in self.REQUIRED_FILES if not (source_dir / name).exists()]
            if missing:
                raise ProtocolError(
                    "MAP_FILES_MISSING",
                    f"selected map is missing local files: {', '.join(missing)} in {source_dir}",
                )
            self._validate_map_constraints(source_dir, command)

            try:
                switched = activate_map_version(
                    self.map_dir,
                    source_dir,
                    required_files=self.REQUIRED_FILES,
                    optional_files=self.OPTIONAL_FILES,
                )
            except MapVersionPointerError as exc:
                raise ProtocolError("MAP_ACTIVATION_POINTER_FAILED", str(exc)) from exc

            self._verify_switched_files(source_dir)
            self.config.robot.current_map_id = map_id
            self.config.robot.current_map_version = map_version
            self.safety_state.current_map_id = map_id
            self.safety_state.current_map_version = map_version
            self._persist_config(map_id, map_version)
            self.applied_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
            self.last_activation_error = ""
        except ProtocolError as exc:
            self.last_activation_error = f"{exc.code}: {exc.message}"
            raise

        result = {
            "map_id": map_id,
            "map_version": map_version,
            "map_name": command.get("map_name", ""),
            "source_dir": str(source_dir),
            "map_dir": str(self.map_dir),
            "switched_files": switched,
            "current_map": self.status(),
        }
        if cleanup_result is not None:
            result["manual_cleanup"] = cleanup_result
        return result

    def _validate_map_constraints(self, source_dir: Path, command: dict) -> dict:
        manifest = load_map_manifest(source_dir)
        if command.get("map_manifest") and isinstance(command.get("map_manifest"), dict) and not manifest:
            manifest = dict(command["map_manifest"])
            (source_dir / "map_manifest.json").write_text(
                json.dumps(manifest, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        gnss_origin = {}
        gnss_path = source_dir / "gnss_origin.yaml"
        if gnss_path.exists():
            try:
                gnss_origin = yaml.safe_load(gnss_path.read_text(encoding="utf-8")) or {}
            except yaml.YAMLError:
                gnss_origin = {}
        constraints = constraints_from_manifest(
            manifest,
            gnss_origin=gnss_origin if isinstance(gnss_origin, dict) else {},
            requested_scene_scope=str(command.get("scene_scope") or ""),
        )
        try:
            validate_map_constraints(constraints)
        except MapConstraintError as exc:
            raise ProtocolError(exc.code, exc.message) from exc
        requested_mode = str(command.get("localization_mode") or "").strip().lower()
        if constraints.get("coordinate_mode") == "local_only" and requested_mode in {"rtk", "rtk_ndt"}:
            raise ProtocolError("MAP_LOCAL_ONLY_NDT_ONLY", "local_only maps can only use NDT localization")
        if constraints.get("coordinate_mode") == "local_only" and str(command.get("scene_scope") or "").lower() in {
            "outdoor",
            "transition",
        }:
            code = (
                "MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN"
                if str(command.get("scene_scope")).lower() == "outdoor"
                else "MAP_LOCAL_ONLY_TRANSITION_FORBIDDEN"
            )
            raise ProtocolError(code, "local_only maps cannot activate outdoor or transition scenes")
        return constraints

    def status(self) -> dict:
        active_files = {}
        missing_files = []
        for name in self.REQUIRED_FILES + self.OPTIONAL_FILES:
            target = self.map_dir / name
            if target.exists():
                try:
                    active_files[name] = str(target.resolve())
                except OSError:
                    active_files[name] = str(target)
            elif name in self.REQUIRED_FILES:
                missing_files.append(name)

        source_dirs = {
            str(Path(path).parent)
            for name, path in active_files.items()
            if name in self.REQUIRED_FILES and path
        }
        local_state = "applied"
        local_error = self.last_activation_error
        if missing_files:
            local_state = "missing_files"
            local_error = f"missing active map files: {', '.join(missing_files)}"
        elif len(source_dirs) > 1:
            local_state = "mixed_files"
            local_error = "active map files point to different source directories"

        status = {
            "map_id": self.config.robot.current_map_id,
            "map_version": self.config.robot.current_map_version,
            "sha256": None,
            "local_map_dir": str(self.map_dir),
            "source_dir": next(iter(source_dirs), ""),
            "version_pointer": str((self.map_dir / "current").resolve())
            if (self.map_dir / "current").is_symlink() else "",
            "active_files": active_files,
            "local_state": local_state,
            "applied_at": self.applied_at,
            "last_activation_error": local_error,
        }
        self.safety_state.current_map_local_state = local_state
        self.safety_state.current_map_error = local_error
        return status

    def mapping_start_pose(self) -> dict:
        """Read the first recorded map trajectory pose for cold-start localization."""
        trajectory = self.map_dir / "map.txt"
        try:
            with trajectory.open("r", encoding="utf-8") as stream:
                for raw_line in stream:
                    line = raw_line.strip()
                    if not line or line.startswith("#"):
                        continue
                    fields = line.replace(",", " ").split()
                    if len(fields) < 3:
                        continue
                    return {
                        "x": float(fields[0]),
                        "y": float(fields[1]),
                        "z": 0.0,
                        "yaw": float(fields[2]),
                        "source": "mapping_start",
                    }
        except (OSError, ValueError) as exc:
            raise ProtocolError(
                "MAPPING_START_POSE_INVALID",
                f"cannot read mapping start pose from {trajectory}: {exc}",
            ) from exc
        raise ProtocolError(
            "MAPPING_START_POSE_MISSING",
            f"map trajectory has no usable start pose: {trajectory}",
        )

    def _resolve_source_dir(self, command: dict) -> Path:
        explicit = str(command.get("local_map_dir") or "").strip()
        if explicit:
            source_dir = Path(explicit).expanduser()
            if source_dir.exists() and all((source_dir / name).is_file() for name in self.REQUIRED_FILES):
                return source_dir

        image_path = str(command.get("local_image_path") or "").strip()
        if image_path:
            source_dir = Path(image_path).expanduser().parent
            if source_dir.exists() and all((source_dir / name).is_file() for name in self.REQUIRED_FILES):
                return source_dir

        map_version = str(command.get("source_map_version") or command.get("map_name") or "").strip()
        if map_version:
            candidate = self.map_dir / map_version
            if candidate.exists() and all((candidate / name).is_file() for name in self.REQUIRED_FILES):
                return candidate

        package_url = str(command.get("package_url") or "").strip()
        if package_url:
            return self._download_package_source(package_url, command)

        raise ProtocolError("MAP_SOURCE_NOT_FOUND", "selected map has no local source directory on this robot")

    def resolve_source_dir(self, command: dict) -> Path:
        """Resolve a map source for read-only workflows such as offline review."""
        return self._resolve_source_dir(command)

    def _download_package_source(self, package_url: str, command: dict) -> Path:
        if not package_url.startswith(("http://", "https://")):
            raise ProtocolError("MAP_PACKAGE_URL_INVALID", "map package URL must use HTTP or HTTPS")
        cache_key = re.sub(
            r"[^A-Za-z0-9_.-]+",
            "_",
            f"{command.get('map_id', '')}_{command.get('map_version', '')}",
        )[:128].strip("_.") or "map"
        cache_root = self.map_dir / ".cloud_packages" / cache_key
        required = [cache_root / name for name in self.REQUIRED_FILES]
        if all(path.is_file() for path in required):
            return cache_root

        temporary_root = cache_root.with_name(cache_root.name + ".partial")
        if temporary_root.exists():
            shutil.rmtree(temporary_root)
        temporary_root.mkdir(parents=True, exist_ok=True)
        archive_path = temporary_root / "map_package.zip"
        try:
            response = requests.get(package_url, stream=True, timeout=300)
            response.raise_for_status()
            digest = hashlib.sha256()
            with archive_path.open("wb") as stream:
                for chunk in response.iter_content(chunk_size=1024 * 1024):
                    if chunk:
                        digest.update(chunk)
                        stream.write(chunk)
            expected_sha256 = str(command.get("package_sha256") or "").strip().lower()
            if expected_sha256 and digest.hexdigest().lower() != expected_sha256:
                raise ProtocolError("MAP_PACKAGE_CHECKSUM_MISMATCH", "downloaded map package checksum is invalid")

            extract_root = temporary_root / "content"
            extract_root.mkdir()
            with zipfile.ZipFile(archive_path) as archive:
                root = extract_root.resolve()
                for member in archive.infolist():
                    destination = (extract_root / member.filename).resolve()
                    if root != destination and root not in destination.parents:
                        raise ProtocolError("MAP_PACKAGE_INVALID", "map package contains an unsafe path")
                archive.extractall(extract_root)
            missing = [name for name in self.REQUIRED_FILES if not (extract_root / name).is_file()]
            if missing:
                raise ProtocolError("MAP_FILES_MISSING", f"downloaded map package is missing: {', '.join(missing)}")
            if cache_root.exists():
                shutil.rmtree(cache_root)
            os.replace(extract_root, cache_root)
            return cache_root
        except ProtocolError:
            raise
        except (OSError, requests.RequestException, zipfile.BadZipFile) as exc:
            raise ProtocolError("MAP_PACKAGE_DOWNLOAD_FAILED", str(exc)) from exc
        finally:
            if temporary_root.exists():
                shutil.rmtree(temporary_root)

    def _persist_config(self, map_id: str, map_version: str) -> None:
        raw = yaml.safe_load(self.config_path.read_text(encoding="utf-8")) or {}
        robot = raw.setdefault("robot", {})
        robot["current_map_id"] = map_id
        robot["current_map_version"] = map_version
        self.config_path.write_text(yaml.safe_dump(raw, allow_unicode=True, sort_keys=False), encoding="utf-8")

    def _verify_switched_files(self, source_dir: Path) -> None:
        mismatched = []
        for name in self.REQUIRED_FILES:
            target = self.map_dir / name
            expected = source_dir / name
            if not target.exists() and not target.is_symlink():
                mismatched.append(name)
                continue
            try:
                if target.resolve() != expected.resolve():
                    mismatched.append(name)
            except OSError:
                mismatched.append(name)
        if mismatched:
            raise ProtocolError(
                "MAP_ACTIVATE_FAILED",
                f"active map files did not switch correctly: {', '.join(mismatched)}",
            )
