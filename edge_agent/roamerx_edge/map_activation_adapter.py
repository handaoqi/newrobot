from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import re

import yaml

from .config import EdgeConfig
from .manual_map_cleanup import ManualMapCleanupError, build_manual_cleanup_map
from .protocol import ProtocolError
from .safety_policy import RuntimeSafetyState


class MapActivationAdapter:
    """Switch the local map selected by the platform."""

    REQUIRED_FILES = ("map.yaml", "map.pgm", "map.pcd")
    OPTIONAL_FILES = ("map.txt", "gnss_origin.yaml")

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

            switched = {}
            for name in self.REQUIRED_FILES + self.OPTIONAL_FILES:
                source = source_dir / name
                target = self.map_dir / name
                if target.exists() or target.is_symlink():
                    target.unlink()
                # Optional metadata is map-scoped. Leaving a previous map's
                # GNSS origin active silently applies the wrong ENU transform.
                if not source.exists():
                    continue
                target.symlink_to(source)
                switched[name] = str(source)

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

    def status(self) -> dict:
        active_files = {}
        missing_files = []
        for name in self.REQUIRED_FILES + self.OPTIONAL_FILES:
            target = self.map_dir / name
            if target.exists() or target.is_symlink():
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
            if source_dir.exists():
                return source_dir

        image_path = str(command.get("local_image_path") or "").strip()
        if image_path:
            source_dir = Path(image_path).expanduser().parent
            if source_dir.exists():
                return source_dir

        map_version = str(command.get("source_map_version") or command.get("map_name") or "").strip()
        if map_version:
            candidate = self.map_dir / map_version
            if candidate.exists():
                return candidate

        raise ProtocolError("MAP_SOURCE_NOT_FOUND", "selected map has no local source directory on this robot")

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
