"""Atomically select one immutable map product directory."""

from __future__ import annotations

import os
from pathlib import Path


class MapVersionPointerError(RuntimeError):
    pass


def activate_map_version(
    map_dir: str | Path,
    source_dir: str | Path,
    *,
    required_files: tuple[str, ...],
    optional_files: tuple[str, ...] = (),
) -> dict[str, str]:
    """Switch all legacy ``map_dir/map.*`` paths through one current pointer.

    The compatibility links are stable (``map.pcd -> current/map.pcd``). Once
    they have been installed, replacing the single ``current`` symlink changes
    every map consumer to the same immutable source directory at once.
    """

    root = Path(map_dir).expanduser().resolve()
    source = Path(source_dir).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    if source == root:
        raise MapVersionPointerError("map source must be an immutable child directory")
    missing = [name for name in required_files if not (source / name).is_file()]
    if missing:
        raise MapVersionPointerError(f"map source is missing: {', '.join(missing)}")

    current = root / "current"
    if current.exists() and not current.is_symlink():
        raise MapVersionPointerError(f"map version pointer is not a symlink: {current}")

    # During the one-time migration, first point current at the existing
    # coherent required-file source. Compatibility links can then be replaced
    # one by one without exposing broken required paths.
    if not current.is_symlink():
        existing_sources = set()
        for name in required_files:
            target = root / name
            if target.exists():
                existing_sources.add(target.resolve().parent)
        initial_source = existing_sources.pop() if len(existing_sources) == 1 else source
        _replace_symlink(current, initial_source)

    for name in required_files + optional_files:
        _replace_symlink(root / name, Path("current") / name)

    _replace_symlink(current, source)

    switched = {
        name: str(source / name)
        for name in required_files + optional_files
        if (source / name).is_file()
    }
    for name in required_files:
        if (root / name).resolve() != (source / name).resolve():
            raise MapVersionPointerError(f"map pointer verification failed for {name}")
    return switched


def _replace_symlink(target: Path, link_value: Path) -> None:
    temporary = target.with_name(f".{target.name}.tmp-{os.getpid()}")
    if temporary.exists() or temporary.is_symlink():
        temporary.unlink()
    temporary.symlink_to(link_value)
    os.replace(temporary, target)
