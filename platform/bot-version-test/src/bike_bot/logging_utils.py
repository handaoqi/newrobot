from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path


def rotating_file_handler(path: str | Path, max_bytes: int, backup_count: int) -> logging.Handler:
    log_path = Path(path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    return RotatingFileHandler(
        log_path,
        maxBytes=max(0, max_bytes),
        backupCount=max(0, backup_count),
        encoding="utf-8",
    )


def rotate_file_if_needed(path: str | Path, max_bytes: int, backup_count: int) -> None:
    if max_bytes <= 0 or backup_count <= 0:
        return

    log_path = Path(path)
    try:
        if not log_path.exists() or log_path.stat().st_size < max_bytes:
            return
    except OSError:
        return

    log_path.parent.mkdir(parents=True, exist_ok=True)
    oldest = log_path.with_name(f"{log_path.name}.{backup_count}")
    try:
        if oldest.exists():
            oldest.unlink()
        for index in range(backup_count - 1, 0, -1):
            current = log_path.with_name(f"{log_path.name}.{index}")
            if current.exists():
                current.rename(log_path.with_name(f"{log_path.name}.{index + 1}"))
        log_path.rename(log_path.with_name(f"{log_path.name}.1"))
    except OSError:
        return
