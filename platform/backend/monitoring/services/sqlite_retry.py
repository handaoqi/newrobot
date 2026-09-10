"""Small SQLite write retry helper used by high-volume background jobs."""

from __future__ import annotations

import logging
import time
from collections.abc import Callable
from typing import TypeVar

from django.conf import settings
from django.db import close_old_connections, connection
from django.db.utils import OperationalError


LOGGER = logging.getLogger(__name__)
T = TypeVar("T")


def configure_sqlite_connection(sender, connection, **kwargs) -> None:
    """Enable WAL and a matching busy timeout on every SQLite connection.

    Django does not turn WAL on by itself.  The center has several writers
    (API workers, the device worker, and the patrol scheduler), so the default
    rollback journal turns routine telemetry into ``database is locked``.
    """

    if connection.vendor != "sqlite":
        return
    timeout_seconds = connection.settings_dict.get("OPTIONS", {}).get("timeout", 30)
    try:
        timeout_ms = max(1_000, int(float(timeout_seconds) * 1_000))
    except (TypeError, ValueError):
        timeout_ms = 30_000
    with connection.cursor() as cursor:
        cursor.execute(f"PRAGMA busy_timeout={timeout_ms}")
        if connection.in_atomic_block:
            return
        cursor.execute("PRAGMA journal_mode=WAL")
        cursor.execute("PRAGMA synchronous=NORMAL")


def checkpoint_sqlite_wal() -> None:
    """Truncate the WAL after a retention drain so deleted pages can be reused."""

    if connection.vendor != "sqlite":
        return
    with connection.cursor() as cursor:
        cursor.execute("PRAGMA wal_checkpoint(TRUNCATE)")


def is_sqlite_lock_error(exc: BaseException) -> bool:
    """Return true only for the transient SQLite writer-lock failure."""

    return connection.vendor == "sqlite" and "database is locked" in str(exc).lower()


def with_sqlite_lock_retry(operation: Callable[[], T], *, label: str = "sqlite write") -> T:
    """Retry a whole transaction after a transient SQLite writer collision."""

    attempts = max(1, int(getattr(settings, "SQLITE_LOCK_RETRY_ATTEMPTS", 4)))
    backoff = max(0.0, float(getattr(settings, "SQLITE_LOCK_RETRY_BACKOFF_SECONDS", 0.25)))
    for attempt in range(1, attempts + 1):
        try:
            return operation()
        except OperationalError as exc:
            if not is_sqlite_lock_error(exc) or attempt >= attempts:
                raise
            delay = backoff * (2 ** (attempt - 1))
            LOGGER.warning(
                "%s hit SQLite writer lock; retry %s/%s in %.2fs",
                label,
                attempt,
                attempts - 1,
                delay,
            )
            close_old_connections()
            if delay:
                time.sleep(delay)
    raise AssertionError("unreachable")
