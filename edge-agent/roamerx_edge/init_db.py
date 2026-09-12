from __future__ import annotations

import argparse
import os
import sqlite3
from pathlib import Path

from .config import EdgeConfig
from .local_store import LocalStore


EDGE_TABLES = {
    "processed_commands",
    "task_context",
    "outbox",
    "trajectory_sequence",
    "agent_metadata",
    "self_heal_episodes",
    "self_heal_actions",
    "self_heal_daily_summary",
}


def initialize_edge_database(path: str | Path) -> dict:
    database_path = Path(path).expanduser()
    existed = database_path.exists()
    store = LocalStore(str(database_path))
    store.close()

    with sqlite3.connect(database_path) as connection:
        integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
        tables = {
            row[0]
            for row in connection.execute(
                "SELECT name FROM sqlite_master WHERE type='table'"
            )
            if not row[0].startswith("sqlite_")
        }
        counts = {
            table: connection.execute(f'SELECT COUNT(*) FROM "{table}"').fetchone()[0]
            for table in sorted(EDGE_TABLES)
        }
    missing = EDGE_TABLES - tables
    if integrity != "ok" or missing:
        raise RuntimeError(
            f"Edge database validation failed: integrity={integrity}, missing={sorted(missing)}"
        )
    return {
        "path": str(database_path),
        "created": not existed,
        "integrity": integrity,
        "counts": counts,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Initialize the Edge Agent SQLite database")
    parser.add_argument(
        "--config",
        default=os.getenv("EDGE_CONFIG", "config.yaml"),
        help="Edge Agent YAML configuration",
    )
    parser.add_argument("--database", help="Override storage.sqlite_path")
    args = parser.parse_args()
    database = args.database or EdgeConfig.load(args.config).storage.sqlite_path
    result = initialize_edge_database(database)
    state = "created" if result["created"] else "verified"
    print(f"edge database {state}: {result['path']}")
    print(f"integrity={result['integrity']}")
    for table, count in result["counts"].items():
        print(f"{table}={count}")


if __name__ == "__main__":
    main()
