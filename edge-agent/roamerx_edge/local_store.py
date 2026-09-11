from __future__ import annotations

import json
import sqlite3
import threading
from pathlib import Path
from typing import Any


class LocalStore:
    def __init__(self, path: str, *, trajectory_outbox_limit: int = 720, system_log_outbox_limit: int = 500) -> None:
        self.path = path
        self.trajectory_outbox_limit = max(1, int(trajectory_outbox_limit))
        self.system_log_outbox_limit = max(1, int(system_log_outbox_limit))
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._connection = sqlite3.connect(path, check_same_thread=False)
        self._connection.row_factory = sqlite3.Row
        self._initialize()

    def close(self) -> None:
        self._connection.close()

    def _initialize(self) -> None:
        with self._connection:
            self._connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS processed_commands (
                    command_id TEXT PRIMARY KEY,
                    ack_json TEXT NOT NULL,
                    result_json TEXT,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
                );
                CREATE TABLE IF NOT EXISTS task_context (
                    task_execution_id TEXT PRIMARY KEY,
                    state TEXT NOT NULL,
                    state_version INTEGER NOT NULL,
                    route_snapshot_json TEXT NOT NULL,
                    current_waypoint_index INTEGER NOT NULL DEFAULT 0,
                    start_command_id TEXT,
                    record_rosbag INTEGER NOT NULL DEFAULT 0,
                    post_arrival_waypoint_index INTEGER,
                    post_arrival_stage TEXT NOT NULL DEFAULT '',
                    arrival_side_effects_started INTEGER NOT NULL DEFAULT 0,
                    arrival_micro_adjust_total_m REAL NOT NULL DEFAULT 0,
                    arrival_micro_adjust_steps INTEGER NOT NULL DEFAULT 0,
                    arrival_micro_adjust_started_at REAL,
                    updated_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
                );
                CREATE TABLE IF NOT EXISTS outbox (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    topic TEXT NOT NULL,
                    qos INTEGER NOT NULL,
                    retain INTEGER NOT NULL,
                    message_type TEXT NOT NULL,
                    dedupe_key TEXT UNIQUE,
                    payload_json TEXT NOT NULL,
                    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
                );
                CREATE TABLE IF NOT EXISTS trajectory_sequence (
                    task_execution_id TEXT PRIMARY KEY,
                    next_seq INTEGER NOT NULL DEFAULT 0
                );
                CREATE TABLE IF NOT EXISTS agent_metadata (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL
                );
                """
            )
            task_columns = {
                row[1] for row in self._connection.execute("PRAGMA table_info(task_context)").fetchall()
            }
            if "record_rosbag" not in task_columns:
                self._connection.execute(
                    "ALTER TABLE task_context ADD COLUMN record_rosbag INTEGER NOT NULL DEFAULT 0"
                )
            if "post_arrival_waypoint_index" not in task_columns:
                self._connection.execute(
                    "ALTER TABLE task_context ADD COLUMN post_arrival_waypoint_index INTEGER"
                )
            if "post_arrival_stage" not in task_columns:
                self._connection.execute(
                    "ALTER TABLE task_context ADD COLUMN post_arrival_stage TEXT NOT NULL DEFAULT ''"
                )
            if "arrival_side_effects_started" not in task_columns:
                self._connection.execute(
                    "ALTER TABLE task_context ADD COLUMN arrival_side_effects_started INTEGER NOT NULL DEFAULT 0"
                )
            if "arrival_micro_adjust_total_m" not in task_columns:
                self._connection.execute(
                    "ALTER TABLE task_context ADD COLUMN arrival_micro_adjust_total_m REAL NOT NULL DEFAULT 0"
                )
            if "arrival_micro_adjust_steps" not in task_columns:
                self._connection.execute(
                    "ALTER TABLE task_context ADD COLUMN arrival_micro_adjust_steps INTEGER NOT NULL DEFAULT 0"
                )
            if "arrival_micro_adjust_started_at" not in task_columns:
                self._connection.execute(
                    "ALTER TABLE task_context ADD COLUMN arrival_micro_adjust_started_at REAL"
                )
            self._prune_trajectory_outbox_locked()

    def _prune_trajectory_outbox_locked(self) -> None:
        self._connection.execute(
            """
            DELETE FROM outbox
            WHERE message_type = 'trajectory.batch'
              AND id NOT IN (
                  SELECT id FROM outbox
                  WHERE message_type = 'trajectory.batch'
                  ORDER BY id DESC
                  LIMIT ?
              )
            """,
            (self.trajectory_outbox_limit,),
        )

    def _prune_system_log_outbox_locked(self) -> None:
        self._connection.execute(
            """
            DELETE FROM outbox
            WHERE message_type = 'system.log.batch'
              AND id NOT IN (
                  SELECT id FROM outbox
                  WHERE message_type = 'system.log.batch'
                  ORDER BY id DESC
                  LIMIT ?
              )
            """,
            (self.system_log_outbox_limit,),
        )

    def get_processed_command(self, command_id: str) -> dict[str, Any] | None:
        row = self._connection.execute(
            "SELECT ack_json, result_json FROM processed_commands WHERE command_id = ?",
            (command_id,),
        ).fetchone()
        if not row:
            return None
        return {
            "ack": json.loads(row["ack_json"]),
            "result": json.loads(row["result_json"]) if row["result_json"] else None,
        }

    def save_command_ack(self, command_id: str, ack: dict) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO processed_commands(command_id, ack_json)
                VALUES (?, ?)
                ON CONFLICT(command_id) DO UPDATE SET ack_json=excluded.ack_json, updated_at=CURRENT_TIMESTAMP
                """,
                (command_id, json.dumps(ack, ensure_ascii=False)),
            )

    def save_command_result(self, command_id: str, result: dict) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO processed_commands(command_id, ack_json, result_json)
                VALUES (?, 'null', ?)
                ON CONFLICT(command_id) DO UPDATE SET
                    result_json=excluded.result_json,
                    updated_at=CURRENT_TIMESTAMP
                """,
                (command_id, json.dumps(result, ensure_ascii=False)),
            )

    def save_task_context(self, context: dict) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO task_context(
                    task_execution_id, state, state_version, route_snapshot_json,
                    current_waypoint_index, start_command_id, record_rosbag,
                    post_arrival_waypoint_index, post_arrival_stage,
                    arrival_side_effects_started, arrival_micro_adjust_total_m,
                    arrival_micro_adjust_steps, arrival_micro_adjust_started_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(task_execution_id) DO UPDATE SET
                    state=excluded.state,
                    state_version=excluded.state_version,
                    route_snapshot_json=excluded.route_snapshot_json,
                    current_waypoint_index=excluded.current_waypoint_index,
                    start_command_id=excluded.start_command_id,
                    record_rosbag=excluded.record_rosbag,
                    post_arrival_waypoint_index=excluded.post_arrival_waypoint_index,
                    post_arrival_stage=excluded.post_arrival_stage,
                    arrival_side_effects_started=excluded.arrival_side_effects_started,
                    arrival_micro_adjust_total_m=excluded.arrival_micro_adjust_total_m,
                    arrival_micro_adjust_steps=excluded.arrival_micro_adjust_steps,
                    arrival_micro_adjust_started_at=excluded.arrival_micro_adjust_started_at,
                    updated_at=CURRENT_TIMESTAMP
                """,
                (
                    context["task_execution_id"],
                    context["state"],
                    context["state_version"],
                    json.dumps(context["route_snapshot"], ensure_ascii=False),
                    context.get("current_waypoint_index", 0),
                    context.get("start_command_id"),
                    int(bool(context.get("record_rosbag", False))),
                    context.get("post_arrival_waypoint_index"),
                    str(context.get("post_arrival_stage") or ""),
                    int(bool(context.get("arrival_side_effects_started", False))),
                    max(0.0, float(context.get("arrival_micro_adjust_total_m") or 0.0)),
                    max(0, int(context.get("arrival_micro_adjust_steps") or 0)),
                    context.get("arrival_micro_adjust_started_at"),
                ),
            )

    def load_active_task_context(self) -> dict | None:
        row = self._connection.execute(
            """
            SELECT * FROM task_context
            WHERE state NOT IN ('completed','failed','cancelled','timed_out','rejected')
            ORDER BY updated_at DESC LIMIT 1
            """
        ).fetchone()
        if not row:
            return None
        return {
            "task_execution_id": row["task_execution_id"],
            "state": row["state"],
            "state_version": row["state_version"],
            "route_snapshot": json.loads(row["route_snapshot_json"]),
            "current_waypoint_index": row["current_waypoint_index"],
            "start_command_id": row["start_command_id"],
            "record_rosbag": bool(row["record_rosbag"]),
            "post_arrival_waypoint_index": row["post_arrival_waypoint_index"],
            "post_arrival_stage": str(row["post_arrival_stage"] or ""),
            "arrival_side_effects_started": bool(row["arrival_side_effects_started"]),
            "arrival_micro_adjust_total_m": float(row["arrival_micro_adjust_total_m"] or 0.0),
            "arrival_micro_adjust_steps": int(row["arrival_micro_adjust_steps"] or 0),
            "arrival_micro_adjust_started_at": row["arrival_micro_adjust_started_at"],
        }

    def clear_task_context(self, task_execution_id: str, final_state: str) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                "UPDATE task_context SET state=?, updated_at=CURRENT_TIMESTAMP WHERE task_execution_id=?",
                (final_state, task_execution_id),
            )

    def enqueue_outbox(self, topic: str, payload: dict, *, qos: int = 1, retain: bool = False, dedupe_key: str | None = None) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT OR IGNORE INTO outbox(topic, qos, retain, message_type, dedupe_key, payload_json)
                VALUES (?, ?, ?, ?, ?, ?)
                """,
                (topic, qos, int(retain), payload["message_type"], dedupe_key, json.dumps(payload, ensure_ascii=False)),
            )
            if payload["message_type"] == "trajectory.batch":
                self._prune_trajectory_outbox_locked()
            elif payload["message_type"] == "system.log.batch":
                self._prune_system_log_outbox_locked()

    def list_pending_outbox(self, limit: int = 100) -> list[dict]:
        rows = self._connection.execute(
            """
            SELECT * FROM outbox
            ORDER BY CASE WHEN message_type = 'trajectory.batch' THEN 1 ELSE 0 END, id
            LIMIT ?
            """,
            (limit,),
        ).fetchall()
        return [
            {
                "id": row["id"],
                "topic": row["topic"],
                "qos": row["qos"],
                "retain": bool(row["retain"]),
                "dedupe_key": row["dedupe_key"],
                "payload": json.loads(row["payload_json"]),
            }
            for row in rows
        ]

    def ack_outbox(self, *, row_id: int | None = None, dedupe_key: str | None = None) -> None:
        with self._lock, self._connection:
            if row_id is not None:
                self._connection.execute("DELETE FROM outbox WHERE id=?", (row_id,))
            elif dedupe_key is not None:
                self._connection.execute("DELETE FROM outbox WHERE dedupe_key=?", (dedupe_key,))

    def outbox_count(self) -> int:
        return int(self._connection.execute("SELECT COUNT(*) FROM outbox").fetchone()[0])

    def set_metadata(self, key: str, value: Any) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO agent_metadata(key, value) VALUES (?, ?)
                ON CONFLICT(key) DO UPDATE SET value=excluded.value
                """,
                (key, json.dumps(value, ensure_ascii=False)),
            )

    def get_metadata(self, key: str) -> Any | None:
        row = self._connection.execute(
            "SELECT value FROM agent_metadata WHERE key=?",
            (key,),
        ).fetchone()
        return json.loads(row["value"]) if row else None

    @staticmethod
    def _trusted_pose_key(map_id: str, map_version: str) -> str:
        return f"trusted_pose:{map_id}:{map_version}"

    def save_last_trusted_pose(self, map_id: str, map_version: str, pose: dict) -> None:
        if not map_id:
            return
        self.set_metadata(self._trusted_pose_key(map_id, map_version), pose)

    def load_last_trusted_pose(self, map_id: str, map_version: str) -> dict | None:
        if not map_id:
            return None
        value = self.get_metadata(self._trusted_pose_key(map_id, map_version))
        return value if isinstance(value, dict) else None

    def next_trajectory_seq(self, task_execution_id: str) -> int:
        with self._lock, self._connection:
            row = self._connection.execute(
                "SELECT next_seq FROM trajectory_sequence WHERE task_execution_id=?",
                (task_execution_id,),
            ).fetchone()
            value = int(row["next_seq"]) if row else 0
            self._connection.execute(
                """
                INSERT INTO trajectory_sequence(task_execution_id, next_seq) VALUES (?, ?)
                ON CONFLICT(task_execution_id) DO UPDATE SET next_seq=excluded.next_seq
                """,
                (task_execution_id, value + 1),
            )
            return value
