from __future__ import annotations

import json
import sqlite3
import threading
from pathlib import Path
from typing import Any


class LocalStore:
    def __init__(
        self,
        path: str,
        *,
        trajectory_outbox_limit: int = 720,
        system_log_outbox_limit: int = 500,
        self_heal_retention_days: int = 180,
    ) -> None:
        self.path = path
        self.trajectory_outbox_limit = max(1, int(trajectory_outbox_limit))
        self.system_log_outbox_limit = max(1, int(system_log_outbox_limit))
        self.self_heal_retention_days = max(1, int(self_heal_retention_days))
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._connection = sqlite3.connect(path, check_same_thread=False)
        self._connection.row_factory = sqlite3.Row
        self._connection.execute("PRAGMA foreign_keys=ON")
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
                    loop_execution INTEGER NOT NULL DEFAULT 0,
                    loop_session_id TEXT NOT NULL DEFAULT '',
                    continuous_rosbag INTEGER NOT NULL DEFAULT 0,
                    post_arrival_waypoint_index INTEGER,
                    post_arrival_stage TEXT NOT NULL DEFAULT '',
                    arrival_side_effects_started INTEGER NOT NULL DEFAULT 0,
                    arrival_coarse_fallback_accepted INTEGER NOT NULL DEFAULT 0,
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
                CREATE TABLE IF NOT EXISTS self_heal_episodes (
                    episode_id TEXT PRIMARY KEY,
                    fault_label TEXT NOT NULL,
                    scene_mode TEXT NOT NULL,
                    task_execution_id TEXT NOT NULL DEFAULT '',
                    map_id TEXT NOT NULL DEFAULT '',
                    evidence_json TEXT NOT NULL DEFAULT '{}',
                    success INTEGER,
                    terminal_level INTEGER,
                    terminal_action TEXT NOT NULL DEFAULT '',
                    duration_seconds REAL,
                    reason TEXT NOT NULL DEFAULT '',
                    started_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    finished_at TEXT
                );
                CREATE INDEX IF NOT EXISTS idx_self_heal_episodes_started
                    ON self_heal_episodes(started_at);
                CREATE INDEX IF NOT EXISTS idx_self_heal_episodes_fault_success
                    ON self_heal_episodes(fault_label, success);
                CREATE TABLE IF NOT EXISTS self_heal_actions (
                    action_id TEXT PRIMARY KEY,
                    episode_id TEXT NOT NULL,
                    level INTEGER NOT NULL,
                    action_type TEXT NOT NULL,
                    success INTEGER,
                    reason TEXT NOT NULL DEFAULT '',
                    duration_seconds REAL,
                    started_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
                    finished_at TEXT,
                    FOREIGN KEY(episode_id) REFERENCES self_heal_episodes(episode_id)
                );
                CREATE INDEX IF NOT EXISTS idx_self_heal_actions_episode
                    ON self_heal_actions(episode_id, level);
                CREATE INDEX IF NOT EXISTS idx_self_heal_actions_type_success
                    ON self_heal_actions(action_type, success);
                CREATE TABLE IF NOT EXISTS self_heal_daily_summary (
                    summary_date TEXT NOT NULL,
                    category TEXT NOT NULL,
                    label TEXT NOT NULL,
                    total INTEGER NOT NULL DEFAULT 0,
                    succeeded INTEGER NOT NULL DEFAULT 0,
                    total_duration_seconds REAL NOT NULL DEFAULT 0,
                    PRIMARY KEY(summary_date, category, label)
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
            if "loop_execution" not in task_columns:
                self._connection.execute(
                    "ALTER TABLE task_context ADD COLUMN loop_execution INTEGER NOT NULL DEFAULT 0"
                )
            if "loop_session_id" not in task_columns:
                self._connection.execute(
                    "ALTER TABLE task_context ADD COLUMN loop_session_id TEXT NOT NULL DEFAULT ''"
                )
            if "continuous_rosbag" not in task_columns:
                self._connection.execute(
                    "ALTER TABLE task_context ADD COLUMN continuous_rosbag INTEGER NOT NULL DEFAULT 0"
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
            if "arrival_coarse_fallback_accepted" not in task_columns:
                self._connection.execute(
                    "ALTER TABLE task_context ADD COLUMN arrival_coarse_fallback_accepted INTEGER NOT NULL DEFAULT 0"
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
            action_columns = {
                row[1]
                for row in self._connection.execute(
                    "PRAGMA table_info(self_heal_actions)"
                ).fetchall()
            }
            if "duration_seconds" not in action_columns:
                self._connection.execute(
                    "ALTER TABLE self_heal_actions ADD COLUMN duration_seconds REAL"
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
                    loop_execution, loop_session_id, continuous_rosbag,
                    post_arrival_waypoint_index, post_arrival_stage,
                    arrival_side_effects_started, arrival_coarse_fallback_accepted,
                    arrival_micro_adjust_total_m, arrival_micro_adjust_steps,
                    arrival_micro_adjust_started_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(task_execution_id) DO UPDATE SET
                    state=excluded.state,
                    state_version=excluded.state_version,
                    route_snapshot_json=excluded.route_snapshot_json,
                    current_waypoint_index=excluded.current_waypoint_index,
                    start_command_id=excluded.start_command_id,
                    record_rosbag=excluded.record_rosbag,
                    loop_execution=excluded.loop_execution,
                    loop_session_id=excluded.loop_session_id,
                    continuous_rosbag=excluded.continuous_rosbag,
                    post_arrival_waypoint_index=excluded.post_arrival_waypoint_index,
                    post_arrival_stage=excluded.post_arrival_stage,
                    arrival_side_effects_started=excluded.arrival_side_effects_started,
                    arrival_coarse_fallback_accepted=excluded.arrival_coarse_fallback_accepted,
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
                    int(bool(context.get("loop_execution", False))),
                    str(context.get("loop_session_id") or ""),
                    int(bool(context.get("continuous_rosbag", False))),
                    context.get("post_arrival_waypoint_index"),
                    str(context.get("post_arrival_stage") or ""),
                    int(bool(context.get("arrival_side_effects_started", False))),
                    int(bool(context.get("arrival_coarse_fallback_accepted", False))),
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
            "loop_execution": bool(row["loop_execution"]),
            "loop_session_id": str(row["loop_session_id"] or ""),
            "continuous_rosbag": bool(row["continuous_rosbag"]),
            "post_arrival_waypoint_index": row["post_arrival_waypoint_index"],
            "post_arrival_stage": str(row["post_arrival_stage"] or ""),
            "arrival_side_effects_started": bool(row["arrival_side_effects_started"]),
            "arrival_coarse_fallback_accepted": bool(row["arrival_coarse_fallback_accepted"]),
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

    def clear_last_trusted_pose(self, map_id: str, map_version: str) -> None:
        """Invalidate the map-scoped trusted seed before a new task starts."""
        if not map_id:
            return
        with self._lock, self._connection:
            self._connection.execute(
                "DELETE FROM agent_metadata WHERE key=?",
                (self._trusted_pose_key(map_id, map_version),),
            )

    def start_self_heal_episode(
        self,
        episode_id: str,
        *,
        fault_label: str,
        scene_mode: str,
        task_execution_id: str = "",
        map_id: str = "",
        evidence: dict | None = None,
    ) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT OR IGNORE INTO self_heal_episodes(
                    episode_id, fault_label, scene_mode, task_execution_id,
                    map_id, evidence_json
                ) VALUES (?, ?, ?, ?, ?, ?)
                """,
                (
                    episode_id,
                    fault_label,
                    scene_mode,
                    task_execution_id,
                    map_id,
                    json.dumps(evidence or {}, ensure_ascii=False),
                ),
            )

    def finish_self_heal_episode(
        self,
        episode_id: str,
        *,
        success: bool,
        terminal_level: int,
        terminal_action: str,
        duration_seconds: float,
        reason: str = "",
    ) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                UPDATE self_heal_episodes
                SET success=?, terminal_level=?, terminal_action=?,
                    duration_seconds=?, reason=?, finished_at=CURRENT_TIMESTAMP
                WHERE episode_id=?
                """,
                (
                    int(bool(success)),
                    int(terminal_level),
                    terminal_action,
                    max(0.0, float(duration_seconds)),
                    reason,
                    episode_id,
                ),
            )

    def start_self_heal_action(
        self,
        action_id: str,
        *,
        episode_id: str,
        level: int,
        action_type: str,
    ) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT OR IGNORE INTO self_heal_actions(
                    action_id, episode_id, level, action_type
                ) VALUES (?, ?, ?, ?)
                """,
                (action_id, episode_id, int(level), action_type),
            )

    def finish_self_heal_action(
        self,
        action_id: str,
        *,
        success: bool,
        reason: str = "",
        duration_seconds: float | None = None,
    ) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                """
                UPDATE self_heal_actions
                SET success=?, reason=?, duration_seconds=COALESCE(?, duration_seconds),
                    finished_at=CURRENT_TIMESTAMP
                WHERE action_id=?
                """,
                (
                    int(bool(success)),
                    reason,
                    max(0.0, float(duration_seconds))
                    if duration_seconds is not None else None,
                    action_id,
                ),
            )

    def reconcile_unfinished_self_healing(
        self, reason: str = "interrupted_by_restart"
    ) -> dict:
        """Close records left open when the previous Edge process exited."""
        with self._lock, self._connection:
            actions = self._connection.execute(
                """
                UPDATE self_heal_actions
                SET success=0, reason=?,
                    duration_seconds=MAX(
                        0.0,
                        (julianday(CURRENT_TIMESTAMP) - julianday(started_at)) * 86400.0
                    ),
                    finished_at=CURRENT_TIMESTAMP
                WHERE finished_at IS NULL
                """,
                (reason,),
            ).rowcount
            episodes = self._connection.execute(
                """
                UPDATE self_heal_episodes
                SET success=0,
                    terminal_level=COALESCE(
                        (SELECT level FROM self_heal_actions a
                         WHERE a.episode_id=self_heal_episodes.episode_id
                         ORDER BY a.rowid DESC LIMIT 1), terminal_level, 0),
                    terminal_action=COALESCE(
                        (SELECT action_type FROM self_heal_actions a
                         WHERE a.episode_id=self_heal_episodes.episode_id
                         ORDER BY a.rowid DESC LIMIT 1), terminal_action, ''),
                    duration_seconds=MAX(
                        0.0,
                        (julianday(CURRENT_TIMESTAMP) - julianday(started_at)) * 86400.0
                    ),
                    reason=?, finished_at=CURRENT_TIMESTAMP
                WHERE finished_at IS NULL
                """,
                (reason,),
            ).rowcount
        return {"episodes": episodes, "actions": actions}

    def prune_self_healing(self, retention_days: int | None = None) -> dict:
        """Aggregate and remove completed self-healing detail beyond retention."""
        days = max(
            1,
            int(self.self_heal_retention_days if retention_days is None else retention_days),
        )
        cutoff = f"-{days} days"
        with self._lock, self._connection:
            self._connection.execute(
                """
                INSERT INTO self_heal_daily_summary(
                    summary_date, category, label, total, succeeded,
                    total_duration_seconds
                )
                SELECT date(started_at), 'fault', fault_label, COUNT(*),
                       SUM(CASE WHEN success=1 THEN 1 ELSE 0 END),
                       SUM(COALESCE(duration_seconds, 0.0))
                FROM self_heal_episodes
                WHERE finished_at IS NOT NULL
                  AND started_at < datetime('now', ?)
                GROUP BY date(started_at), fault_label
                ON CONFLICT(summary_date, category, label) DO UPDATE SET
                    total=total+excluded.total,
                    succeeded=succeeded+excluded.succeeded,
                    total_duration_seconds=
                        total_duration_seconds+excluded.total_duration_seconds
                """,
                (cutoff,),
            )
            self._connection.execute(
                """
                INSERT INTO self_heal_daily_summary(
                    summary_date, category, label, total, succeeded,
                    total_duration_seconds
                )
                SELECT date(a.started_at), 'action', a.action_type, COUNT(*),
                       SUM(CASE WHEN a.success=1 THEN 1 ELSE 0 END),
                       SUM(COALESCE(a.duration_seconds, 0.0))
                FROM self_heal_actions a
                JOIN self_heal_episodes e ON e.episode_id=a.episode_id
                WHERE e.finished_at IS NOT NULL
                  AND e.started_at < datetime('now', ?)
                GROUP BY date(a.started_at), a.action_type
                ON CONFLICT(summary_date, category, label) DO UPDATE SET
                    total=total+excluded.total,
                    succeeded=succeeded+excluded.succeeded,
                    total_duration_seconds=
                        total_duration_seconds+excluded.total_duration_seconds
                """,
                (cutoff,),
            )
            actions = self._connection.execute(
                """
                DELETE FROM self_heal_actions
                WHERE episode_id IN (
                    SELECT episode_id FROM self_heal_episodes
                    WHERE finished_at IS NOT NULL
                      AND started_at < datetime('now', ?)
                )
                """,
                (cutoff,),
            ).rowcount
            episodes = self._connection.execute(
                """
                DELETE FROM self_heal_episodes
                WHERE finished_at IS NOT NULL
                  AND started_at < datetime('now', ?)
                """,
                (cutoff,),
            ).rowcount
            foreign_key_violations = len(
                self._connection.execute("PRAGMA foreign_key_check").fetchall()
            )
            if foreign_key_violations:
                raise sqlite3.IntegrityError(
                    f"self-healing retention left {foreign_key_violations} foreign-key violations"
                )
        return {
            "episodes": episodes,
            "actions": actions,
            "retention_days": days,
            "foreign_key_violations": 0,
        }

    def self_heal_statistics(self) -> dict:
        episodes = self._connection.execute(
            """
            WITH combined AS (
                SELECT fault_label AS label, COUNT(*) AS total,
                       SUM(CASE WHEN success=1 THEN 1 ELSE 0 END) AS succeeded,
                       SUM(COALESCE(duration_seconds, 0.0)) AS total_duration
                FROM self_heal_episodes WHERE finished_at IS NOT NULL
                GROUP BY fault_label
                UNION ALL
                SELECT label, total, succeeded, total_duration_seconds
                FROM self_heal_daily_summary WHERE category='fault'
            )
            SELECT label AS fault_label, SUM(total) AS total,
                   SUM(succeeded) AS succeeded,
                   SUM(total_duration) / NULLIF(SUM(total), 0)
                       AS average_duration_seconds
            FROM combined GROUP BY label
            ORDER BY total DESC, fault_label
            """
        ).fetchall()
        actions = self._connection.execute(
            """
            WITH combined AS (
                SELECT action_type AS label, COUNT(*) AS total,
                       SUM(CASE WHEN success=1 THEN 1 ELSE 0 END) AS succeeded,
                       SUM(COALESCE(duration_seconds, 0.0)) AS total_duration
                FROM self_heal_actions WHERE finished_at IS NOT NULL
                GROUP BY action_type
                UNION ALL
                SELECT label, total, succeeded, total_duration_seconds
                FROM self_heal_daily_summary WHERE category='action'
            )
            SELECT label AS action_type, SUM(total) AS total,
                   SUM(succeeded) AS succeeded,
                   SUM(total_duration) / NULLIF(SUM(total), 0)
                       AS average_duration_seconds
            FROM combined GROUP BY label
            ORDER BY total DESC, action_type
            """
        ).fetchall()
        return {
            "faults": [dict(row) for row in episodes],
            "actions": [dict(row) for row in actions],
        }

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
