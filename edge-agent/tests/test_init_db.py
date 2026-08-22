import sqlite3

from roamerx_edge.init_db import EDGE_TABLES, initialize_edge_database


def test_initialize_edge_database_creates_empty_schema_and_is_idempotent(tmp_path):
    path = tmp_path / "edge.db"

    first = initialize_edge_database(path)
    assert first["created"] is True
    assert set(first["counts"]) == EDGE_TABLES
    assert all(count == 0 for count in first["counts"].values())

    with sqlite3.connect(path) as connection:
        connection.execute(
            "INSERT INTO agent_metadata(key, value) VALUES (?, ?)",
            ("test", '"value"'),
        )
    second = initialize_edge_database(path)
    assert second["created"] is False
    assert second["counts"]["agent_metadata"] == 1
