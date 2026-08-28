from types import SimpleNamespace

import roamerx_edge.mapping_cli as mapping_cli


def _capture_save(monkeypatch):
    """Run mapping_cli against a stub adapter and return the command it built."""
    seen = {}

    def save_mapping(command):
        seen.update(command)
        return {"state": "exited"}

    monkeypatch.setattr(
        mapping_cli,
        "build_adapter",
        lambda _config: SimpleNamespace(save_mapping=save_mapping),
    )
    return seen


def test_save_stops_the_slam_node_by_default(monkeypatch):
    # Mapping is finished once the map is saved, so the CLI has to match the
    # cloud mapping.save default (stop_process defaults to True in the adapter).
    # Leaving the node behind blocks the next start_mapping.
    seen = _capture_save(monkeypatch)

    assert mapping_cli.main(["save", "--config", "/dev/null"]) == 0
    assert seen["stop_process"] is True


def test_keep_slam_preserves_the_checkpoint_save(monkeypatch):
    seen = _capture_save(monkeypatch)

    assert mapping_cli.main(["save", "--config", "/dev/null", "--keep-slam"]) == 0
    assert seen["stop_process"] is False
