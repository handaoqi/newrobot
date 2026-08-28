from pathlib import Path

from roamerx_patrol_demo.bag_contract import BagContractError, inspect_bag_directory


def test_missing_metadata_is_a_contract_error(tmp_path: Path):
    (tmp_path / "part_0.db3").write_bytes(b"not-a-database")
    try:
        inspect_bag_directory(tmp_path)
    except BagContractError as exc:
        assert "metadata.yaml" in str(exc)
    else:
        raise AssertionError("missing metadata should fail")


def test_path_must_stay_under_allowed_root(tmp_path: Path):
    bag = tmp_path / "bags" / "ok"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    (bag / "ok_0.db3").write_bytes(b"x")
    result = inspect_bag_directory(bag, allowed_root=tmp_path / "bags")
    assert result.path == bag.resolve()


def test_mcap_bag_is_accepted(tmp_path: Path):
    """Recording defaults to mcap; only the payload suffix differs from sqlite3."""
    (tmp_path / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    (tmp_path / "ok_0.mcap").write_bytes(b"x")
    result = inspect_bag_directory(tmp_path)
    assert [p.name for p in result.data_files] == ["ok_0.mcap"]


def test_metadata_without_payload_is_a_contract_error(tmp_path: Path):
    (tmp_path / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    try:
        inspect_bag_directory(tmp_path)
    except BagContractError as exc:
        assert ".mcap" in str(exc)
    else:
        raise AssertionError("a bag with no storage file should fail")
