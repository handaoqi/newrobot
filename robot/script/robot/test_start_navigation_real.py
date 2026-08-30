from pathlib import Path


SCRIPT = Path(__file__).with_name("start_navigation_real.sh")


def _function_body(source: str, name: str) -> str:
    start = source.index(f"{name}() {{")
    end = source.index("\n}", start)
    return source[start:end]


def test_localization_start_waits_for_real_process_not_cached_ros_graph():
    source = SCRIPT.read_text(encoding="utf-8")
    body = _function_body(source, "wait_for_localization_process")

    assert "is_localization_node_alive" in body
    assert "ros2 node list" not in body


def test_map_loading_uses_live_rclpy_client_and_validates_response():
    source = SCRIPT.read_text(encoding="utf-8")
    body = _function_body(source, "load_pcd_map")

    assert "load_localization_map.py" in body
    assert "ros2 service call" not in body


def test_invalid_localization_reloads_map_or_restarts_safe_hold_lio():
    source = SCRIPT.read_text(encoding="utf-8")
    body = _function_body(source, "recover_invalid_localization")

    assert "localization_has_fresh_lio" in body
    assert "load_pcd_map" in body
    assert "restart_localization_only" in body
    assert "wait_for_localization" in body


def test_localization_wait_uses_an_absolute_deadline():
    source = SCRIPT.read_text(encoding="utf-8")
    body = _function_body(source, "wait_for_localization")

    assert "deadline=$((SECONDS + LOCALIZATION_WAIT_SECONDS))" in body
    assert "--timeout 1" in body
