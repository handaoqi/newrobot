from pathlib import Path
from xml.etree import ElementTree

import yaml

from roamerx_edge.navigation_controllers import (
    global_controller_plugin_id,
    local_controller_plugin_id,
    normalize_global_controller,
    normalize_local_controller,
)


def test_normalize_local_controller_defaults_to_registered_mppi():
    assert normalize_local_controller(None) == "mppi"
    assert normalize_local_controller("mppi") == "mppi"
    assert normalize_local_controller("rpp") == "rpp"
    assert normalize_local_controller("unknown") == "mppi"


def test_normalize_global_controller_defaults_to_theta_star():
    assert normalize_global_controller(None) == "theta_star"
    assert normalize_global_controller("navfn") == "navfn"
    assert normalize_global_controller("invalid") == "theta_star"


def test_controller_plugin_ids():
    assert local_controller_plugin_id("rpp") == "RPP"
    assert local_controller_plugin_id("mppi") == "FollowPath"
    assert global_controller_plugin_id("theta_star") == "ThetaStar"
    assert global_controller_plugin_id("navfn") == "NavFn"


def test_default_navigation_trees_use_runtime_controller_selectors():
    tree_dir = (
        Path(__file__).resolve().parents[2]
        / "robot/src/navigation/src/navigo_bt_navigator/behavior_trees"
    )
    for filename in (
        "navigate_to_pose_w_replanning_and_recovery.xml",
        "navigate_through_poses_w_replanning_and_recovery.xml",
    ):
        root = ElementTree.parse(tree_dir / filename).getroot()
        planner_selector = root.find(".//PlannerSelector")
        controller_selector = root.find(".//ControllerSelector")
        compute = root.find(".//ComputePathToPose")
        if compute is None:
            compute = root.find(".//ComputePathThroughPoses")
        follow = root.find(".//FollowPath")

        assert planner_selector is not None
        assert planner_selector.attrib == {
            "selected_planner": "{selected_planner}",
            "default_planner": "ThetaStar",
            "topic_name": "/planner_selector",
        }
        assert controller_selector is not None
        assert controller_selector.attrib == {
            "selected_controller": "{selected_controller}",
            "default_controller": "FollowPath",
            "topic_name": "/controller_selector",
        }
        assert compute is not None
        assert compute.attrib["planner_id"] == "{selected_planner}"
        assert follow is not None
        assert follow.attrib["controller_id"] == "{selected_controller}"


def test_nav2_plugin_registry_matches_edge_controller_mapping():
    repo_root = Path(__file__).resolve().parents[2]
    params = yaml.safe_load(
        (
            repo_root
            / "robot/src/navigation/src/robot_navigo/params/navigo_params.yaml"
        ).read_text(encoding="utf-8")
    )
    planner = params["planner_server"]["ros__parameters"]
    controller = params["controller_server"]["ros__parameters"]

    assert planner["planner_plugins"] == ["ThetaStar", "NavFn"]
    assert planner["ThetaStar"]["plugin"] == "navigo_navfn_planner/ThetaStarPlanner"
    assert planner["NavFn"]["plugin"] == "navigo_navfn_planner/NavfnPlanner"
    assert controller["controller_plugins"] == ["FollowPath", "RPP"]
    assert controller["FollowPath"]["plugin"] == "navigo_mppi_controller::MPPIController"
    assert controller["RPP"]["plugin"] == "navigo_mppi_controller::RPPController"
