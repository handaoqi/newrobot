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
    assert normalize_local_controller("ilqr") == "ilqr"
    assert normalize_local_controller("unknown") == "mppi"


def test_normalize_global_controller_defaults_to_theta_star():
    assert normalize_global_controller(None) == "theta_star"
    assert normalize_global_controller("navfn") == "navfn"
    assert normalize_global_controller("smac_hybrid") == "smac_hybrid"
    assert normalize_global_controller("invalid") == "theta_star"


def test_controller_plugin_ids():
    assert local_controller_plugin_id("rpp") == "RPP"
    assert local_controller_plugin_id("mppi") == "FollowPath"
    assert local_controller_plugin_id("ilqr") == "ILQR"
    assert global_controller_plugin_id("theta_star") == "ThetaStar"
    assert global_controller_plugin_id("navfn") == "NavFn"
    assert global_controller_plugin_id("smac_hybrid") == "SmacHybrid"


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
        smoother_selector = root.find(".//SmootherSelector")
        smoothers = root.findall(".//SmoothPath")
        backups = root.findall(".//BackUp")
        drive_on_heading_nodes = root.findall(".//DriveOnHeading")
        lateral_recoveries = [
            node for node in drive_on_heading_nodes if "lateral_dist" in node.attrib
        ]
        feature_searches = root.findall(".//SearchLaserFeature/DriveOnHeading")
        spins = root.findall(".//Spin")
        adaptive_spins = root.findall(".//AdaptiveSpin")
        guarded_spins = root.findall(".//AdaptiveSpin/Spin")
        diagnoses = root.findall(".//FaultDiagnoseNode")
        smart_rtk_waits = root.findall(".//SmartRTKWait")
        fusion_profiles = root.findall(".//SetUkfWeight")

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
        assert smoother_selector is not None
        assert smoother_selector.attrib["default_smoother"] == "savitzky_golay"
        assert [node.attrib["smoother_id"] for node in smoothers] == [
            "{selected_smoother}", "simple_smoother"
        ]
        assert len(backups) == 1
        assert {node.attrib["lateral_dist"] for node in lateral_recoveries} == {
            "-0.20", "0.20"
        }
        assert len(feature_searches) == 1
        assert feature_searches[0].attrib["dist_to_travel"] == "0.12"
        assert spins == guarded_spins
        assert len(adaptive_spins) == 1
        assert adaptive_spins[0].attrib["episode_id"] == "{self_heal_episode}"
        assert adaptive_spins[0].attrib["fault_label"] == "{self_heal_fault}"
        assert adaptive_spins[0].attrib["permission_timeout_seconds"] == "1.0"
        assert len(diagnoses) >= 3
        assert len(smart_rtk_waits) == 1
        assert fusion_profiles[0].attrib["duration_seconds"] == "10.0"


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

    assert planner["planner_plugins"] == ["ThetaStar", "NavFn", "SmacHybrid"]
    assert planner["ThetaStar"]["plugin"] == "navigo_navfn_planner/ThetaStarPlanner"
    assert planner["NavFn"]["plugin"] == "navigo_navfn_planner/NavfnPlanner"
    assert planner["SmacHybrid"]["plugin"] == "navigo_navfn_planner/SmacHybridPlanner"
    assert controller["controller_plugins"] == ["FollowPath", "RPP", "ILQR"]
    assert controller["FollowPath"]["plugin"] == "navigo_mppi_controller::MPPIController"
    assert controller["RPP"]["plugin"] == "navigo_mppi_controller::RPPController"
    assert controller["ILQR"]["plugin"] == "navigo_mppi_controller::ILQRController"
    smoother = params["smoother_server"]["ros__parameters"]
    assert smoother["smoother_plugins"] == [
        "savitzky_golay", "simple_smoother", "passthrough_smoother"
    ]
    assert smoother["savitzky_golay"]["plugin"] == (
        "navigo_smoother::SavitzkyGolaySmoother"
    )


def test_collision_monitor_has_directional_lateral_stop_zones():
    repo_root = Path(__file__).resolve().parents[2]
    params = yaml.safe_load(
        (repo_root / "robot/src/navigation/src/robot_navigo/params/navigo_params.yaml")
        .read_text(encoding="utf-8")
    )
    monitor = params["collision_monitor"]["ros__parameters"]
    assert monitor["cmd_vel_in_topic"] == "cmd_vel_raw"
    assert monitor["cmd_vel_out_topic"] == "cmd_vel"
    assert monitor["PolygonLeftStop"]["motion_scope"] == "left"
    assert monitor["PolygonRightStop"]["motion_scope"] == "right"
    assert monitor["PolygonLeftStop"]["enabled"] is True
    assert monitor["PolygonRightStop"]["enabled"] is True
