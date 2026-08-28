from roamerx_patrol_demo.bag_contract import REQUIRED_TOPICS, validate_topic_names


def test_demo_contract_contains_required_standard_topics():
    expected = {
        "/clock",
        "/map",
        "/tf",
        "/tf_static",
        "/odom",
        "/cmd_vel",
        "/scan",
        "/camera/front/image/compressed",
        "/imu/data",
        "/battery_state",
        "/diagnostics",
        "/plan",
        "/goal_pose",
        "/patrol/trajectory",
        "/patrol/status",
    }
    assert expected <= set(REQUIRED_TOPICS)


def test_namespace_is_applied_without_double_slashes():
    names = validate_topic_names("/demo", ["/scan", "patrol/status"])
    assert names == ["/demo/scan", "/demo/patrol/status"]
