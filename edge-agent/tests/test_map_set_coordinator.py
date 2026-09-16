from roamerx_edge.map_set_coordinator import MapSetCoordinator


class FakeActivation:
    def __init__(self):
        self.commands = []

    def activate(self, command):
        self.commands.append(command)
        return command


class FakeNavigationStack:
    def __init__(self):
        self.switches = 0

    def switch_map(self):
        self.switches += 1


class LifecycleNavigationStack(FakeNavigationStack):
    def __init__(self):
        super().__init__()
        self.calls = []

    def deactivate_execution(self):
        self.calls.append("deactivate")

    def reload_map_if_running(self, pcd_path, yaml_path):
        self.calls.append(("reload", pcd_path, yaml_path))
        return {"deferred": False}

    def reload_boundary_filter(self):
        self.calls.append("boundary")

    def prepare(self, command):
        self.calls.append(("prepare", command["reason"]))

    def activate_execution(self):
        self.calls.append("activate")
        return {"action": "activate_execution"}


def _route_snapshot():
    return {
        "waypoints": [
            {"x": 10.0, "y": 0.0},
            {"x": 180.0, "y": 0.0},
            {"x": 230.0, "y": 0.0},
            {"x": 350.0, "y": 0.0},
        ],
        "map_set": {
            "submaps": [
                {"submap_id": "submap_001", "map_id": "11", "map_version": "v1", "local_map_dir": "/maps/1", "metadata": {"bounds": {"min_x": 0, "max_x": 220, "min_y": -10, "max_y": 10}}},
                {"submap_id": "submap_002", "map_id": "12", "map_version": "v1", "local_map_dir": "/maps/2", "metadata": {"bounds": {"min_x": 190, "max_x": 420, "min_y": -10, "max_y": 10}}},
            ]
        },
    }


def test_segments_route_by_submap_boundaries():
    coordinator = MapSetCoordinator(FakeActivation(), FakeNavigationStack())
    segments = coordinator.build_segments(_route_snapshot())
    assert [(item.submap_id, item.start_index, item.end_index) for item in segments] == [
        ("submap_001", 0, 2),
        ("submap_002", 2, 4),
    ]


def test_activate_switches_local_map_then_navigation_stack():
    activation = FakeActivation()
    navigation_stack = FakeNavigationStack()
    coordinator = MapSetCoordinator(activation, navigation_stack)
    segment = coordinator.build_segments(_route_snapshot())[0]
    status = coordinator.activate(segment)
    assert activation.commands[0]["local_map_dir"] == "/maps/1"
    assert navigation_stack.switches == 1
    assert status["current_submap_id"] == "submap_001"


def test_activate_uses_lifecycle_safe_reload_when_available():
    class Activation(FakeActivation):
        def activate(self, command):
            super().activate(command)
            return {"current_map": {"active_files": {
                "map.pcd": "/maps/1/map.pcd", "map.yaml": "/maps/1/map.yaml",
            }}}

    navigation_stack = LifecycleNavigationStack()
    coordinator = MapSetCoordinator(Activation(), navigation_stack)
    coordinator.activate(coordinator.build_segments(_route_snapshot())[0])

    assert navigation_stack.switches == 0
    assert navigation_stack.calls == [
        "deactivate",
        ("reload", "/maps/1/map.pcd", "/maps/1/map.yaml"),
        "boundary",
        ("prepare", "map_set_transition"),
    ]
    coordinator.activate_execution()
    assert navigation_stack.calls[-1] == "activate"
