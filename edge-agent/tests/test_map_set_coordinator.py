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
