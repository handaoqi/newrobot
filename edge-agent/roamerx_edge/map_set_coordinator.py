from __future__ import annotations

from dataclasses import dataclass

from .protocol import ProtocolError


@dataclass(frozen=True)
class RouteSegment:
    submap_id: str
    start_index: int
    end_index: int
    waypoints: list[dict]
    submap: dict


class MapSetCoordinator:
    """Select local submaps for a global-frame patrol route.

    Submap PGM/PCD files retain the original map frame coordinates. This keeps
    route waypoints stable across a switch; only the NDT/static map target is
    replaced while the robot is stopped at a segment boundary.
    """

    def __init__(self, map_activation, navigation_stack) -> None:
        self.map_activation = map_activation
        self.navigation_stack = navigation_stack
        self.current_submap_id = ""
        self.next_submap_id = ""
        self.switch_state = "idle"
        self.failure_reason = ""

    def build_segments(self, route_snapshot: dict) -> list[RouteSegment]:
        map_set = route_snapshot.get("map_set") or {}
        submaps = list(map_set.get("submaps") or [])
        if not submaps:
            return []
        by_id = {str(item["submap_id"]): item for item in submaps}
        assignments = []
        for index, waypoint in enumerate(route_snapshot.get("waypoints") or []):
            submap_id = str(waypoint.get("submap_id") or self._submap_for_point(waypoint, submaps) or "")
            if not submap_id or submap_id not in by_id:
                raise ProtocolError("MAP_SET_ROUTE_OUT_OF_BOUNDS", f"waypoint {index} is outside every submap")
            assignments.append(submap_id)
        segments = []
        start = 0
        for index in range(1, len(assignments) + 1):
            if index == len(assignments) or assignments[index] != assignments[start]:
                submap_id = assignments[start]
                segments.append(RouteSegment(submap_id, start, index, route_snapshot["waypoints"][start:index], by_id[submap_id]))
                start = index
        return segments

    def activate(self, segment: RouteSegment) -> dict:
        self.switch_state = "switching"
        self.next_submap_id = segment.submap_id
        local_map_dir = str(segment.submap.get("local_map_dir") or "")
        if not local_map_dir:
            raise ProtocolError("MAP_SET_SOURCE_MISSING", f"submap {segment.submap_id} has no local map directory")
        try:
            self.map_activation.activate(
                {
                    "map_id": str(segment.submap["map_id"]),
                    "map_version": str(segment.submap.get("map_version") or ""),
                    "map_name": segment.submap_id,
                    "local_map_dir": local_map_dir,
                }
            )
            self.navigation_stack.switch_map()
        except Exception as exc:
            self.switch_state = "failed"
            self.failure_reason = str(exc)
            raise
        self.current_submap_id = segment.submap_id
        self.next_submap_id = ""
        self.switch_state = "ready"
        self.failure_reason = ""
        return self.status()

    def status(self) -> dict:
        return {
            "current_submap_id": self.current_submap_id,
            "next_submap_id": self.next_submap_id,
            "switch_state": self.switch_state,
            "failure_reason": self.failure_reason,
        }

    @staticmethod
    def _submap_for_point(waypoint: dict, submaps: list[dict]) -> str | None:
        x, y = float(waypoint["x"]), float(waypoint["y"])
        for submap in submaps:
            bounds = (submap.get("metadata") or {}).get("bounds") or submap.get("bounds") or {}
            if bounds and bounds["min_x"] <= x <= bounds["max_x"] and bounds["min_y"] <= y <= bounds["max_y"]:
                return str(submap["submap_id"])
        return None
