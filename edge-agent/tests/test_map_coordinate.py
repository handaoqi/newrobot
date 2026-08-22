from roamerx_edge.map_coordinate import (
    MAP_LOCAL_ONLY_NDT_ONLY,
    MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN,
    MAP_LOCAL_ONLY_TRANSITION_FORBIDDEN,
    MapConstraintError,
    constraints_from_manifest,
    new_map_constraints,
    validate_route_against_map,
)
import pytest


def test_new_map_without_rtk_is_indoor_ndt_only():
    constraints = new_map_constraints(gnss_origin={}, requested_scene_scope="outdoor")
    assert constraints["coordinate_mode"] == "local_only"
    assert constraints["scene_scope"] == "indoor"
    assert constraints["localization_mode"] == "ndt"


def test_new_map_with_locked_origin_can_be_outdoor():
    constraints = new_map_constraints(
        gnss_origin={"alignment_locked": 1, "origin_latitude": 39.0},
        requested_scene_scope="outdoor",
    )
    assert constraints["coordinate_mode"] == "rtk_fixed"
    assert constraints["scene_scope"] == "outdoor"
    assert constraints["localization_mode"] == "rtk_ndt"


def test_legacy_maps_without_manifest_are_not_forced_local_only():
    constraints = constraints_from_manifest({})
    assert constraints["coordinate_mode"] == ""
    assert constraints["origin_status"] == "legacy_incomplete"
    validate_route_against_map(constraints, scene_scope="outdoor", waypoints=[{"localization_mode": "rtk"}])


def test_local_only_route_rejects_outdoor_and_rtk():
    constraints = new_map_constraints(gnss_origin={}, requested_scene_scope="indoor")
    with pytest.raises(MapConstraintError) as outdoor:
        validate_route_against_map(constraints, scene_scope="outdoor", waypoints=[{"localization_mode": "ndt"}])
    assert outdoor.value.code == MAP_LOCAL_ONLY_OUTDOOR_FORBIDDEN
    with pytest.raises(MapConstraintError) as transition:
        validate_route_against_map(constraints, scene_scope="transition", waypoints=[{"localization_mode": "ndt"}])
    assert transition.value.code == MAP_LOCAL_ONLY_TRANSITION_FORBIDDEN
    with pytest.raises(MapConstraintError) as rtk:
        validate_route_against_map(constraints, scene_scope="indoor", waypoints=[{"localization_mode": "rtk"}])
    assert rtk.value.code == MAP_LOCAL_ONLY_NDT_ONLY
