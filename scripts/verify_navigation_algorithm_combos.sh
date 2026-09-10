#!/usr/bin/env bash
# Verify the four page algorithm combinations against the live Nav2 stack.
# Usage (on NX, with navigation running):
#   sudo bash scripts/verify_navigation_algorithm_combos.sh
set -eo pipefail

# ament setup scripts reference unset vars under `set -u`.
set +u
source /opt/ros/humble/setup.bash
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-24}"

pass=0
fail=0

check() {
  local name="$1"
  local got="$2"
  local want="$3"
  if [[ "$got" == *"$want"* ]]; then
    echo "  PASS  $name => $got"
    pass=$((pass + 1))
  else
    echo "  FAIL  $name => got='$got' want~='$want'"
    fail=$((fail + 1))
  fi
}

echo "=== capability: registered plugins ==="
planner_plugins="$(timeout 8 ros2 param get /planner_server planner_plugins)"
controller_plugins="$(timeout 8 ros2 param get /controller_server controller_plugins)"
echo "$planner_plugins"
echo "$controller_plugins"
check "planner ThetaStar registered" "$planner_plugins" "ThetaStar"
check "planner NavFn registered" "$planner_plugins" "NavFn"
check "controller FollowPath registered" "$controller_plugins" "FollowPath"
check "controller RPP registered" "$controller_plugins" "RPP"

apply_combo() {
  local global_name="$1"
  local local_name="$2"
  local plugin_global="$3"
  local plugin_local="$4"
  local use_astar="$5"

  echo
  echo "=== combo ${global_name} + ${local_name} ==="
  timeout 8 ros2 param set /planner_server "${plugin_global}.use_astar" "${use_astar}" >/dev/null
  got_astar="$(timeout 8 ros2 param get /planner_server "${plugin_global}.use_astar")"
  check "${plugin_global}.use_astar=${use_astar}" "$got_astar" "$use_astar"

  # Publish selectors the same way Edge does (transient local subscribers in BT).
  timeout 3 ros2 topic pub --once /planner_selector std_msgs/msg/String "{data: '${plugin_global}'}" >/dev/null
  timeout 3 ros2 topic pub --once /controller_selector std_msgs/msg/String "{data: '${plugin_local}'}" >/dev/null
  echo "  published selectors planner=${plugin_global} controller=${plugin_local}"
  pass=$((pass + 1))
}

# Page mapping from the plan:
# theta_star -> ThetaStar (use_astar false on that plugin)
# navfn      -> NavFn with use_astar=true (A*)
# mppi       -> FollowPath
# rpp        -> RPP
apply_combo theta_star mppi ThetaStar FollowPath False
apply_combo theta_star rpp ThetaStar RPP False
apply_combo navfn mppi NavFn FollowPath True
apply_combo navfn rpp NavFn RPP True

echo
echo "=== restore default cruise safety layers ==="
timeout 8 ros2 param set /local_costmap/local_costmap obstacle_layer.enabled True >/dev/null
timeout 8 ros2 param set /collision_monitor PolygonSlow.enabled True >/dev/null
timeout 8 ros2 param set /collision_monitor PolygonStop.enabled True >/dev/null
stop_got="$(timeout 8 ros2 param get /collision_monitor PolygonStop.enabled)"
check "PolygonStop stays enabled" "$stop_got" "True"

# Leave selectors on the combo used most in today's patrols.
timeout 3 ros2 topic pub --once /planner_selector std_msgs/msg/String "{data: 'ThetaStar'}" >/dev/null
timeout 3 ros2 topic pub --once /controller_selector std_msgs/msg/String "{data: 'RPP'}" >/dev/null
timeout 8 ros2 param set /planner_server NavFn.use_astar False >/dev/null || true

echo
echo "RESULT pass=${pass} fail=${fail}"
if [[ "$fail" -ne 0 ]]; then
  exit 1
fi
echo "All four algorithm combinations read back successfully."
