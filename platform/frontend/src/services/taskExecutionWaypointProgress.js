import {
  GUARD_WAYPOINT_STATE,
  activeGuardDutyTarget,
  guardDutyWaypointStates,
} from '../utils/guardDutyWaypointState.js'

function milestoneEvents(execution) {
  return (execution?.events || [])
    .filter((event) => ['task.target_dispatched', 'task.waypoint_reached'].includes(event.event_type))
    .sort((left, right) => Number(left.state_version || 0) - Number(right.state_version || 0))
}

function mapPointNumberForIndex(waypoints, index) {
  const point = waypoints?.[index]
  if (!point) return index + 1
  if (point.map_point_number !== null && point.map_point_number !== undefined && point.map_point_number !== '') {
    return point.map_point_number
  }
  if (Number.isFinite(Number(point.sequence))) return Number(point.sequence) + 1
  return index + 1
}

/**
 * Resolve the live waypoint cursor for task execution UI.
 *
 * Prefer milestone events (same source as 巡检值守) so reverse loop routes
 * show map point numbers instead of the raw array index + 1.
 */
export function resolveTaskExecutionWaypointProgress(execution) {
  const waypoints = execution?.route_snapshot?.waypoints || []
  const milestones = milestoneEvents(execution)
  const states = guardDutyWaypointStates(waypoints, milestones)
  const activeTarget = activeGuardDutyTarget(milestones, waypoints, states)

  let currentIndex = Number(execution?.current_waypoint_index)
  if (!Number.isInteger(currentIndex) || currentIndex < 0) currentIndex = 0

  if (activeTarget) {
    const fromPayload = Number(activeTarget.payload?.execution_waypoint_index)
    if (Number.isInteger(fromPayload) && fromPayload >= 0 && fromPayload < waypoints.length) {
      currentIndex = fromPayload
    } else {
      const targetStateIndex = states.findIndex((state) => state === GUARD_WAYPOINT_STATE.target)
      if (targetStateIndex >= 0) currentIndex = targetStateIndex
    }
  } else if (waypoints.length) {
    currentIndex = Math.min(currentIndex, waypoints.length - 1)
  }

  const currentMapPointNumber = activeTarget?.payload?.waypoint?.map_point_number
    ?? mapPointNumberForIndex(waypoints, currentIndex)

  return {
    waypoints,
    milestones,
    states,
    activeTarget,
    currentIndex,
    currentMapPointNumber,
    totalWaypoints: Number(execution?.total_waypoints) || waypoints.length,
  }
}

export function taskExecutionWaypointClass({
  index,
  states = [],
  currentIndex = 0,
  failedIndexes = [],
  isActive = false,
}) {
  if (failedIndexes.includes(index)) return 'failed'
  const state = states[index]
  if (state === GUARD_WAYPOINT_STATE.reached) return 'done'
  if (state === GUARD_WAYPOINT_STATE.target && isActive) return 'current'
  if (!states.length || states.every((item) => item === GUARD_WAYPOINT_STATE.idle)) {
    if (index < currentIndex) return 'done'
    if (index === currentIndex && isActive) return 'current'
  }
  return ''
}

export function formatWaypointLabel(waypoints, index) {
  return `${mapPointNumberForIndex(waypoints, index)}号点`
}
