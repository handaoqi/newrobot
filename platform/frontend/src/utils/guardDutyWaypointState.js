export const GUARD_WAYPOINT_STATE = Object.freeze({
  idle: 'idle',
  target: 'target',
  reached: 'reached',
})

function eventWaypointIndex(event, waypoints) {
  const payload = event?.payload || {}
  const waypointId = String(
    payload?.waypoint?.waypoint_id
    || payload?.current_waypoint_id
    || '',
  )
  if (waypointId) {
    const matches = waypoints
      .map((waypoint, index) => String(waypoint?.waypoint_id || '') === waypointId ? index : -1)
      .filter((index) => index >= 0)
    if (matches.length === 1) return matches[0]
  }

  const mapPointNumber = payload?.waypoint?.map_point_number
  if (mapPointNumber !== null && mapPointNumber !== undefined && mapPointNumber !== '') {
    const matches = waypoints
      .map((waypoint, index) => String(waypoint?.map_point_number ?? '') === String(mapPointNumber) ? index : -1)
      .filter((index) => index >= 0)
    if (matches.length === 1) return matches[0]
  }

  const rawIndex = payload?.execution_waypoint_index
  if (rawIndex !== null && rawIndex !== undefined && rawIndex !== '') {
    const index = Number(rawIndex)
    if (Number.isInteger(index) && index >= 0 && index < waypoints.length) return index
  }

  return -1
}

export function guardDutyExecutionWaypointPlan(waypoints = [], executionOrder = []) {
  const order = Array.isArray(executionOrder) && executionOrder.length
    ? executionOrder
    : waypoints.map((waypoint, index) => waypoint?.map_point_number
      ?? (Number.isFinite(Number(waypoint?.sequence)) ? Number(waypoint.sequence) + 1 : index + 1))
  const unusedIndexes = new Set(waypoints.map((_, index) => index))

  return order.map((mapPointNumber, executionIndex) => {
    const matches = waypoints
      .map((waypoint, index) => (
        unusedIndexes.has(index)
        && String(waypoint?.map_point_number ?? '') === String(mapPointNumber)
          ? index
          : -1
      ))
      .filter((index) => index >= 0)
    const waypointIndex = matches[0] ?? (unusedIndexes.has(executionIndex) ? executionIndex : -1)
    if (waypointIndex >= 0) unusedIndexes.delete(waypointIndex)
    return { mapPointNumber, waypointIndex }
  })
}

export function guardDutyWaypointStates(waypoints = [], milestones = []) {
  const states = waypoints.map(() => GUARD_WAYPOINT_STATE.idle)
  milestones.forEach((event) => {
    const index = eventWaypointIndex(event, waypoints)
    if (index < 0) return
    if (event.event_type === 'task.target_dispatched') states[index] = GUARD_WAYPOINT_STATE.target
    if (event.event_type === 'task.waypoint_reached') states[index] = GUARD_WAYPOINT_STATE.reached
  })
  return states
}

export function guardDutyRouteState(executionState = '') {
  return executionState === 'completed'
    ? GUARD_WAYPOINT_STATE.reached
    : GUARD_WAYPOINT_STATE.idle
}

export function activeGuardDutyTarget(milestones = [], waypoints = [], states = []) {
  return [...milestones].reverse().find((event) => {
    if (event.event_type !== 'task.target_dispatched') return false
    const index = eventWaypointIndex(event, waypoints)
    return index >= 0 && states[index] === GUARD_WAYPOINT_STATE.target
  }) || null
}
