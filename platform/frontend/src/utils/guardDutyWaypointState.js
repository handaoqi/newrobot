export const GUARD_WAYPOINT_STATE = Object.freeze({
  idle: 'idle',
  target: 'target',
  reached: 'reached',
})

function eventWaypointIndex(event, waypoints) {
  const rawIndex = event?.payload?.execution_waypoint_index
  if (rawIndex !== null && rawIndex !== undefined && rawIndex !== '') {
    const index = Number(rawIndex)
    if (Number.isInteger(index) && index >= 0 && index < waypoints.length) return index
  }

  const waypointId = String(
    event?.payload?.waypoint?.waypoint_id
    || event?.payload?.current_waypoint_id
    || '',
  )
  if (!waypointId) return -1
  return waypoints.findIndex((waypoint) => String(waypoint?.waypoint_id || '') === waypointId)
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
