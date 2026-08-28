import test from 'node:test'
import assert from 'node:assert/strict'

import {
  activeGuardDutyTarget,
  guardDutyRouteState,
  guardDutyWaypointStates,
} from '../src/utils/guardDutyWaypointState.js'

const waypoints = [
  { waypoint_id: 'wp-1', map_point_number: 1 },
  { waypoint_id: 'wp-2', map_point_number: 2 },
]

function milestone(eventType, waypointIndex, waypointId, stateVersion) {
  return {
    event_type: eventType,
    state_version: stateVersion,
    payload: {
      execution_waypoint_index: waypointIndex,
      current_waypoint_id: waypointId,
      waypoint: { waypoint_id: waypointId },
    },
  }
}

test('a newly started task keeps every waypoint blue until a target is dispatched', () => {
  assert.deepEqual(guardDutyWaypointStates(waypoints, []), ['idle', 'idle'])
  assert.equal(guardDutyRouteState([]), 'idle')
})

test('target dispatch changes only that waypoint to yellow', () => {
  const events = [milestone('task.target_dispatched', 0, 'wp-1', 1)]
  assert.deepEqual(guardDutyWaypointStates(waypoints, events), ['target', 'idle'])
  assert.equal(guardDutyRouteState(events), 'target')
})

test('target arrival changes the waypoint from yellow to green', () => {
  const events = [
    milestone('task.target_dispatched', 0, 'wp-1', 1),
    milestone('task.waypoint_reached', 0, 'wp-1', 2),
  ]
  const states = guardDutyWaypointStates(waypoints, events)
  assert.deepEqual(states, ['reached', 'idle'])
  assert.equal(guardDutyRouteState(events), 'reached')
  assert.equal(activeGuardDutyTarget(events, waypoints, states), null)
})

test('execution waypoint index distinguishes repeated waypoint ids', () => {
  const repeatedWaypoints = [waypoints[0], waypoints[1], { ...waypoints[0] }]
  const events = [
    milestone('task.target_dispatched', 0, 'wp-1', 1),
    milestone('task.waypoint_reached', 0, 'wp-1', 2),
    milestone('task.target_dispatched', 2, 'wp-1', 3),
  ]
  const states = guardDutyWaypointStates(repeatedWaypoints, events)
  assert.deepEqual(states, ['reached', 'idle', 'target'])
  assert.equal(activeGuardDutyTarget(events, repeatedWaypoints, states), events[2])
})
