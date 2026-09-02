import test from 'node:test'
import assert from 'node:assert/strict'

import {
  formatWaypointLabel,
  resolveTaskExecutionWaypointProgress,
  taskExecutionWaypointClass,
} from '../src/services/taskExecutionWaypointProgress.js'

const reverseWaypoints = [
  { waypoint_id: 'wp-7', map_point_number: 7, name: '点7', x: 1, y: 1 },
  { waypoint_id: 'wp-6', map_point_number: 6, name: '点6', x: 2, y: 2 },
  { waypoint_id: 'wp-5', map_point_number: 5, name: '点5', x: 3, y: 3 },
  { waypoint_id: 'wp-4', map_point_number: 4, name: '点4', x: 4, y: 4 },
]

test('reverse loop shows map point number instead of array index + 1', () => {
  const progress = resolveTaskExecutionWaypointProgress({
    current_waypoint_index: 2,
    total_waypoints: 4,
    route_snapshot: { waypoints: reverseWaypoints },
    events: [
      {
        event_type: 'task.target_dispatched',
        state_version: 1,
        payload: {
          execution_waypoint_index: 0,
          waypoint: { waypoint_id: 'wp-7', map_point_number: 7 },
        },
      },
      {
        event_type: 'task.waypoint_reached',
        state_version: 2,
        payload: {
          execution_waypoint_index: 0,
          waypoint: { waypoint_id: 'wp-7', map_point_number: 7 },
        },
      },
      {
        event_type: 'task.target_dispatched',
        state_version: 3,
        payload: {
          execution_waypoint_index: 2,
          waypoint: { waypoint_id: 'wp-5', map_point_number: 5 },
        },
      },
    ],
  })

  assert.equal(progress.currentIndex, 2)
  assert.equal(progress.currentMapPointNumber, 5)
  assert.equal(formatWaypointLabel(reverseWaypoints, 2), '5号点')
  assert.equal(
    taskExecutionWaypointClass({
      index: 2,
      states: progress.states,
      currentIndex: progress.currentIndex,
      isActive: true,
    }),
    'current',
  )
  assert.equal(
    taskExecutionWaypointClass({
      index: 0,
      states: progress.states,
      currentIndex: progress.currentIndex,
      isActive: true,
    }),
    'done',
  )
})

test('falls back to current_waypoint_index map number when milestones are missing', () => {
  const progress = resolveTaskExecutionWaypointProgress({
    current_waypoint_index: 2,
    total_waypoints: 4,
    route_snapshot: { waypoints: reverseWaypoints },
    events: [],
  })
  assert.equal(progress.currentIndex, 2)
  assert.equal(progress.currentMapPointNumber, 5)
})
