import test from 'node:test'
import assert from 'node:assert/strict'

import {
  buildTaskExecutionTimeline,
  taskExecutionIsActive,
} from '../src/services/taskExecutionWaypointProgress.js'

function event(id, eventType, stateVersion, occurredAt, payload = {}, extra = {}) {
  return {
    id,
    event_type: eventType,
    state_version: stateVersion,
    occurred_at: occurredAt,
    payload,
    ...extra,
  }
}

test('builds an ordered real preview timeline from durable task events', () => {
  const execution = {
    state: 'completed',
    route_name: '南门路线',
    created_at: '2026-09-06T08:00:00+08:00',
    route_snapshot: {
      waypoints: [
        { map_point_number: 3, x: 3, y: 4 },
        { map_point_number: 1, x: 1, y: 2 },
      ],
    },
    events: [
      event(5, 'task.completed', 5, '2026-09-06T08:00:09+08:00'),
      event(3, 'task.waypoint_reached', 3, '2026-09-06T08:00:05+08:00', {
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 3, x: 3, y: 4 },
        robot_pose: { x: 3.02, y: 3.98 },
      }),
      event(1, 'task.created', 0, '2026-09-06T08:00:00+08:00'),
      event(2, 'task.target_dispatched', 2, '2026-09-06T08:00:02+08:00', {
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 3, x: 3, y: 4 },
      }),
      event(4, 'task.target_dispatched', 4, '2026-09-06T08:00:06+08:00', {
        execution_waypoint_index: 1,
        waypoint: { map_point_number: 1, x: 1, y: 2 },
      }),
    ],
  }

  const timeline = buildTaskExecutionTimeline(execution, {
    requestedAt: Date.parse('2026-09-06T08:00:00+08:00'),
  })

  assert.deepEqual(timeline.map(item => item.title), [
    '路线下发中',
    '预演任务已创建',
    '3号点目标已下发',
    '3号点已到达',
    '1号点目标已下发',
    '预演完成',
  ])
  assert.match(timeline[3].detail, /目标 x 3\.00 \/ y 4\.00/)
  assert.match(timeline[3].detail, /机器狗 x 3\.02 \/ y 3\.98/)
  assert.equal(timeline.at(-1).elapsedSeconds, 9)
})

test('falls back to execution index and removes duplicated events', () => {
  const duplicated = event('', 'task.target_dispatched', 2, '2026-09-06T08:00:02+08:00', {
    execution_waypoint_index: 1,
    waypoint: { x: 5, y: 6 },
  }, { message_id: 'same-message' })
  const timeline = buildTaskExecutionTimeline({
    state: 'running',
    route_snapshot: {
      waypoints: [
        { map_point_number: 7 },
        { map_point_number: 6 },
      ],
    },
    events: [duplicated, { ...duplicated }],
  })

  assert.equal(timeline.length, 1)
  assert.equal(timeline[0].title, '6号点目标已下发')
})

test('arrival confirmation shows correction mode, three retries and strict radius', () => {
  const timeline = buildTaskExecutionTimeline({
    state: 'running',
    route_snapshot: { waypoints: [{ map_point_number: 2 }] },
    events: [
      event(8, 'task.arrival_confirmed', 8, '2026-09-06T08:00:08+08:00', {
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 2, x: 2, y: 3 },
        arrival_mode: 'lightweight',
        localization_correction: 'skipped',
        distance_m: 0.29,
        acceptance_tolerance_m: 0.3,
        reapproach_attempts: 3,
        coarse_completed: false,
      }),
      event(9, 'task.waypoint_reached', 9, '2026-09-06T08:00:08+08:00', {
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 2, x: 2, y: 3 },
      }),
    ],
  })

  assert.equal(timeline.length, 1)
  assert.equal(timeline[0].title, '2号点验收完成')
  assert.match(timeline[0].detail, /轻量到达（跳过定位校正）/)
  assert.match(timeline[0].detail, /偏差 0\.29m/)
  assert.match(timeline[0].detail, /验收半径 0\.30m/)
  assert.match(timeline[0].detail, /追加靠近 3 次/)
})

test('shows a request failure before an execution exists', () => {
  const timeline = buildTaskExecutionTimeline(null, {
    requestedAt: Date.parse('2026-09-06T08:00:00+08:00'),
    requestError: '网络不可用',
  })

  assert.equal(timeline.length, 1)
  assert.equal(timeline[0].type, 'error')
  assert.equal(timeline[0].title, '预演下发失败')
  assert.equal(timeline[0].detail, '网络不可用')
})

test('identifies active and terminal execution states', () => {
  assert.equal(taskExecutionIsActive({ state: 'running' }), true)
  assert.equal(taskExecutionIsActive({ state: 'paused' }), true)
  assert.equal(taskExecutionIsActive({ state: 'completed' }), false)
  assert.equal(taskExecutionIsActive({ state: 'failed' }), false)
})
