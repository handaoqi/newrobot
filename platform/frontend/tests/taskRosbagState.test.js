import assert from 'node:assert/strict'
import test from 'node:test'

import { resolveTaskRosbagStatus } from '../src/services/taskRosbagState.js'

test('active task uses the latest running rosbag event', () => {
  const status = resolveTaskRosbagStatus({
    state: 'running',
    events: [{
      received_at: '2026-08-30T12:00:00Z',
      payload: { rosbag: { running: true, bag_dir: '/bags/round-1', started_at_unix: 100 } },
    }],
    commands: [],
  })
  assert.equal(status.running, true)
  assert.equal(status.bag_dir, '/bags/round-1')
})

test('terminal command result supersedes an older running event', () => {
  const status = resolveTaskRosbagStatus({
    state: 'cancelled',
    finished_at: '2026-08-30T12:02:00Z',
    events: [{
      received_at: '2026-08-30T12:00:00Z',
      payload: { rosbag: { running: true, bag_dir: '/bags/round-1' } },
    }],
    commands: [{
      finished_at: '2026-08-30T12:02:01Z',
      result_payload: { rosbag: { running: false, bag_dir: '/bags/round-1', duration_seconds: 120 } },
    }],
  })
  assert.equal(status.running, false)
  assert.equal(status.duration_seconds, 120)
})

test('terminal task cannot keep displaying a stale running snapshot', () => {
  const status = resolveTaskRosbagStatus({
    state: 'completed',
    finished_at: '1970-01-01T00:02:00Z',
    events: [{
      received_at: '1970-01-01T00:01:40Z',
      payload: { rosbag: { running: true, bag_dir: '/bags/round-1', started_at_unix: 100 } },
    }],
    commands: [],
  }, 130_000)
  assert.equal(status.running, false)
  assert.equal(status.duration_seconds, 20)
})

test('task without rosbag snapshots does not show a recording card', () => {
  assert.equal(resolveTaskRosbagStatus({ state: 'running', events: [], commands: [] }), null)
  assert.equal(resolveTaskRosbagStatus({
    state: 'completed',
    events: [],
    commands: [{ result_payload: { rosbag: {} } }],
  }), null)
})
