import assert from 'node:assert/strict'
import test from 'node:test'

import {
  calculateTrajectoryDistance,
  displayedGuardDutyDistance,
  guardDutyTrajectoryCaptureState,
} from '../src/utils/guardDutyDistance.js'

test('calculates valid same-map trajectory segments and skips jumps', () => {
  assert.equal(calculateTrajectoryDistance([
    { x: 0, y: 0, map_id: 1 },
    { x: 3, y: 4, map_id: 1 },
    { x: 20, y: 4, map_id: 1 },
    { x: 4, y: 4, map_id: 2 },
    { x: 4, y: 6, map_id: 2 },
  ]), 7)
})

test('falls back to a trajectory that arrived after a zero loop capture', () => {
  assert.equal(displayedGuardDutyDistance({
    currentDistance: 24.02,
    currentExecutionId: 'execution-1',
    executionLoopSessionId: 'loop-1',
    loopStartedAt: 100,
    loopSessionId: 'loop-1',
    loopAccumulatedDistance: 0,
    loopCountedExecutionIds: ['execution-1'],
  }), 24.02)
})

test('stale loop state does not replace a normal execution distance', () => {
  assert.equal(displayedGuardDutyDistance({
    currentDistance: 8.5,
    currentExecutionId: 'one-shot',
    loopStartedAt: 100,
    loopSessionId: 'old-loop',
    loopAccumulatedDistance: 31,
    loopCountedExecutionIds: ['old-execution'],
  }), 8.5)
})

test('captures zero distance immediately when the execution never started', () => {
  assert.deepEqual(guardDutyTrajectoryCaptureState({
    execution: {
      started_at: null,
      finished_at: '2026-09-11T00:14:13.702+08:00',
    },
    points: [],
    distance: 0,
    now: Date.parse('2026-09-11T00:14:14.000+08:00'),
  }), {
    ready: true,
    reason: 'execution_not_started',
    retryAfterMilliseconds: 0,
  })
})

test('waits briefly for the final trajectory batch of a started execution', () => {
  assert.deepEqual(guardDutyTrajectoryCaptureState({
    execution: {
      started_at: '2026-09-11T00:10:11.114+08:00',
      finished_at: '2026-09-11T00:12:04.614+08:00',
    },
    points: [],
    distance: 0,
    now: Date.parse('2026-09-11T00:12:09.614+08:00'),
  }), {
    ready: false,
    reason: 'sync_pending',
    retryAfterMilliseconds: 10_000,
  })
})

test('captures zero distance after the final trajectory grace period expires', () => {
  assert.deepEqual(guardDutyTrajectoryCaptureState({
    execution: {
      started_at: '2026-09-11T00:10:11.114+08:00',
      finished_at: '2026-09-11T00:12:04.614+08:00',
    },
    points: [{ x: 1, y: 1 }],
    distance: 0,
    now: Date.parse('2026-09-11T00:12:19.614+08:00'),
  }), {
    ready: true,
    reason: 'sync_grace_expired',
    retryAfterMilliseconds: 0,
  })
})

test('does not wait when a usable trajectory is already available', () => {
  assert.deepEqual(guardDutyTrajectoryCaptureState({
    execution: {
      started_at: '2026-09-11T00:10:11.114+08:00',
      finished_at: '2026-09-11T00:12:04.614+08:00',
    },
    points: [{ x: 1, y: 1 }, { x: 1, y: 1 }],
    distance: 0,
    now: Date.parse('2026-09-11T00:12:04.700+08:00'),
  }), {
    ready: true,
    reason: 'trajectory_ready',
    retryAfterMilliseconds: 0,
  })
})
