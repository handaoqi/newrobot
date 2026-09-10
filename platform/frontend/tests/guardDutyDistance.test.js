import assert from 'node:assert/strict'
import test from 'node:test'

import {
  calculateTrajectoryDistance,
  displayedGuardDutyDistance,
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
