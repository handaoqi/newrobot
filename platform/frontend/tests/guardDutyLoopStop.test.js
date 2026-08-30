import test from 'node:test'
import assert from 'node:assert/strict'

import {
  clearGuardDutyLoopExecution,
  guardDutyLoopCleanupExecutionId,
} from '../src/utils/guardDutyLoopStop.js'

test('loop stop prefers its persisted execution id over a stale visible execution', () => {
  assert.equal(
    guardDutyLoopCleanupExecutionId('loop-execution', { id: 'visible-execution' }),
    'loop-execution',
  )
  assert.equal(guardDutyLoopCleanupExecutionId('', { id: 'visible-execution' }), 'visible-execution')
})

test('loop stop force-exits the execution so cloud active state is cleared', async () => {
  const calls = []
  const result = await clearGuardDutyLoopExecution(async (...args) => {
    calls.push(args)
    return { id: args[0], state: 'cancelled' }
  }, 'loop-execution')

  assert.deepEqual(calls, [['loop-execution', 'force-exit']])
  assert.equal(result.state, 'cancelled')
})
