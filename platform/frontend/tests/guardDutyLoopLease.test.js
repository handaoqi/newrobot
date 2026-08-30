import test from 'node:test'
import assert from 'node:assert/strict'

import {
  acquireGuardDutyLoopLease,
  releaseGuardDutyLoopLease,
  renewGuardDutyLoopLease,
} from '../src/utils/guardDutyLoopLease.js'

function memoryStorage() {
  const values = new Map()
  return {
    getItem: (key) => values.get(key) ?? null,
    setItem: (key, value) => values.set(key, String(value)),
    removeItem: (key) => values.delete(key),
  }
}

test('only one guard duty tab owns an unexpired loop lease', () => {
  const storage = memoryStorage()
  assert.equal(acquireGuardDutyLoopLease(storage, 'robot-1', 'tab-a', 1000, 5000), true)
  assert.equal(acquireGuardDutyLoopLease(storage, 'robot-1', 'tab-b', 2000, 5000), false)
  assert.equal(renewGuardDutyLoopLease(storage, 'robot-1', 'tab-a', 3000, 5000), true)
  assert.equal(renewGuardDutyLoopLease(storage, 'robot-1', 'tab-b', 3000, 5000), false)
})

test('another tab can take over an expired or explicitly released lease', () => {
  const storage = memoryStorage()
  acquireGuardDutyLoopLease(storage, 'robot-1', 'tab-a', 1000, 1000)
  assert.equal(acquireGuardDutyLoopLease(storage, 'robot-1', 'tab-b', 2001, 1000), true)
  assert.equal(releaseGuardDutyLoopLease(storage, 'robot-1', 'tab-a'), false)
  assert.equal(releaseGuardDutyLoopLease(storage, 'robot-1', 'tab-b'), true)
  assert.equal(acquireGuardDutyLoopLease(storage, 'robot-1', 'tab-a', 2100, 1000), true)
})
