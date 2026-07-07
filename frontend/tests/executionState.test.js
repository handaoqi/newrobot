import assert from 'node:assert/strict'
import test from 'node:test'

import { executionActions, powerLabel } from '../src/services/executionState.js'

test('terminal tasks cannot resume or cancel', () => {
  assert.deepEqual(executionActions('completed'), { pause: false, resume: false, cancel: false })
  assert.deepEqual(executionActions('cancelled'), { pause: false, resume: false, cancel: false })
})

test('only paused tasks can resume', () => {
  assert.equal(executionActions('paused').resume, true)
  assert.equal(executionActions('running').resume, false)
})

test('unknown power is explicit', () => {
  assert.equal(powerLabel({ power_available: false }), '未知')
  assert.equal(powerLabel({ power_available: true, battery_percent: 61 }), '61%')
})
