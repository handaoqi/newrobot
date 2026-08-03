import assert from 'node:assert/strict'
import test from 'node:test'

import { executionActions, isExecutionActive, powerLabel } from '../src/services/executionState.js'

test('terminal tasks disable the combined control but keep force exit available', () => {
  assert.deepEqual(executionActions('completed').control, { enabled: false, action: null, label: '任务结束' })
  assert.deepEqual(executionActions('cancelled').control, { enabled: false, action: null, label: '任务结束' })
  assert.equal(executionActions('completed').forceExit, true)
})

test('combined control follows running and paused states', () => {
  assert.deepEqual(executionActions('running').control, { enabled: true, action: 'pause', label: '暂停' })
  assert.deepEqual(executionActions('paused').control, { enabled: true, action: 'resume', label: '继续' })
  assert.deepEqual(executionActions('accepted').control, { enabled: true, action: 'pause', label: '暂停' })
  assert.deepEqual(executionActions('interrupted').control, { enabled: true, action: 'resume', label: '继续' })
  assert.equal(executionActions('running').statusLabel, '任务执行中')
  assert.equal(executionActions('paused').statusLabel, '任务暂停中')
  assert.equal(executionActions('cancelling').statusLabel, '任务退出中')
})

test('all persisted in-flight states are active', () => {
  for (const state of ['created', 'dispatching', 'accepted', 'running', 'pausing', 'paused', 'resuming', 'cancelling', 'interrupted']) {
    assert.equal(isExecutionActive(state), true)
  }
  assert.equal(isExecutionActive('failed'), false)
})

test('unknown power is explicit', () => {
  assert.equal(powerLabel({ power_available: false }), '未知')
  assert.equal(powerLabel({ power_available: true, battery_percent: 61 }), '61%')
})
