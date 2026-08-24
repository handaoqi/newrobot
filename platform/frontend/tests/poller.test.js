import assert from 'node:assert/strict'
import test from 'node:test'

import { createAsyncPoller } from '../src/composables/useAsyncPoller.js'

const wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms))

test('async poller never overlaps executions', async () => {
  let active = 0
  let maxActive = 0
  let calls = 0
  const poller = createAsyncPoller(async () => {
    active += 1
    maxActive = Math.max(maxActive, active)
    calls += 1
    await wait(15)
    active -= 1
  }, { intervalMs: 1 })
  poller.start()
  await wait(60)
  poller.stop()
  assert.equal(maxActive, 1)
  assert.ok(calls >= 2)
})

test('stopping a poller aborts the active request and prevents rescheduling', async () => {
  let signal
  let aborted = false
  const poller = createAsyncPoller((requestSignal) => {
    signal = requestSignal
    return new Promise((resolve) => requestSignal.addEventListener('abort', () => {
      aborted = true
      resolve()
    }))
  }, { intervalMs: 1 })
  poller.start()
  await wait(5)
  poller.stop()
  await wait(10)
  assert.ok(signal)
  assert.equal(signal.aborted, true)
  assert.equal(aborted, true)
  assert.equal(poller.running, false)
})
