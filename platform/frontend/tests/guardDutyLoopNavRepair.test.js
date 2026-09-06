import test from 'node:test'
import assert from 'node:assert/strict'

import {
  LOOP_FAILURE_RETRY_MS,
  ensureGuardDutyLoopNavigationReady,
  loopRestMilliseconds,
} from '../src/services/guardDutyLoopNavRepair.js'

test('normal rest uses configured minutes', () => {
  assert.equal(loopRestMilliseconds(5), 5 * 60 * 1000)
  assert.equal(loopRestMilliseconds(0), 0)
})

test('failed start/repair uses a short retry capped by configured rest', () => {
  assert.equal(loopRestMilliseconds(10, { shortRetry: true }), LOOP_FAILURE_RETRY_MS)
  assert.equal(loopRestMilliseconds(0.5, { shortRetry: true }), 30_000)
  assert.equal(loopRestMilliseconds(0, { shortRetry: true }), LOOP_FAILURE_RETRY_MS)
})

test('ensure skips repair when already ready', async () => {
  const calls = []
  const result = await ensureGuardDutyLoopNavigationReady({
    fetchStatus: async () => {
      calls.push('fetch')
      return { ready: true }
    },
    isReady: (status) => status.ready === true,
    repair: async () => {
      calls.push('repair')
      return { ready: true }
    },
  })
  assert.deepEqual(calls, ['fetch'])
  assert.equal(result.ok, true)
  assert.equal(result.repaired, false)
  assert.equal(result.reason, 'already_ready')
})

test('ensure repairs and polls until ready', async () => {
  let fetches = 0
  const progress = []
  const result = await ensureGuardDutyLoopNavigationReady({
    fetchStatus: async () => {
      fetches += 1
      return { ready: fetches >= 3 }
    },
    isReady: (status) => status.ready === true,
    repair: async () => ({ ready: false }),
    onProgress: (message) => progress.push(message),
    readyTimeoutMs: 5_000,
    pollIntervalMs: 1,
    sleep: async () => {},
    now: (() => {
      let t = 0
      return () => {
        t += 10
        return t
      }
    })(),
  })
  assert.equal(result.ok, true)
  assert.equal(result.repaired, true)
  assert.equal(result.reason, 'ready_after_wait')
  assert.ok(progress.some((item) => item.includes('正在修复')))
  assert.ok(progress.some((item) => item.includes('已就绪')))
})

test('ensure reports failure when repair never becomes ready', async () => {
  const result = await ensureGuardDutyLoopNavigationReady({
    fetchStatus: async () => ({ ready: false }),
    isReady: () => false,
    repair: async () => ({ ready: false }),
    readyTimeoutMs: 30,
    pollIntervalMs: 1,
    sleep: async () => {},
    now: (() => {
      let t = 0
      return () => {
        const value = t
        t += 20
        return value
      }
    })(),
  })
  assert.equal(result.ok, false)
  assert.equal(result.repaired, true)
  assert.equal(result.reason, 'not_ready_after_repair')
})
