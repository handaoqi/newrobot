import test from 'node:test'
import assert from 'node:assert/strict'

import {
  DEFAULT_LOOP_REST_SECONDS,
  LOOP_FAILURE_RETRY_MS,
  ensureGuardDutyLoopNavigationReady,
  guardDutyLoopRepairFailureMessage,
  isTransientNavigationFetchError,
  loopRestMilliseconds,
  restoreLoopRestSeconds,
  waitForGuardDutyLoopRepair,
} from '../src/services/guardDutyLoopNavRepair.js'

test('normal rest uses configured seconds', () => {
  assert.equal(loopRestMilliseconds(10), 10_000)
  assert.equal(loopRestMilliseconds(0), 0)
})

test('rest defaults to ten seconds and does not reinterpret legacy minutes', () => {
  assert.equal(DEFAULT_LOOP_REST_SECONDS, 10)
  assert.equal(restoreLoopRestSeconds({}), 10)
  assert.equal(restoreLoopRestSeconds({ restMinutes: 1 }), 10)
  assert.equal(restoreLoopRestSeconds({ restSeconds: 25 }), 25)
  assert.equal(restoreLoopRestSeconds({ restSeconds: -1 }), 10)
})

test('failed start/repair uses a short retry capped by configured rest', () => {
  assert.equal(loopRestMilliseconds(120, { shortRetry: true }), LOOP_FAILURE_RETRY_MS)
  assert.equal(loopRestMilliseconds(30, { shortRetry: true }), 30_000)
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

test('transient navigation fetch failures retry before deciding to repair', async () => {
  let fetches = 0
  let repairs = 0
  const progress = []
  const result = await ensureGuardDutyLoopNavigationReady({
    fetchStatus: async () => {
      fetches += 1
      if (fetches < 3) throw new TypeError('Failed to fetch')
      return { ready: true }
    },
    isReady: (status) => status.ready === true,
    repair: async () => {
      repairs += 1
      return { ready: true }
    },
    onProgress: (message) => progress.push(message),
    statusFetchRetryMs: 1,
    sleep: async () => {},
  })
  assert.equal(fetches, 3)
  assert.equal(repairs, 0)
  assert.equal(result.reason, 'already_ready')
  assert.equal(progress.filter((message) => message.includes('网络波动')).length, 2)
})

test('exhausted transient retries are reported as network instead of Nav2 failure', async () => {
  await assert.rejects(
    ensureGuardDutyLoopNavigationReady({
      fetchStatus: async () => { throw new TypeError('Failed to fetch') },
      isReady: () => false,
      statusFetchAttempts: 2,
      statusFetchRetryMs: 1,
      sleep: async () => {},
    }),
    (error) => {
      assert.equal(error.code, 'NAV_STATUS_UNREACHABLE')
      assert.match(guardDutyLoopRepairFailureMessage(error), /^网络连接暂时中断/)
      return true
    },
  )
})

test('real device errors retain the navigation repair failure label', () => {
  assert.equal(isTransientNavigationFetchError(new TypeError('Failed to fetch')), true)
  assert.equal(isTransientNavigationFetchError(new Error('Nav2 lifecycle node inactive')), false)
  assert.equal(
    guardDutyLoopRepairFailureMessage(new Error('Nav2 lifecycle node inactive')),
    '导航栈修复失败：Nav2 lifecycle node inactive，短间隔后重试',
  )
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

test('next round proceeds immediately when rest repair has finished', async () => {
  const result = await waitForGuardDutyLoopRepair({
    isBusy: () => false,
    fetchStatus: async () => {
      throw new Error('status fetch should not run')
    },
    isReady: () => false,
  })
  assert.deepEqual(result, {
    ok: true,
    supersede: false,
    status: null,
    reason: 'repair_finished',
  })
})

test('next round supersedes stale repair when device is already ready', async () => {
  let nowValue = 0
  const status = { nav_ready: true }
  const result = await waitForGuardDutyLoopRepair({
    isBusy: () => true,
    fetchStatus: async () => status,
    isReady: (value) => value.nav_ready === true,
    timeoutMs: 10,
    pollIntervalMs: 5,
    sleep: async () => {},
    now: () => {
      nowValue += 5
      return nowValue
    },
  })
  assert.equal(result.ok, true)
  assert.equal(result.supersede, true)
  assert.equal(result.status, status)
  assert.equal(result.reason, 'device_already_ready')
})

test('next round returns to retry when repair is busy and device is not ready', async () => {
  let nowValue = 0
  const result = await waitForGuardDutyLoopRepair({
    isBusy: () => true,
    fetchStatus: async () => ({ nav_ready: false }),
    isReady: (value) => value.nav_ready === true,
    timeoutMs: 10,
    pollIntervalMs: 5,
    sleep: async () => {},
    now: () => {
      nowValue += 5
      return nowValue
    },
  })
  assert.equal(result.ok, false)
  assert.equal(result.supersede, false)
  assert.equal(result.reason, 'repair_still_busy')
})
