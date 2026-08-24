import assert from 'node:assert/strict'
import test from 'node:test'

import { createTTLCache } from '../src/services/cache.js'

test('cache de-duplicates in-flight loaders and cleans up after rejection', async () => {
  let calls = 0
  let rejectLoader
  const cache = createTTLCache({ ttlMs: 1000, now: () => 0 })
  const loader = () => {
    calls += 1
    return new Promise((resolve, reject) => { rejectLoader = reject })
  }
  const first = cache.get('robots', loader)
  const second = cache.get('robots', loader)
  assert.strictEqual(first, second)
  await Promise.resolve()
  rejectLoader(new Error('expected'))
  await assert.rejects(first, /expected/)
  assert.equal(cache.inFlightSize, 0)
  const retry = cache.get('robots', loader)
  await Promise.resolve()
  rejectLoader(new Error('retry'))
  await assert.rejects(retry, /retry/)
  assert.equal(calls, 2)
})

test('cache expires values and supports explicit or global invalidation', async () => {
  let clock = 10
  let calls = 0
  const cache = createTTLCache({ ttlMs: 50, now: () => clock })
  const loader = async () => ({ calls: ++calls })
  assert.deepEqual(await cache.get('maps', loader), { calls: 1 })
  assert.deepEqual(await cache.get('maps', loader), { calls: 1 })
  clock = 61
  assert.deepEqual(await cache.get('maps', loader), { calls: 2 })
  cache.invalidate('maps')
  assert.deepEqual(await cache.get('maps', loader), { calls: 3 })
  cache.invalidate()
  assert.equal(cache.size, 0)
})

test('global invalidation prevents an older in-flight loader from repopulating the cache', async () => {
  let resolveLoader
  let calls = 0
  const cache = createTTLCache({ ttlMs: 1000, now: () => 0 })
  const pending = cache.get('routes', () => {
    calls += 1
    return new Promise((resolve) => { resolveLoader = resolve })
  })
  await Promise.resolve()
  cache.clear()
  resolveLoader(['stale'])
  assert.deepEqual(await pending, ['stale'])
  assert.equal(cache.size, 0)
  assert.deepEqual(await cache.get('routes', async () => {
    calls += 1
    return ['fresh']
  }), ['fresh'])
  assert.equal(calls, 2)
})
