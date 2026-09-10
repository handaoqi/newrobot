import assert from 'node:assert/strict'
import test from 'node:test'

import { fetchRobotNavigationStatus } from '../src/services/api.js'

test('navigation status summary uses compact endpoint and an eight second timeout', async () => {
  const originalFetch = globalThis.fetch
  const originalLocalStorage = globalThis.localStorage
  const calls = []
  globalThis.localStorage = {
    getItem: () => null,
    removeItem: () => {},
  }
  globalThis.fetch = async (url, options) => {
    calls.push({ url, options })
    return {
      ok: true,
      status: 200,
      json: async () => ({}),
    }
  }

  try {
    await fetchRobotNavigationStatus(7, { summary: true })
    await fetchRobotNavigationStatus(7)
  } finally {
    globalThis.fetch = originalFetch
    globalThis.localStorage = originalLocalStorage
  }

  assert.equal(calls[0].url, '/api/robots/7/navigation/status/?view=summary')
  assert.ok(calls[0].options.signal instanceof AbortSignal)
  assert.equal(calls[1].url, '/api/robots/7/navigation/status/')
  assert.equal(calls[1].options.signal, undefined)
})
