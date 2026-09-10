import assert from 'node:assert/strict'
import test from 'node:test'

import {
  ASSET_LOAD_RETRY_KEY,
  claimAssetLoadRetry,
  clearAssetLoadRetry,
  isAssetLoadError,
} from '../src/services/assetLoadRecovery.js'

function createStorage() {
  const values = new Map()
  return {
    getItem: (key) => values.get(key) || null,
    setItem: (key, value) => values.set(key, value),
    removeItem: (key) => values.delete(key),
  }
}

test('recognizes lazy JavaScript and CSS asset load failures', () => {
  assert.equal(isAssetLoadError(new Error('Failed to fetch dynamically imported module')), true)
  assert.equal(isAssetLoadError(new Error('Unable to preload CSS for /assets/RoutePlannerPage.css')), true)
  assert.equal(isAssetLoadError(new Error('API request failed with HTTP 500')), false)
})

test('allows one automatic asset reload and resets after a successful navigation', () => {
  const storage = createStorage()
  assert.equal(claimAssetLoadRetry(storage), true)
  assert.equal(storage.getItem(ASSET_LOAD_RETRY_KEY), '1')
  assert.equal(claimAssetLoadRetry(storage), false)
  clearAssetLoadRetry(storage)
  assert.equal(storage.getItem(ASSET_LOAD_RETRY_KEY), null)
  assert.equal(claimAssetLoadRetry(storage), true)
})
