import assert from 'node:assert/strict'
import test from 'node:test'

import {
  expectedLegacyMapVersion,
  navigationMapIdentity,
  navigationReadyForMap,
} from '../src/services/mapActivationState.js'

test('legacy map versions use the platform map id', () => {
  assert.equal(expectedLegacyMapVersion(9), 'legacy-mapdata-9')
  assert.equal(expectedLegacyMapVersion(''), '')
})

test('navigation map identity prefers the latest telemetry current_map', () => {
  assert.deepEqual(navigationMapIdentity({
    current_map_id: 'old',
    current_map_version: 'old-version',
    status: {
      map_id: 'status-map',
      map_version: 'status-version',
      current_map: { map_id: '9', map_version: 'legacy-mapdata-9' },
    },
  }), { mapId: '9', mapVersion: 'legacy-mapdata-9' })
})

test('map readiness requires exact id/version, online connection, normal localization and Nav2', () => {
  const ready = {
    connection_status: 'online',
    status: {
      map_id: '9',
      map_version: 'legacy-mapdata-9',
      localization_status: 'normal',
      nav_ready: true,
    },
  }
  assert.equal(navigationReadyForMap(ready, 9), true)
  assert.equal(navigationReadyForMap({ ...ready, connection_status: 'offline' }, 9), false)
  assert.equal(navigationReadyForMap({ ...ready, status: { ...ready.status, map_version: 'v1' } }, 9), false)
  assert.equal(navigationReadyForMap({ ...ready, status: { ...ready.status, localization_status: 'initializing' } }, 9), false)
})
