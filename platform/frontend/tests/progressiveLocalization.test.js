import assert from 'node:assert/strict'
import test from 'node:test'

import {
  buildProgressiveLocalizationPayload,
  progressiveLocalizationTimeoutMs,
} from '../src/services/progressiveLocalization.js'

test('progressive initialization sends every route waypoint in order', () => {
  const payload = buildProgressiveLocalizationPayload({
    mapId: 7,
    mapVersion: 'v7',
    waypoints: [
      [1, 2, 0.1],
      { x: 3, y: 4, yaw: -0.2, name: 'point 2' },
    ],
  })

  assert.deepEqual(payload, {
    seed_source: 'progressive',
    map_id: 7,
    map_version: 'v7',
    waypoints: [
      { x: 1, y: 2, yaw: 0.1 },
      { x: 3, y: 4, yaw: -0.2 },
    ],
    wait_seconds: 180,
  })
  assert.equal(progressiveLocalizationTimeoutMs(payload), 420_000)
})

test('progressive initialization scales its budget without accepting invalid points', () => {
  const payload = buildProgressiveLocalizationPayload({
    mapId: 7,
    mapVersion: 'v7',
    waypoints: Array.from({ length: 30 }, (_, index) => ({ x: index, y: index + 1, yaw: 0 })),
  })

  assert.equal(payload.wait_seconds, 308)
  assert.throws(
    () => buildProgressiveLocalizationPayload({ waypoints: [{ x: 1, y: null, yaw: 0 }] }),
    /第 1 个路线航点坐标无效/,
  )
})
