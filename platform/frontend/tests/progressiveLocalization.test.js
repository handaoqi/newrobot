import assert from 'node:assert/strict'
import test from 'node:test'

import {
  buildProgressiveLocalizationPayload,
  initializeProgressiveLocalization,
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

test('guard duty and route planner share one activation and localization orchestration', async () => {
  const calls = []
  const progress = []
  const result = await initializeProgressiveLocalization({
    mapId: 9,
    robotId: 3,
    mapVersion: 'v9',
    waypoints: [{ x: 1, y: 2, yaw: 0.3 }],
    onProgress: message => progress.push(message),
    dependencies: {
      activateRouteMap: async options => {
        calls.push(['activate', options.mapId, options.robotId])
        return { navigationStatus: { localization_status: 'uninitialized' } }
      },
      sendRobotNavigationCommand: async (robotId, action, payload) => {
        calls.push(['send', robotId, action, payload])
        return { id: 'command-1', status: 'created' }
      },
      waitForRobotCommand: async (robotId, command, options) => {
        calls.push(['wait', robotId, command.id, options.timeoutMs])
        options.onProgress({ status: 'running' })
        return { ...command, status: 'succeeded' }
      },
    },
  })

  assert.equal(calls[0][0], 'activate')
  assert.deepEqual(calls[1].slice(0, 3), ['send', 3, 'relocalize'])
  assert.equal(calls[1][3].seed_source, 'progressive')
  assert.equal(calls[1][3].waypoints.length, 1)
  assert.deepEqual(calls[2], ['wait', 3, 'command-1', 420_000])
  assert.equal(result.command.status, 'succeeded')
  assert.ok(progress.some(message => message.includes('静态航向、1米范围')))
})
