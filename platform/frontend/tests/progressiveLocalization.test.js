import assert from 'node:assert/strict'
import test from 'node:test'

import {
  buildProgressiveLocalizationPayload,
  initializeProgressiveLocalization,
  progressiveLocalizationTimeoutMs,
  shouldInitializeFromRtk,
} from '../src/services/progressiveLocalization.js'

test('initialization sends mapping-origin and route-waypoint candidates for progressive search', () => {
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

test('progressive initialization uses a bounded budget without accepting invalid points', () => {
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
  assert.deepEqual(calls[1][3].waypoints, [{ x: 1, y: 2, yaw: 0.3 }])
  assert.deepEqual(calls[2], ['wait', 3, 'command-1', 420_000])
  assert.equal(result.command.status, 'succeeded')
  assert.ok(progress.some(message => message.includes('原点/航点候选搜索')))
})

test('outdoor RTK-fixed maps initialize from fixed RTK and local NDT before progressive search', async () => {
  const calls = []
  const result = await initializeProgressiveLocalization({
    mapId: 12,
    robotId: 3,
    mapVersion: 'v12',
    sceneScope: 'outdoor',
    coordinateMode: 'rtk_fixed',
    dependencies: {
      activateRouteMap: async () => ({ navigationStatus: {} }),
      sendRobotNavigationCommand: async (robotId, action, payload) => {
        calls.push(['send', robotId, action, payload])
        return { id: 'rtk-command', status: 'created' }
      },
      waitForRobotCommand: async (robotId, command, options) => {
        calls.push(['wait', robotId, command.id, options.timeoutMs])
        return { ...command, status: 'succeeded' }
      },
    },
  })

  assert.deepEqual(calls[0].slice(0, 3), ['send', 3, 'initial-pose'])
  assert.deepEqual(calls[0][3], {
    seed_source: 'rtk',
    map_id: 12,
    map_version: 'v12',
    wait_seconds: 30,
    start_navigation: true,
  })
  assert.deepEqual(calls[1], ['wait', 3, 'rtk-command', 90_000])
  assert.equal(calls.length, 2)
  assert.equal(result.selectedSource, 'rtk_fixed')
  assert.equal(result.rtkAttempted, true)
})

test('outdoor RTK failure falls back to quick search with global fallback', async () => {
  const calls = []
  const progress = []
  const result = await initializeProgressiveLocalization({
    mapId: 12,
    robotId: 3,
    mapVersion: 'v12',
    sceneScope: 'transition',
    coordinateMode: 'rtk_fixed',
    waypoints: [{ x: 4, y: 5, yaw: 0.2 }],
    onProgress: message => progress.push(message),
    dependencies: {
      activateRouteMap: async () => ({ navigationStatus: {} }),
      sendRobotNavigationCommand: async (robotId, action, payload) => {
        calls.push(['send', action, payload])
        return { id: `${action}-${calls.length}`, status: 'created' }
      },
      waitForRobotCommand: async (_robotId, command) => {
        if (command.id.startsWith('initial-pose')) {
          const error = new Error('RTK unavailable')
          error.command = { error_code: 'RTK_POSE_UNAVAILABLE' }
          throw error
        }
        return { ...command, status: 'succeeded' }
      },
    },
  })

  assert.deepEqual(calls.map(call => call[1]), ['initial-pose', 'relocalize'])
  assert.equal(calls[1][2].seed_source, 'progressive')
  assert.equal(calls[1][2].waypoints[0].x, 4)
  assert.equal(result.selectedSource, 'progressive')
  assert.equal(result.rtkAttempt.errorCode, 'RTK_POSE_UNAVAILABLE')
  assert.ok(progress.some(message => message.includes('转入快速定位')))
  assert.ok(progress.some(message => message.includes('全局搜索')))
})

test('local-only maps never attempt RTK even when scene metadata is inconsistent', () => {
  assert.equal(shouldInitializeFromRtk({ sceneScope: 'outdoor', coordinateMode: 'local_only' }), false)
  assert.equal(shouldInitializeFromRtk({ sceneScope: 'outdoor', coordinateMode: 'rtk_fixed' }), true)
  assert.equal(shouldInitializeFromRtk({ sceneScope: 'indoor', coordinateMode: 'rtk_fixed' }), false)
})

test('operator conflicts do not silently fall back from outdoor RTK initialization', async () => {
  const actions = []
  await assert.rejects(
    initializeProgressiveLocalization({
      mapId: 12,
      robotId: 3,
      mapVersion: 'v12',
      sceneScope: 'outdoor',
      coordinateMode: 'rtk_fixed',
      dependencies: {
        activateRouteMap: async () => ({ navigationStatus: {} }),
        sendRobotNavigationCommand: async (_robotId, action) => {
          actions.push(action)
          return { id: 'rtk-command', status: 'created' }
        },
        waitForRobotCommand: async () => {
          const error = new Error('superseded')
          error.command = { error_code: 'RELOCALIZATION_SUPERSEDED' }
          throw error
        },
      },
    }),
    /superseded/,
  )
  assert.deepEqual(actions, ['initial-pose'])
})
