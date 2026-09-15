import assert from 'node:assert/strict'
import test from 'node:test'

import {
  buildProgressiveLocalizationPayload,
  initializeProgressiveLocalization,
  localizationCommandVerified,
  progressiveLocalizationTimeoutMs,
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
    scene_scope: 'indoor',
    coordinate_mode: 'local_only',
    localization_mode: '',
    waypoints: [
      { x: 1, y: 2, yaw: 0.1 },
      { x: 3, y: 4, yaw: -0.2 },
    ],
    wait_seconds: 180,
  })
  assert.equal(progressiveLocalizationTimeoutMs(payload), 420_000)
})

test('active relocalization payload keeps the manually selected pose as a candidate', () => {
  const manualPose = { x: 5, y: 6, yaw: 0.4 }
  const payload = buildProgressiveLocalizationPayload({
    mapId: 7,
    mapVersion: 'v7',
    waypoints: [{ ...manualPose, localization_mode: 'ukf' }, { x: 9, y: 10, yaw: -0.2 }],
  })

  assert.equal(payload.seed_source, 'progressive')
  assert.equal(payload.localization_mode, 'ukf')
  assert.deepEqual(payload.waypoints[0], manualPose)
  assert.deepEqual(payload.waypoints[1], { x: 9, y: 10, yaw: -0.2 })
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

test('outdoor RTK-fixed maps still initialize NDT-first and retain RTK as secondary policy', async () => {
  const calls = []
  const result = await initializeProgressiveLocalization({
    mapId: 12,
    robotId: 3,
    mapVersion: 'v12',
    sceneScope: 'outdoor',
    coordinateMode: 'rtk_fixed',
    localizationMode: 'rtk',
    dependencies: {
      activateRouteMap: async () => ({
        navigationStatus: {
          status: {
            localization_quality: {
              decision: {
                rtk_usable: true,
                rtk_quality: 'fixed',
                rtk_heading_usable: true,
              },
            },
          },
        },
      }),
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

  assert.deepEqual(calls[0].slice(0, 3), ['send', 3, 'relocalize'])
  assert.deepEqual(calls[0][3], {
    seed_source: 'progressive',
    map_id: 12,
    map_version: 'v12',
    scene_scope: 'outdoor',
    coordinate_mode: 'rtk_fixed',
    localization_mode: 'rtk',
    waypoints: [],
    wait_seconds: 180,
  })
  assert.deepEqual(calls[1], ['wait', 3, 'rtk-command', 420_000])
  assert.equal(calls.length, 2)
  assert.equal(result.selectedSource, 'progressive')
  assert.equal(result.rtkAttempted, false)
})

test('outdoor fixed RTK telemetry cannot bypass the progressive NDT command', async () => {
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
      activateRouteMap: async () => ({
        navigationStatus: {
          status: {
            localization_quality: {
              decision: {
                rtk_usable: true,
                rtk_quality: 'fixed',
                rtk_heading_usable: true,
              },
            },
          },
        },
      }),
      sendRobotNavigationCommand: async (robotId, action, payload) => {
        calls.push(['send', action, payload])
        return { id: `${action}-${calls.length}`, status: 'created' }
      },
      waitForRobotCommand: async (_robotId, command) => ({ ...command, status: 'succeeded' }),
    },
  })

  assert.deepEqual(calls.map(call => call[1]), ['relocalize'])
  assert.equal(calls[0][2].seed_source, 'progressive')
  assert.equal(calls[0][2].waypoints[0].x, 4)
  assert.equal(result.selectedSource, 'progressive')
  assert.equal(result.rtkAttempt, null)
  assert.ok(progress.some(message => message.includes('先搜索建图原点及附近候选')))
})

test('progressive FAST-LIO handoff failure remains terminal without direct RTK fallback', async () => {
  const actions = []
  await assert.rejects(
    initializeProgressiveLocalization({
      mapId: 12,
      robotId: 3,
      mapVersion: 'v12',
      sceneScope: 'outdoor',
      coordinateMode: 'rtk_fixed',
      dependencies: {
        activateRouteMap: async () => ({
          navigationStatus: {
            status: {
              localization_quality: {
                decision: {
                  rtk_usable: true,
                  rtk_quality: 'fixed',
                  rtk_heading_usable: true,
                },
              },
            },
          },
        }),
        sendRobotNavigationCommand: async (_robotId, action) => {
          actions.push(action)
          return { id: 'rtk-command', status: 'created' }
        },
        waitForRobotCommand: async () => {
          const error = new Error('FAST-LIO handoff failed')
          error.command = { error_code: 'LIO_HANDOFF_TIMEOUT' }
          throw error
        },
      },
    }),
    /FAST-LIO handoff failed/,
  )
  assert.deepEqual(actions, ['relocalize'])
})

test('outdoor non-fixed RTK skips the manual RTK command and starts progressive search', async () => {
  const calls = []
  const progress = []
  const result = await initializeProgressiveLocalization({
    mapId: 12,
    robotId: 3,
    mapVersion: 'v12',
    sceneScope: 'outdoor',
    coordinateMode: 'rtk_fixed',
    waypoints: [{ x: 4, y: 5, yaw: 0.2 }],
    onProgress: message => progress.push(message),
    dependencies: {
      activateRouteMap: async () => ({
        navigationStatus: {
          status: {
            localization_quality: {
              decision: {
                rtk_usable: true,
                rtk_quality: 'float',
                rtk_heading_usable: false,
              },
            },
          },
        },
      }),
      sendRobotNavigationCommand: async (_robotId, action, payload) => {
        calls.push([action, payload])
        return { id: `${action}-command`, status: 'created' }
      },
      waitForRobotCommand: async (_robotId, command) => ({ ...command, status: 'succeeded' }),
    },
  })

  assert.deepEqual(calls.map(call => call[0]), ['relocalize'])
  assert.equal(calls[0][1].seed_source, 'progressive')
  assert.equal(result.rtkAttempted, false)
  assert.equal(result.rtkAttempt, null)
  assert.ok(progress.some(message => message.includes('最优 NDT')))
})

test('outdoor map with an empty status snapshot still starts NDT-first localization', async () => {
  const calls = []
  const result = await initializeProgressiveLocalization({
    mapId: 12,
    robotId: 3,
    mapVersion: 'v12',
    sceneScope: 'outdoor',
    coordinateMode: 'rtk_fixed',
    dependencies: {
      activateRouteMap: async () => ({ navigationStatus: { status: {} } }),
      sendRobotNavigationCommand: async (_robotId, action, payload) => {
        calls.push([action, payload])
        return { id: 'live-rtk-command', status: 'created' }
      },
      waitForRobotCommand: async (_robotId, command) => ({ ...command, status: 'succeeded' }),
    },
  })

  assert.deepEqual(calls.map(call => call[0]), ['relocalize'])
  assert.equal(calls[0][1].seed_source, 'progressive')
  assert.equal(result.selectedSource, 'progressive')
})

test('a successful Edge localization result is authoritative even before telemetry replication', () => {
  assert.equal(localizationCommandVerified({
    status: 'succeeded',
    result_payload: { localization_attempts: { state: 'accepted' } },
  }), true)
  assert.equal(localizationCommandVerified({
    status: 'succeeded',
    result_payload: { handoff_pending: true },
  }), false)
  assert.equal(localizationCommandVerified({ status: 'running', result_payload: {} }), false)
})

test('operator conflicts terminate the single NDT-first initialization command', async () => {
  const actions = []
  await assert.rejects(
    initializeProgressiveLocalization({
      mapId: 12,
      robotId: 3,
      mapVersion: 'v12',
      sceneScope: 'outdoor',
      coordinateMode: 'rtk_fixed',
      dependencies: {
        activateRouteMap: async () => ({
          navigationStatus: {
            status: {
              localization_quality: {
                decision: {
                  rtk_usable: true,
                  rtk_quality: 'fixed',
                  rtk_heading_usable: true,
                },
              },
            },
          },
        }),
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
  assert.deepEqual(actions, ['relocalize'])
})
