import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { fileURLToPath } from 'node:url'

import {
  expectedLegacyMapVersion,
  navigationMapIdentity,
  navigationReadyForMap,
  navigationStatusFresh,
  navigationUnreadinessReason,
  shouldFallbackToGlobalRelocalization,
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
  const sampledAt = new Date().toISOString()
  const ready = {
    connection_status: 'online',
    status: {
      sampled_at: sampledAt,
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

test('navigation readiness rejects stale or explicitly expired localization samples', () => {
  const nowMs = Date.parse('2026-08-30T12:00:00.000Z')
  assert.equal(navigationStatusFresh({ status: { sampled_at: '2026-08-30T11:59:55.000Z' } }, { nowMs }), true)
  assert.equal(navigationStatusFresh({ status: { sampled_at: '2026-08-30T11:59:49.000Z' } }, { nowMs }), false)
  assert.equal(navigationStatusFresh({
    status: {
      sampled_at: '2026-08-30T11:59:59.000Z',
      localization_quality: { localization_fresh: false },
    },
  }, { nowMs }), false)
})

test('global search fallback is limited to an unavailable or exhausted trusted seed', () => {
  assert.equal(shouldFallbackToGlobalRelocalization('RELOCALIZATION_SEED_UNAVAILABLE'), true)
  assert.equal(shouldFallbackToGlobalRelocalization('ACTIVE_RELOCALIZATION_FAILED'), true)
  assert.equal(shouldFallbackToGlobalRelocalization('RELOCALIZATION_SUPERSEDED'), false)
  assert.equal(shouldFallbackToGlobalRelocalization('LOCALIZATION_COMMAND_BUSY'), false)
  assert.equal(shouldFallbackToGlobalRelocalization('NAV_COMMAND_FAILED'), false)
})

test('saving a route prepares its map, localization and navigation stack', () => {
  const source = readFileSync(fileURLToPath(
    new URL('../src/views/RoutePlannerPage.vue', import.meta.url),
  ), 'utf8')
  const saveHandler = source.match(
    /async function persistRoute\(\{ createOnly = false \} = \{\}\) \{([\s\S]*?)\n\}\n\nasync function handleSaveRoute/,
  )?.[1] || ''

  assert.match(saveHandler, /await activateAndRelocalizeMap\(/)
  assert.doesNotMatch(saveHandler, /await activateRouteMap\(/)
  assert.match(saveHandler, /beginLocalizationAttemptSession\(\{ phase: 'transfer', commandType: 'map\.activate' \}\)/)
  assert.match(saveHandler, /onCommand: event => applyLocalizationAttemptCommand\(event\.command, event\)/)
  assert.match(saveHandler, /sceneScope: selectedMap\.value\?\.scene_scope \|\| routeForm\.value\.scene_scope/)
  assert.match(saveHandler, /地图、定位与导航栈均已就绪/)
})

test('active relocalization reuses map activation source selection instead of quick/manual search', () => {
  const source = readFileSync(fileURLToPath(
    new URL('../src/views/RoutePlannerPage.vue', import.meta.url),
  ), 'utf8')
  const handler = source.match(
    /async function activeRelocalize\(\) \{([\s\S]*?)\n\}\n\nasync function handleExecuteRoute/,
  )?.[1] || ''

  assert.match(handler, /await activateRouteMap\(/)
  assert.match(handler, /await initializeProgressiveLocalization\(/)
  assert.doesNotMatch(handler, /quick_then_global/)
  assert.doesNotMatch(handler, /manuallySelected/)
})

test('navigation test hints describe the implemented localization gates and fallback order', () => {
  const source = readFileSync(fileURLToPath(
    new URL('../src/views/RoutePlannerPage.vue', import.meta.url),
  ), 'utf8')

  assert.match(source, /目标地图已就绪时直接复用/)
  assert.match(source, /局部候选均无合格结果时才进入全图位置与航向匹配/)
  assert.match(source, /连续 3 个新样本的 RTK 自身位置跨度不超过 0\.30 m/)
  assert.match(source, /随后必须由新鲜 FAST-LIO \+ IMU 完成接管/)
  assert.match(source, /原点阶段无合格候选才尝试手选点\/路线航点/)
  assert.match(source, /稳定 NDT 分数严格小于 0\.01 时提前结束/)
  assert.match(source, /最优 NDT 位姿已提交且分数严格小于 0\.01/)
  assert.match(source, /不表示 FAST-LIO 已完成稳定接管/)
})

test('unreadiness reason distinguishes missing map, lost localization and Nav2 down', () => {
  const sampledAt = new Date().toISOString()
  const base = {
    connection_status: 'online',
    status: {
      sampled_at: sampledAt,
      map_id: '9',
      map_version: 'legacy-mapdata-9',
      localization_status: 'normal',
      nav_ready: true,
    },
  }
  assert.equal(navigationUnreadinessReason(base, 9), null)
  assert.equal(navigationUnreadinessReason({ ...base, connection_status: 'offline' }, 9), '机器人不在线')
  assert.equal(navigationUnreadinessReason({
    ...base,
    status: { ...base.status, localization_status: 'lost' },
  }, 9), '定位丢失，待恢复')
  assert.equal(navigationUnreadinessReason({
    ...base,
    status: { ...base.status, nav_ready: false },
  }, 9), '导航栈未就绪')
  assert.equal(navigationUnreadinessReason({ connection_status: 'online', status: {} }, 9), '尚未加载任务地图')
})
