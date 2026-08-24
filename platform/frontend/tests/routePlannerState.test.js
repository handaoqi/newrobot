import assert from 'node:assert/strict'
import test from 'node:test'

import {
  clampMapZoom,
  headingBetweenMapPoints,
  headingDegreesToRadians,
  normalizeHeadingDegrees,
  normalizeRoutePlannerTelemetry,
  normalizeRtkQuality,
  paginateKeyframes,
  resolveMapClickAction,
  rtkFixStatusLabel,
  rtkPositionTypeLabel,
  rtkQualityLabel,
  rtkSolutionStatusLabel,
} from '../src/services/routePlannerState.js'

test('heading input is normalized and converted only when valid', () => {
  assert.equal(normalizeHeadingDegrees(450), 90)
  assert.equal(normalizeHeadingDegrees(-15), 345)
  assert.equal(normalizeHeadingDegrees('invalid'), null)
  assert.equal(headingDegreesToRadians(90), Number((Math.PI / 2).toFixed(5)))
})

test('map zoom stays within the supported range', () => {
  assert.equal(clampMapZoom(0.1), 0.5)
  assert.equal(clampMapZoom(1.25), 1.25)
  assert.equal(clampMapZoom(9), 3)
  assert.equal(clampMapZoom('invalid'), 1)
})

test('keyframe pagination clamps the page and returns stable offsets', () => {
  const samples = Array.from({ length: 121 }, (_, index) => ({ index }))
  assert.deepEqual(paginateKeyframes(samples, 2, 50), {
    page: 2,
    pageCount: 3,
    start: 50,
    rows: samples.slice(50, 100),
  })
  assert.equal(paginateKeyframes(samples, 99, 50).page, 3)
})

test('map click mode has one explicit action and initial pose takes precedence', () => {
  assert.equal(resolveMapClickAction('waypoint'), 'waypoint')
  assert.equal(resolveMapClickAction('inspect'), 'inspect')
  assert.equal(resolveMapClickAction('waypoint', true), 'initial_pose')
})

test('two map points produce the same yaw convention used by initial pose', () => {
  assert.equal(headingBetweenMapPoints({ x: 1, y: 1 }, { x: 2, y: 1 }), 0)
  assert.equal(headingBetweenMapPoints({ x: 1, y: 1 }, { x: 1, y: 2 }), Number((Math.PI / 2).toFixed(5)))
  assert.equal(headingBetweenMapPoints({ x: 1, y: 1 }, { x: 1.01, y: 1.01 }), null)
})

test('RTK quality aliases normalize to one route-planner enum', () => {
  assert.equal(normalizeRtkQuality('rtk_fixed'), 'fixed')
  assert.equal(normalizeRtkQuality('fixed'), 'fixed')
  assert.equal(normalizeRtkQuality('rtk_float'), 'float')
  assert.equal(normalizeRtkQuality('standalone'), 'standalone')
  assert.equal(normalizeRtkQuality('unknown-quality'), 'invalid')
  assert.equal(rtkQualityLabel('rtk_fixed'), '固定解')
  assert.equal(rtkQualityLabel('float'), '浮点解')
  assert.equal(rtkQualityLabel(null), '无数据')
})

test('RTK position and solution codes retain numeric values and expose confirmed meanings', () => {
  assert.equal(rtkPositionTypeLabel(0), '无定位（码 0）')
  assert.equal(rtkPositionTypeLabel(48), 'RTK固定类型（码 48）')
  assert.equal(rtkPositionTypeLabel(17), 'RTK浮点类型（码 17）')
  assert.equal(rtkPositionTypeLabel(99), '未知类型（码 99）')
  assert.equal(rtkSolutionStatusLabel(0), '解算成功（码 0）')
  assert.equal(rtkSolutionStatusLabel(2), '解算未通过（码 2）')
  assert.equal(rtkFixStatusLabel(-1), '无定位（码 -1）')
  assert.equal(rtkFixStatusLabel(2), 'GBAS差分定位（RTK固定映射）（码 2）')
})

test('route planner telemetry normalizes nested and raw RTK payloads', () => {
  const telemetry = normalizeRoutePlannerTelemetry({
    localization: {
      sensors: { imu: { online: true } },
      raw_rtk: { position_type: 48, solution_status: 0 },
    },
    sensors: { rtk: { online: true, sample_age_seconds: 0.2 } },
  })
  assert.equal(telemetry.sensors.imu.online, true)
  assert.equal(telemetry.rtk.online, true)
  assert.equal(telemetry.rawRtk.position_type, 48)
  assert.equal(telemetry.rawRtk.solution_status, 0)
})
