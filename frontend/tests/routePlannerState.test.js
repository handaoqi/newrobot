import assert from 'node:assert/strict'
import test from 'node:test'

import {
  clampMapZoom,
  headingDegreesToRadians,
  normalizeHeadingDegrees,
  paginateKeyframes,
  resolveMapClickAction,
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
