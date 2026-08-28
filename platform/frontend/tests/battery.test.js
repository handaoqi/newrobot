import assert from 'node:assert/strict'
import test from 'node:test'

import { resolveBatteryPercent } from '../src/utils/battery.js'

test('live robot status battery has priority over stale robot list data', () => {
  assert.equal(resolveBatteryPercent(
    { battery_percent: 62, power: { percent: 55 } },
    { battery_level: 78 },
  ), 62)
})

test('battery falls back through power status and robot list data', () => {
  assert.equal(resolveBatteryPercent({ power: { percent: 55 } }, { battery_level: 78 }), 55)
  assert.equal(resolveBatteryPercent({}, { battery_level: '78.4' }), 78)
  assert.equal(resolveBatteryPercent(null, { battery_level: 0 }), 0)
})

test('battery values are bounded and invalid values are ignored', () => {
  assert.equal(resolveBatteryPercent({ battery_percent: 120 }, null), 100)
  assert.equal(resolveBatteryPercent({ battery_percent: -2 }, null), 0)
  assert.equal(resolveBatteryPercent({ battery_percent: 'offline' }, { battery_level: 45 }), 45)
  assert.equal(resolveBatteryPercent({}, {}), null)
})
