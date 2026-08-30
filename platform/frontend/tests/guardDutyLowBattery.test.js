import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

import {
  isLowBatteryBlocked,
  isLowBatteryStopAlert,
  isLowBatteryTaskError,
  lowBatteryGuardMessage,
} from '../src/utils/guardDutyLowBattery.js'

test('recognizes both low-battery edge event formats', () => {
  assert.equal(isLowBatteryStopAlert({ event_type: 'low_battery_alert' }), true)
  assert.equal(isLowBatteryStopAlert({ source_code: 'LOW_BATTERY_RETURN_CHARGE' }), true)
  assert.equal(isLowBatteryStopAlert({ raw_detection: { class: 'low_battery' } }), true)
  assert.equal(isLowBatteryStopAlert({ event_type: 'vehicle_illegal_parking' }), false)
})

test('recognizes low-battery task rejection and threshold', () => {
  assert.equal(isLowBatteryTaskError(new Error('电量不足（当前 19%）')), true)
  assert.equal(isLowBatteryTaskError({ payload: { error_code: 'LOW_BATTERY' } }), true)
  assert.equal(isLowBatteryTaskError(new Error('导航栈未就绪')), false)
  assert.equal(isLowBatteryBlocked(19), true)
  assert.equal(isLowBatteryBlocked(20), false)
})

test('builds the operator warning shown below task loop controls', () => {
  assert.equal(
    lowBatteryGuardMessage(19),
    '低电量停车告警：当前电量 19%，当前任务及循环巡检已停止，请及时人工处理或手动回充。',
  )

  const source = readFileSync(new URL('../src/views/GuardDutyPage.vue', import.meta.url), 'utf8')
  assert.match(source, /<p class="guard-loop-message">\{\{ loopMessage \}\}<\/p>/)
  assert.match(source, /loopMessage\.value = lowBatteryGuardMessage\(batteryPercent\.value\)/)
})
