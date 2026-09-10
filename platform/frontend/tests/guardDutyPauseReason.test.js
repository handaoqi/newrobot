import assert from 'node:assert/strict'
import test from 'node:test'

import {
  guardDutyExecutionReason,
  guardDutyPauseReason,
} from '../src/utils/guardDutyPauseReason.js'

test('shows the newest explicit pause reason', () => {
  const reason = guardDutyPauseReason({
    state: 'paused',
    failure_message: 'older fallback',
    events: [
      { state: 'paused', state_version: 4, reason_message: '定位丢失' },
      { state: 'paused', state_version: 9, reason_message: '到点位姿未连续稳定' },
    ],
  })
  assert.equal(reason, '到点位姿未连续稳定')
})

test('maps a reason code and hides pause copy while running', () => {
  assert.equal(guardDutyPauseReason({
    state: 'paused',
    failure_code: 'ARRIVAL_CONFIRMATION_UNSTABLE',
  }), '到点位姿未稳定，等待重新确认')
  assert.equal(guardDutyPauseReason({ state: 'running', failure_message: 'stale reason' }), '')
})

test('shows a localized rejection reason from the execution', () => {
  assert.deepEqual(guardDutyExecutionReason({
    state: 'rejected',
    failure_code: 'ROBOT_BUSY',
    failure_message: 'another motion task is active',
  }), {
    label: '拒绝原因',
    text: '机器狗仍有未结束的运动任务，请先恢复、结束或强制退出原任务',
    code: 'ROBOT_BUSY',
  })
})

test('falls back to command acknowledgement reason', () => {
  assert.deepEqual(guardDutyExecutionReason({
    state: 'rejected',
    commands: [{
      status: 'rejected',
      ack_reason_code: 'SAFETY_GATE',
      ack_reason_message: '定位未就绪',
    }],
  }), {
    label: '拒绝原因',
    text: '定位未就绪',
    code: 'SAFETY_GATE',
  })
})
