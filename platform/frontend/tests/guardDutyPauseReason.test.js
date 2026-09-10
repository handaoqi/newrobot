import assert from 'node:assert/strict'
import test from 'node:test'

import { guardDutyPauseReason } from '../src/utils/guardDutyPauseReason.js'

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
