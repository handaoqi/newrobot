import assert from 'node:assert/strict'
import test from 'node:test'

import {
  guardDutySpeedLabel,
  guardDutySpeedReading,
  guardDutySpeedTitle,
  isLatestTrajectoryResponse,
} from '../src/utils/guardDutySpeed.js'

const nowMs = new Date('2026-09-10T20:00:10+08:00').getTime()

function status({ speed = 0.25, turn = 0, velocityAge = 0.1, localizationFresh = true } = {}) {
  return {
    status: {
      received_at: '2026-09-10T20:00:09+08:00',
      speed_mps: String(speed),
      localization_quality: {
        localization_fresh: localizationFresh,
        localization_sample_age_seconds: 0.1,
      },
      navigation: {
        actual_planar_speed_mps: speed,
        actual_turn_speed_rps: turn,
        actual_velocity_sample_age_seconds: velocityAge,
      },
    },
  }
}

test('inactive tasks report a real zero instead of unavailable data', () => {
  const reading = guardDutySpeedReading({ executionState: 'completed', nowMs })
  assert.equal(reading.available, true)
  assert.equal(reading.valueMps, 0)
  assert.equal(reading.source, 'task_inactive')
  assert.equal(guardDutySpeedLabel(reading), '0.00 m/s')
})

test('fresh actual velocity wins and exposes in-place turn speed', () => {
  const reading = guardDutySpeedReading({
    executionState: 'running',
    navigationStatus: status({ speed: 0, turn: 0.42 }),
    nowMs,
  })
  assert.equal(reading.source, 'actual_velocity')
  assert.equal(reading.valueMps, 0)
  assert.equal(reading.turnRps, 0.42)
  assert.match(guardDutySpeedTitle(reading), /角速度 0.42 rad\/s/)
})

test('fresh localization velocity supports old edge agents without command age', () => {
  const payload = status({ speed: 0.18 })
  delete payload.status.navigation.actual_velocity_sample_age_seconds
  const reading = guardDutySpeedReading({
    executionState: 'running',
    navigationStatus: payload,
    nowMs,
  })
  assert.equal(reading.source, 'localization_velocity')
  assert.equal(reading.valueMps, 0.18)
})

test('trajectory received time tolerates the five-second batch interval', () => {
  const reading = guardDutySpeedReading({
    executionState: 'running',
    navigationStatus: status({ velocityAge: 20, localizationFresh: false }),
    trajectory: [
      { sampled_at: '2026-09-10T20:00:03+08:00', received_at: '2026-09-10T20:00:05+08:00', x: 1, y: 1, map_id: '151' },
      { sampled_at: '2026-09-10T20:00:04+08:00', received_at: '2026-09-10T20:00:05+08:00', x: 1.008, y: 1, map_id: '151' },
    ],
    nowMs,
  })
  assert.equal(reading.source, 'trajectory')
  assert.equal(reading.available, true)
  assert.ok(Math.abs(reading.valueMps - 0.008) < 1e-9)
  assert.equal(guardDutySpeedLabel(reading), '0.01 m/s')
})

test('running task with stale sources reports unavailable instead of false zero', () => {
  const reading = guardDutySpeedReading({
    executionState: 'running',
    navigationStatus: status({ velocityAge: 20, localizationFresh: false }),
    trajectory: [
      { sampled_at: '2026-09-10T19:59:01+08:00', received_at: '2026-09-10T19:59:02+08:00', x: 1, y: 1 },
      { sampled_at: '2026-09-10T19:59:02+08:00', received_at: '2026-09-10T19:59:03+08:00', x: 1.2, y: 1 },
    ],
    nowMs,
  })
  assert.equal(reading.available, false)
  assert.equal(guardDutySpeedLabel(reading), '-- m/s')
})

test('only the newest request for the current execution may replace trajectory', () => {
  assert.equal(isLatestTrajectoryResponse({
    requestGeneration: 3,
    latestGeneration: 3,
    requestedExecutionId: 'new',
    currentExecutionId: 'new',
  }), true)
  assert.equal(isLatestTrajectoryResponse({
    requestGeneration: 2,
    latestGeneration: 3,
    requestedExecutionId: 'old',
    currentExecutionId: 'new',
  }), false)
})
