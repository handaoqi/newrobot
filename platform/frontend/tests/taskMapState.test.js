import assert from 'node:assert/strict'
import test from 'node:test'

import {
  buildLocalizationLossMarkers,
  currentRobotMapPose,
} from '../src/services/taskMapState.js'

test('loss markers use persisted trusted pose and stay on their task map', () => {
  const execution = {
    map_data: 17,
    events: [
      {
        id: 1,
        event_type: 'task.pausing',
        reason_code: 'LOCALIZATION_LOST',
        state_version: 8,
        occurred_at: '2026-08-21T10:00:00Z',
        payload: {
          map_id: '17',
          last_trusted_pose: { x: 1.5, y: 2.5, yaw: 0.3 },
          localization_quality: { matching_error: 0.8 },
        },
      },
      {
        id: 2,
        event_type: 'task.resuming',
        reason_code: 'LOCALIZATION_RECOVERED',
        state_version: 10,
        payload: {},
      },
    ],
  }

  const markers = buildLocalizationLossMarkers(execution, [], 17)
  assert.equal(markers.length, 1)
  assert.equal(markers[0].recoveryState, 'recovered')
  assert.equal(buildLocalizationLossMarkers(execution, [], 18).length, 0)
})

test('robot marker exposes lost telemetry as untrusted and falls back to trusted loss point', () => {
  const lost = currentRobotMapPose({
    current_map_id: 17,
    status: { map_id: 17, x: 4, y: 5, yaw: 1, localization_status: 'lost' },
  }, 17, [])
  assert.equal(lost.trusted, false)
  assert.equal(lost.source, 'telemetry')

  const fallback = currentRobotMapPose({ status: null }, 17, [{ x: 1, y: 2, yaw: 0.4 }])
  assert.equal(fallback.source, 'last_trusted')
  assert.equal(fallback.x, 1)
})
