import assert from 'node:assert/strict'
import test from 'node:test'

import {
  attemptStatusClass,
  ATTEMPT_MARKER_VISIBLE_MS,
  emptyAttemptSession,
  formatAttemptPose,
  isAttemptSessionTerminal,
  localizationAttemptSessionFromCommand,
  shouldShowAttemptMarkers,
  withAttemptMarkerExpiry,
} from '../src/services/localizationAttemptSession.js'

test('command progress snapshots keep candidate order and hide transfer-phase markers', () => {
  const session = localizationAttemptSessionFromCommand({
    id: 'cmd-1',
    command_type: 'nav.relocalize',
    status: 'executing',
    result_payload: {
      localization_attempts: {
        state: 'running',
        source: 'mapping_origin',
        live_pose: { x: 1.1, y: 2.2, yaw: 0.3 },
        attempts: [
          { index: 1, status: 'rejected', x: 0, y: 0, yaw: 0, reject_reason: 'quality_gate' },
          {
            index: 2,
            status: 'verifying',
            seed_pose: { x: 0.3, y: 0, yaw: 0 },
            live_pose: { x: 0.28, y: 0.01, yaw: 0.02 },
            ndt_candidate: {
              eligible: true,
              matching_error: 0.12,
              inlier_fraction: 0.81,
              geometric_rmse: 0.08,
              matched_pose: { x: 0.29, y: 0.01, yaw: 0.02 },
            },
          },
          { index: 3, status: 'waiting', x: 0.6, y: 0, yaw: 0 },
        ],
      },
    },
  })

  assert.equal(session.commandId, 'cmd-1')
  assert.equal(session.attempts.length, 3)
  assert.deepEqual(session.attempts.map(item => item.index), [1, 2, 3])
  assert.equal(session.attempts[1].matchingError, 0.12)
  assert.equal(session.attempts[1].inlierFraction, 0.81)
  assert.equal(formatAttemptPose(session.livePose), 'x 1.100 / y 2.200 / yaw 0.300')
  assert.equal(shouldShowAttemptMarkers(session), true)

  const transfer = localizationAttemptSessionFromCommand({
    id: 'cmd-2',
    command_type: 'map.activate',
    status: 'executing',
  }, { phase: 'transfer', showCandidates: false })
  assert.equal(transfer.showCandidates, false)
  assert.equal(shouldShowAttemptMarkers(transfer), false)
})

test('new operations start from an empty session and accepted status uses a distinct class', () => {
  const session = emptyAttemptSession({ phase: 'localization', commandType: 'nav.relocalize' })
  assert.equal(session.attempts.length, 0)
  assert.equal(session.showCandidates, true)
  assert.equal(attemptStatusClass('accepted'), 'accepted')
  assert.equal(attemptStatusClass('verifying'), 'active')
  assert.equal(attemptStatusClass('rejected'), 'failed')
})

test('refresh restore keeps the committed best match pose', () => {
  const session = localizationAttemptSessionFromCommand({
    id: 'cmd-9',
    command_type: 'nav.relocalize',
    status: 'succeeded',
    result_payload: {
      best_match_pose: { x: 4.5, y: 6.1, yaw: 1.2 },
      best_ndt_candidate: {
        matching_error: 0.09,
        inlier_fraction: 0.88,
        matched_pose: { x: 4.5, y: 6.1, yaw: 1.2 },
      },
      localization_attempts: {
        state: 'accepted',
        selected_stage: 'mapping_origin_bounded',
        attempts: [
          { index: 1, status: 'rejected', x: 0, y: 0, yaw: 0 },
          { index: 2, status: 'accepted', seed_pose: { x: 4.4, y: 6.0, yaw: 1.1 } },
        ],
      },
    },
  })
  assert.deepEqual(session.bestMatchPose, { x: 4.5, y: 6.1, yaw: 1.2 })
  assert.equal(session.source, 'mapping_origin_bounded')
  assert.equal(session.attempts[1].status, 'accepted')
})

test('attempt markers hide one minute after a terminal session', () => {
  const now = 1_700_000_000_000
  const session = withAttemptMarkerExpiry({
    showCandidates: true,
    status: 'accepted',
    attempts: [{ index: 1, status: 'accepted', seedPose: { x: 1, y: 2, yaw: 0 } }],
  }, now)
  assert.equal(isAttemptSessionTerminal(session), true)
  assert.equal(shouldShowAttemptMarkers(session, now), true)
  assert.equal(shouldShowAttemptMarkers(session, now + ATTEMPT_MARKER_VISIBLE_MS - 1), true)
  assert.equal(shouldShowAttemptMarkers(session, now + ATTEMPT_MARKER_VISIBLE_MS), false)
})

test('terminal relocalize command sets marker expiry metadata', () => {
  const session = localizationAttemptSessionFromCommand({
    id: 'cmd-10',
    command_type: 'nav.relocalize',
    status: 'succeeded',
    result_payload: {
      best_match_pose: { x: 1, y: 2, yaw: 0.1 },
      localization_attempts: {
        state: 'accepted',
        attempts: [{ index: 1, status: 'accepted', seed_pose: { x: 1, y: 2, yaw: 0.1 } }],
      },
    },
  })
  assert.ok(session.markersVisibleUntil > Date.now())
  assert.equal(shouldShowAttemptMarkers(session, session.markersVisibleUntil), false)
})

test('only strict NDT or verified RTK results qualify as optimal markers', () => {
  const ndt = localizationAttemptSessionFromCommand({
    status: 'succeeded',
    result_payload: {
      best_ndt_committed: true,
      best_ndt_candidate: { matching_error: 0.009, matched_pose: { x: 1, y: 2, yaw: 0 } },
    },
  })
  assert.equal(ndt.optimalVerified, true)

  const ordinary = localizationAttemptSessionFromCommand({
    status: 'succeeded',
    result_payload: {
      best_ndt_committed: true,
      best_ndt_candidate: { matching_error: 0.01, matched_pose: { x: 1, y: 2, yaw: 0 } },
    },
  })
  assert.equal(ordinary.optimalVerified, false)

  const rtk = localizationAttemptSessionFromCommand({
    status: 'succeeded',
    result_payload: { rtk_drift: { xy_m: 0.29, verified: true } },
  })
  assert.equal(rtk.optimalVerified, true)
})
