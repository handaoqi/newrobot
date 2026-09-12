import assert from 'node:assert/strict'
import test from 'node:test'

import {
  attemptMarkerPose,
  attemptRejectReasonLabel,
  attemptStatusClass,
  ATTEMPT_MARKER_VISIBLE_MS,
  beginStoredAttemptSession,
  emptyAttemptSession,
  formatAttemptPose,
  formatRtkVerificationSummary,
  localizationAttemptTimeline,
  localizationAttemptFailureMessage,
  isAttemptSessionTerminal,
  localizationAttemptSessionFromCommand,
  shouldShowAttemptMarkers,
  rtkVerificationConclusionLabel,
  updateStoredAttemptSession,
  withAttemptMarkerExpiry,
} from '../src/services/localizationAttemptSession.js'

test('candidate marker keeps the numbered seed pose instead of matched output pose', () => {
  const seed = { x: 0, y: 0, yaw: Math.PI / 2 }
  const attempt = {
    seedPose: seed,
    matchedPose: { x: 1.4, y: -0.8, yaw: 0.1 },
  }
  assert.deepEqual(attemptMarkerPose(attempt), seed)
})

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
  assert.deepEqual(session.attempts.map(item => item.candidateNumber), [1, 2, 3])
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

test('failed candidates retain score, inlier, convergence and a readable summary', () => {
  const session = localizationAttemptSessionFromCommand({
    id: 'cmd-failed',
    command_type: 'nav.initial_pose',
    status: 'failed',
    result_payload: {
      localization_attempts: {
        state: 'failed',
        attempts: [{
          index: 1,
          status: 'rejected',
          ndt_candidate: {
            has_converged: false,
            matching_error: 1.65,
            inlier_fraction: 0,
            reject_reason: 'ndt_not_converged',
            quality_failures: [
              'ndt_not_converged',
              'ndt_score_above_threshold',
              'ndt_inlier_fraction_below_threshold',
            ],
          },
        }],
      },
    },
  })

  assert.equal(session.attempts[0].hasConverged, false)
  assert.equal(session.attempts[0].matchingError, 1.65)
  assert.equal(session.attempts[0].inlierFraction, 0)
  assert.equal(attemptRejectReasonLabel(session.attempts[0].rejectReason), 'NDT未收敛')
  assert.equal(
    localizationAttemptFailureMessage(session),
    '定位初始化未通过：最佳失败候选 #1 · NDT 1.650 · 内点 0.0% · NDT未收敛',
  )
})

test('new operations start from an empty session and accepted status uses a distinct class', () => {
  const session = emptyAttemptSession({ phase: 'localization', commandType: 'nav.relocalize' })
  assert.equal(session.attempts.length, 0)
  assert.equal(session.showCandidates, true)
  assert.equal(attemptStatusClass('accepted'), 'accepted')
  assert.equal(attemptStatusClass('verifying'), 'active')
  assert.equal(attemptStatusClass('rejected'), 'failed')
})

test('non-planner pages can refresh the shared route-planner attempt session', () => {
  const values = new Map()
  const previousStorage = globalThis.sessionStorage
  globalThis.sessionStorage = {
    getItem: key => values.get(key) ?? null,
    setItem: (key, value) => values.set(key, value),
    removeItem: key => values.delete(key),
  }
  try {
    const started = beginStoredAttemptSession(7, {
      phase: 'transfer',
      commandType: 'map.activate',
    })
    assert.equal(started.phase, 'transfer')
    assert.equal(started.attempts.length, 0)

    updateStoredAttemptSession(7, {
      id: 'map-command',
      command_type: 'map.activate',
      status: 'succeeded',
      started_at: '2026-09-11T00:00:00.000Z',
      finished_at: '2026-09-11T00:00:01.000Z',
    }, { phase: 'transfer', showCandidates: false })
    const localized = updateStoredAttemptSession(7, {
      id: 'relocalize-command',
      command_type: 'nav.relocalize',
      status: 'executing',
      started_at: '2026-09-11T00:00:02.000Z',
      result_payload: {
        localization_attempts: {
          state: 'running',
          attempts: [{ index: 1, status: 'verifying', x: 1, y: 2, yaw: 0.3 }],
        },
      },
    }, { phase: 'localization', showCandidates: true })

    assert.equal(localized.commandId, 'relocalize-command')
    assert.equal(localized.attempts.length, 1)
    assert.equal(localized.timelineHistory[0].key, 'map_transfer')
    assert.equal(localized.timelineHistory[0].status, 'done')
  } finally {
    if (previousStorage === undefined) delete globalThis.sessionStorage
    else globalThis.sessionStorage = previousStorage
  }
})

test('localization timeline exposes ordered stages and every candidate result', () => {
  const session = localizationAttemptSessionFromCommand({
    id: 'cmd-timeline',
    command_type: 'nav.relocalize',
    status: 'executing',
    result_payload: {
      localization_attempts: {
        state: 'global_searching',
        strategy: ['mapping_origin_bounded', 'route_waypoints', 'keyframe_global_match'],
        selected_stage: 'keyframe_global_match',
        global_search_started: true,
        stages: [
          { stage: 'mapping_origin_bounded', status: 'rejected' },
          { stage: 'route_waypoints', status: 'rejected' },
          { stage: 'keyframe_global_match', status: 'searching' },
        ],
        attempts: [
          { index: 1, stage: 'mapping_origin', status: 'rejected', x: 0, y: 0, yaw: 0, reject_reason: 'quality_gate' },
          { index: 2, stage: 'route_waypoint', status: 'rejected', x: 2, y: 3, yaw: 0.2, reject_reason: 'quality_gate' },
        ],
      },
    },
  })

  const timeline = localizationAttemptTimeline(session)
  assert.deepEqual(timeline.map(step => step.key), [
    'map_transfer',
    'localization_bootstrap',
    'mapping_origin_bounded',
    'route_waypoints',
    'keyframe_global_match',
    'best_candidate_commit',
    'navigation_start',
  ])
  assert.equal(timeline[0].status, 'done')
  assert.equal(timeline[2].status, 'failed')
  assert.equal(timeline[3].attempts.length, 1)
  assert.equal(timeline[4].status, 'active')
  assert.equal(timeline[5].status, 'waiting')
})

test('timeline preserves the RTK phase when it falls back to progressive localization', () => {
  const rtk = localizationAttemptSessionFromCommand({
    id: 'cmd-rtk',
    command_type: 'nav.initial_pose',
    status: 'failed',
  }, { phase: 'localization', source: 'rtk' })
  const progressive = localizationAttemptSessionFromCommand({
    id: 'cmd-progressive',
    command_type: 'nav.relocalize',
    status: 'executing',
    result_payload: {
      localization_attempts: {
        state: 'running',
        strategy: ['mapping_origin_bounded', 'route_waypoints', 'keyframe_global_match'],
      },
    },
  }, {
    phase: 'localization',
    timelineHistory: localizationAttemptTimeline(rtk),
  })

  const timeline = localizationAttemptTimeline(progressive)
  assert.deepEqual(timeline.map(step => step.key), [
    'map_transfer',
    'localization_bootstrap',
    'rtk_fixed',
    'mapping_origin_bounded',
    'route_waypoints',
    'keyframe_global_match',
    'best_candidate_commit',
    'navigation_start',
  ])
  assert.equal(timeline[2].status, 'failed')
  assert.equal(timeline[3].status, 'active')
})

test('timeline keeps stage times and groups mapping-origin attempts under the origin step', () => {
  const session = localizationAttemptSessionFromCommand({
    id: 'cmd-origin-times',
    command_type: 'nav.relocalize',
    status: 'succeeded',
    issued_at: '2026-09-06T13:39:36.000Z',
    started_at: '2026-09-06T13:39:36.200Z',
    finished_at: '2026-09-06T13:41:22.000Z',
    result_payload: {
      localization_attempts: {
        state: 'accepted',
        strategy: ['mapping_origin_bounded', 'route_waypoints', 'keyframe_global_match'],
        stages: [
          {
            stage: 'mapping_origin_bounded',
            status: 'rejected',
            started_at: '2026-09-06T13:39:36.300Z',
            finished_at: '2026-09-06T13:40:38.000Z',
            attempts: [
              { index: 1, status: 'rejected', x: 0, y: 0, yaw: 0 },
              { index: 2, status: 'waiting', x: 0.3, y: 0, yaw: 0 },
            ],
          },
        ],
        attempts: [
          { index: 1, status: 'rejected', x: 0, y: 0, yaw: 0 },
          { index: 2, status: 'waiting', x: 0.3, y: 0, yaw: 0 },
        ],
      },
    },
  })

  const origin = localizationAttemptTimeline(session).find(item => item.key === 'mapping_origin_bounded')
  assert.equal(origin.startedAt, '2026-09-06T13:39:36.300Z')
  assert.equal(origin.finishedAt, '2026-09-06T13:40:38.000Z')
  assert.deepEqual(origin.attempts.map(attempt => attempt.index), [1, 2])
})

test('timeline does not timestamp future stages and keeps displayed times chronological', () => {
  const session = localizationAttemptSessionFromCommand({
    id: 'cmd-ordered-times',
    command_type: 'nav.relocalize',
    status: 'executing',
    started_at: '2026-09-06T13:00:00.000Z',
    result_payload: {
      localization_attempts: {
        state: 'running',
        strategy: ['mapping_origin_bounded', 'route_waypoints', 'keyframe_global_match'],
        stages: [
          {
            stage: 'mapping_origin_bounded',
            status: 'rejected',
            started_at: '2026-09-06T13:00:20.000Z',
            finished_at: '2026-09-06T13:00:10.000Z',
          },
          {
            stage: 'route_waypoints',
            status: 'searching',
            started_at: '2026-09-06T13:00:05.000Z',
          },
          { stage: 'keyframe_global_match', status: 'waiting' },
        ],
      },
    },
  })

  const timeline = localizationAttemptTimeline(session)
  const origin = timeline.find(item => item.key === 'mapping_origin_bounded')
  const route = timeline.find(item => item.key === 'route_waypoints')
  const global = timeline.find(item => item.key === 'keyframe_global_match')
  assert.equal(origin.finishedAt, origin.startedAt)
  assert.equal(route.startedAt, origin.finishedAt)
  assert.equal(route.finishedAt, null)
  assert.equal(global.startedAt, null)
  assert.equal(global.finishedAt, null)
})

test('active attempts override a stale waiting stage record', () => {
  const session = localizationAttemptSessionFromCommand({
    id: 'cmd-active-origin',
    command_type: 'nav.relocalize',
    status: 'executing',
    started_at: '2026-09-06T13:39:00.000Z',
    result_payload: {
      localization_attempts: {
        state: 'running',
        strategy: ['mapping_origin_bounded', 'route_waypoints'],
        selected_stage: 'mapping_origin_bounded',
        active_candidate_number: 1,
        active_candidate_stage: 'mapping_origin_bounded',
        evaluated_candidate_count: 0,
        stages: [{
          stage: 'mapping_origin_bounded',
          status: 'waiting',
          started_at: '2026-09-06T13:39:36.300Z',
        }],
        attempts: [{
          index: 1,
          stage: 'mapping_origin_bounded',
          status: 'verifying',
          x: 0,
          y: 0,
          yaw: 0,
        }],
      },
    },
  })
  const origin = localizationAttemptTimeline(session).find(item => item.key === 'mapping_origin_bounded')
  assert.equal(origin.status, 'active')
  assert.equal(origin.statusLabel, '执行中')
  assert.match(origin.detail, /正在尝试 #1/)
  assert.equal(session.activeCandidateNumber, 1)
  assert.equal(session.activeCandidateStage, 'mapping_origin_bounded')
  assert.equal(attemptStatusClass(origin.attempts[0].status), 'active')
  assert.equal(origin.startedAt, '2026-09-06T13:39:36.300Z')
})

test('manual initial-pose command does not display unrelated global-search stages', () => {
  const session = localizationAttemptSessionFromCommand({
    id: 'cmd-manual',
    command_type: 'nav.initial_pose',
    status: 'succeeded',
    result_payload: {
      best_match_pose: { x: 1, y: 2, yaw: 0.1 },
      best_ndt_committed: true,
    },
  })
  const timeline = localizationAttemptTimeline(session)
  assert.deepEqual(timeline.map(step => step.key), [
    'map_transfer',
    'localization_bootstrap',
    'operator_initial_pose',
    'best_candidate_commit',
  ])
  assert.equal(timeline[2].status, 'done')
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

test('RTK fixed verification keeps concrete samples, rejection conclusion, and LIO handoff', () => {
  const session = localizationAttemptSessionFromCommand({
    id: 'cmd-rtk-details',
    command_type: 'nav.initial_pose',
    status: 'succeeded',
    result_payload: {
      localization_attempts: {
        state: 'accepted',
        selected_stage: 'rtk_fixed',
        strategy: ['rtk_fixed'],
        stages: [{
          stage: 'rtk_fixed',
          status: 'accepted',
          rtk_verification: {
            status: 'accepted',
            verified: true,
            conclusion_code: 'fixed_rtk_verified',
            sample_count: 3,
            stable_frames: 3,
            required_stable_frames: 3,
            span_m: 0.08,
            threshold_xy_m: 0.30,
            last_sample: {
              sample_stamp_ns: 103,
              quality: 'fixed',
              usable: true,
              heading_usable: true,
              map_x: 10.05,
              map_y: 2.06,
              latitude: 31.1234567,
              longitude: 121.7654321,
              horizontal_std_m: 0.012,
              solution_satellites: 24,
              heading_deg: 93.2,
              heading_std_deg: 0.4,
              accepted: true,
              reject_reasons: [],
            },
            sample_history: [
              { sample_stamp_ns: 101, quality: 'float', usable: true, heading_usable: false, reject_reasons: ['fixed_quality', 'heading_usable'] },
              { sample_stamp_ns: 103, quality: 'fixed', usable: true, heading_usable: true, map_x: 10.05, map_y: 2.06, accepted: true },
            ],
            handoff: {
              status: 'accepted',
              conclusion_code: 'lio_imu_handoff_verified',
              active_source: 'lio_imu',
              lio_healthy: true,
              lio_anchored: true,
              absolute_stable: true,
            },
          },
        }],
      },
    },
  })

  assert.equal(session.rtkVerification.verified, true)
  assert.equal(session.rtkVerification.lastSample.quality, 'fixed')
  assert.equal(session.rtkVerification.lastSample.solutionSatellites, 24)
  assert.deepEqual(session.rtkVerification.sampleHistory[0].rejectReasons, ['fixed_quality', 'heading_usable'])
  assert.equal(session.rtkVerification.handoff.activeSource, 'lio_imu')
  assert.equal(rtkVerificationConclusionLabel(session.rtkVerification), 'RTK 固定解验证通过')
  assert.match(formatRtkVerificationSummary(session.rtkVerification), /稳定样本 3\/3/)
  assert.match(
    localizationAttemptTimeline(session).find(step => step.key === 'rtk_fixed').detail,
    /结论：RTK 固定解验证通过/,
  )
})
