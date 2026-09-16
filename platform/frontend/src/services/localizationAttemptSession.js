export const ATTEMPT_STATUS_LABELS = {
  waiting: '等待',
  started: '执行中',
  verifying: '执行中',
  qualified: 'NDT通过，待提交',
  committing: '提交中',
  accepted: '成功',
  rejected: '失败',
  failed: '失败',
  skipped: '已跳过',
}

export const LOCALIZATION_ATTEMPT_COMMAND_TYPES = new Set([
  'nav.initial_pose',
  'nav.relocalize',
  'task.start',
])

export const ATTEMPT_MARKER_VISIBLE_MS = 60_000

const ATTEMPT_REJECT_REASON_LABELS = {
  ndt_not_converged: 'NDT未收敛',
  ndt_score_unavailable: '未收到NDT分数',
  ndt_score_above_threshold: 'NDT分数超过阈值',
  ndt_inlier_fraction_unavailable: '未收到内点率',
  ndt_inlier_fraction_below_threshold: '内点率低于门限',
  ndt_matched_pose_unavailable: '无有效NDT匹配位姿',
  ndt_stable_frames_insufficient: 'NDT稳定帧不足',
  seed_position_correction_exceeded: '位置修正超过安全门限',
  seed_yaw_correction_exceeded: '航向修正超过安全门限',
  ndt_sample_unavailable: '本候选未收到新的NDT观测',
  quality_gate: '未通过NDT质量门限',
  out_ranked: '已被更优候选替代',
  optimal_threshold_reached: '已有最优候选，未再尝试',
  local_search_budget_exhausted: '局部搜索时间已用完，未执行',
  lio_handoff_failed: 'FAST-LIO 接管失败',
  quick_search_budget_exhausted: '快速搜索时间已用完',
  LOCAL_SEARCH_BUDGET_EXHAUSTED: '局部搜索时间已用完',
  INITIAL_POSE_NOT_ACCEPTED: '初始位姿未通过定位验收',
}

const ATTEMPT_TERMINAL_STATES = new Set([
  'accepted',
  'failed',
  'handoff_failed',
  'succeeded',
])

const RTK_VERIFICATION_CONCLUSION_LABELS = {
  awaiting_fresh_rtk_samples: '等待新的 RTK 样本',
  fixed_rtk_samples_pending: '固定解稳定样本不足',
  fixed_rtk_verified: 'RTK 固定解验证通过',
  no_fresh_rtk_samples: '验证窗口内没有新的 RTK 样本',
  position_usable: 'RTK 位置不可用或已过期',
  fixed_quality: 'RTK 不是固定解',
  heading_usable: '双天线航向不可用或质量不合格',
  map_position_finite: 'RTK 无有效地图坐标',
  map_heading_finite: 'RTK 无有效地图航向',
  rtk_lost_before_commit: '提交前 RTK 固定解失效',
  rtk_self_span_above_threshold: 'RTK 位置稳定跨度超过门限',
}

const RTK_VERIFICATION_REASON_LABELS = {
  position_usable: '位置可用性未通过',
  fixed_quality: '不是固定解',
  heading_usable: '航向不可用',
  map_position_finite: '地图坐标无效',
  map_heading_finite: '地图航向无效',
  rtk_self_span_above_threshold: '位置稳定跨度超限',
}

const TIMELINE_STAGE_META = {
  map_transfer: { title: '地图下发', detail: '确认目标地图已传输并应用到机器狗' },
  navigation_prepare: { title: '准备导航安全组', detail: '配置地图、过滤器和 Collision Monitor；执行组保持未激活' },
  fast_lio_readiness: { title: 'FAST-LIO 局部收敛', detail: '确认 IMU、iKD-tree 与连续新鲜本地里程计，尚未要求地图定位 status=3' },
  localization_bootstrap: { title: '定位节点准备', detail: '准备 /initialpose 接收器和定位服务' },
  rtk_fixed: { title: 'RTK 二次校正验证', detail: '最优 NDT 已提交并由 FAST-LIO + IMU 接管后，再验证固定解并更新锚点' },
  trusted_rtk_fixed: { title: 'RTK 固定解可信搜索种子', detail: '仅缩小室外/过渡场景 NDT 候选范围，不直接提交为初始位姿' },
  fast_lio_imu_handoff: { title: 'FAST-LIO + IMU 主定位接管', detail: '确认新鲜 FAST-LIO + IMU 帧、锚点代数和连续主定位源' },
  final_localization_gate: { title: '最终定位放行', detail: '连续 3 帧新鲜 status=3、LIO 锚点与二次校正终态均通过' },
  navigation_execution_activate: { title: '激活导航执行组', detail: '定位通过后激活规划、控制、行为树、平滑与航点执行节点' },
  secondary_correction: { title: '二次定位校正', detail: 'NDT 最优提交后按场景和航点策略执行 RTK、UKF 或 NDT 校正' },
  rtk_correction: { title: 'RTK 二次校正', detail: '仅合格固定解可作为绝对校正源；非 fixed 不阻塞任务' },
  ukf_correction: { title: 'UKF 二次融合校正', detail: '按 NDT 与浮点 RTK 偏差门限决定是否加权融合' },
  ndt_secondary_correction: { title: 'NDT 二次校正', detail: '使用高质量 NDT 更新 map→LIO 锚点，不改写 FAST-LIO 原始轨迹' },
  last_trusted: { title: '可信位姿候选', detail: '尝试最近一次可信定位位姿' },
  mapping_origin_bounded: { title: '建图原点及周边候选', detail: '原点、航向假设和 0.3/0.6/1.0 m 周边候选' },
  route_waypoints: { title: '手选点/路线航点候选', detail: '逐个验证手选点和路线航点' },
  keyframe_global_match: { title: '全局关键帧匹配', detail: 'Scan Context / FastGICP / ICP 全局回退搜索' },
  quick_initialization: { title: '快速候选搜索', detail: '兼容旧版本的快速种子搜索' },
  operator_initial_pose: { title: '手选初始位姿 NDT 验证', detail: '验证手选位置和朝向，确认 NDT 质量门限' },
  best_candidate_commit: { title: '提交最优定位结果', detail: '提交排名最高的 NDT 位姿并等待定位接管' },
  navigation_start: { title: '启动导航栈', detail: '等待 Nav2 生命周期节点和导航接口就绪' },
}

const TIMELINE_STAGE_ALIASES = {
  mapping_origin: 'mapping_origin_bounded',
  mapping_origin_local: 'mapping_origin_bounded',
  route_waypoint: 'route_waypoints',
  global: 'keyframe_global_match',
}

const TIMELINE_STAGE_ORDER = [
  'map_transfer',
  'navigation_prepare',
  'fast_lio_readiness',
  'localization_bootstrap',
  'trusted_rtk_fixed',
  'last_trusted',
  'mapping_origin_bounded',
  'route_waypoints',
  'keyframe_global_match',
  'quick_initialization',
  'operator_initial_pose',
  'best_candidate_commit',
  'fast_lio_imu_handoff',
  'secondary_correction',
  'rtk_fixed',
  'rtk_correction',
  'ukf_correction',
  'ndt_secondary_correction',
  'final_localization_gate',
  'navigation_execution_activate',
  'navigation_start',
]

export function localizationAttemptStorageKey(robotId) {
  return `roamerx.localizationAttemptSession.${robotId || 'unknown'}`
}

function finiteNumber(value) {
  if (value === null || value === undefined || value === '') return null
  const number = Number(value)
  return Number.isFinite(number) ? number : null
}

function finitePose(value) {
  if (!value || typeof value !== 'object') return null
  const x = finiteNumber(value.x)
  const y = finiteNumber(value.y)
  const yaw = finiteNumber(value.yaw)
  if (![x, y, yaw].every(item => item !== null)) return null
  const z = finiteNumber(value.z)
  return z === null ? { x, y, yaw } : { x, y, yaw, z }
}

function normalizeRtkSample(value) {
  if (!value || typeof value !== 'object') return null
  return {
    sampleStampNs: finiteNumber(value.sample_stamp_ns ?? value.sampleStampNs),
    quality: value.quality || 'unknown',
    usable: value.usable === true,
    headingUsable: value.heading_usable === true || value.headingUsable === true,
    goodForNavigation: value.good_for_navigation === true || value.goodForNavigation === true,
    blockedReason: value.blocked_reason || value.blockedReason || '',
    mapX: finiteNumber(value.map_x ?? value.mapX),
    mapY: finiteNumber(value.map_y ?? value.mapY),
    mapYaw: finiteNumber(value.map_yaw ?? value.mapYaw),
    latitude: finiteNumber(value.latitude),
    longitude: finiteNumber(value.longitude),
    altitude: finiteNumber(value.altitude),
    positionAgeSeconds: finiteNumber(value.position_age_s ?? value.positionAgeSeconds),
    horizontalStdM: finiteNumber(value.horizontal_std_m ?? value.horizontalStdM),
    fixStatus: value.fix_status ?? value.fixStatus ?? null,
    solutionStatus: value.solution_status ?? value.solutionStatus ?? null,
    positionType: value.position_type ?? value.positionType ?? null,
    solutionSatellites: finiteNumber(value.solution_satellites ?? value.solutionSatellites),
    headingStatus: value.heading_status ?? value.headingStatus ?? null,
    headingType: value.heading_type ?? value.headingType ?? null,
    headingDeg: finiteNumber(value.heading_deg ?? value.headingDeg),
    headingStdDeg: finiteNumber(value.heading_std_deg ?? value.headingStdDeg),
    headingBaselineM: finiteNumber(value.heading_baseline_m ?? value.headingBaselineM),
    headingAgeSeconds: finiteNumber(value.heading_age_s ?? value.headingAgeSeconds),
    accepted: value.accepted === true,
    rejectReasons: Array.isArray(value.reject_reasons ?? value.rejectReasons)
      ? [...(value.reject_reasons ?? value.rejectReasons)]
      : [],
  }
}

function normalizeRtkVerification(value) {
  if (!value || typeof value !== 'object') return null
  const handoff = value.handoff && typeof value.handoff === 'object'
    ? {
        status: value.handoff.status || '',
        conclusionCode: value.handoff.conclusion_code || value.handoff.conclusionCode || '',
        conclusion: value.handoff.conclusion || '',
        activeSource: value.handoff.active_source || value.handoff.activeSource || '',
        handoffState: value.handoff.handoff_state || value.handoff.handoffState || '',
        lioHealthy: value.handoff.lio_healthy === true || value.handoff.lioHealthy === true,
        lioAnchored: value.handoff.lio_anchored === true || value.handoff.lioAnchored === true,
        absoluteStable: value.handoff.absolute_stable === true || value.handoff.absoluteStable === true,
        rosExecutorAlive: typeof (value.handoff.ros_executor_alive ?? value.handoff.rosExecutorAlive) === 'boolean'
          ? Boolean(value.handoff.ros_executor_alive ?? value.handoff.rosExecutorAlive)
          : null,
        localizationFrameAgeSeconds: finiteNumber(
          value.handoff.localization_frame_age_seconds ?? value.handoff.localizationFrameAgeSeconds,
        ),
        anchorGeneration: finiteNumber(
          value.handoff.anchor_generation ?? value.handoff.anchorGeneration,
        ),
        failureReason: value.handoff.handoff_failure_reason || value.handoff.failureReason || '',
        startedAt: firstTimestamp(value.handoff.started_at, value.handoff.startedAt),
        updatedAt: firstTimestamp(value.handoff.updated_at, value.handoff.updatedAt),
        finishedAt: firstTimestamp(value.handoff.finished_at, value.handoff.finishedAt),
      }
    : null
  return {
    status: value.status || '',
    verified: value.verified === true,
    conclusionCode: value.conclusion_code || value.conclusionCode || '',
    conclusion: value.conclusion || '',
    source: value.source || '',
    sampleCount: Number(value.sample_count ?? value.sampleCount ?? 0),
    stableFrames: Number(value.stable_frames ?? value.stableFrames ?? 0),
    requiredStableFrames: Number(value.required_stable_frames ?? value.requiredStableFrames ?? 0),
    spanM: finiteNumber(value.span_m ?? value.spanM),
    thresholdM: finiteNumber(value.threshold_xy_m ?? value.thresholdM),
    timedOut: value.timed_out === true || value.timedOut === true,
    checks: value.checks && typeof value.checks === 'object' ? value.checks : {},
    lastSample: normalizeRtkSample(value.last_sample || value.lastSample),
    sampleHistory: Array.isArray(value.sample_history ?? value.sampleHistory)
      ? (value.sample_history ?? value.sampleHistory).map(normalizeRtkSample).filter(Boolean)
      : [],
    handoff,
  }
}

export function rtkVerificationConclusionLabel(verification) {
  if (!verification) return ''
  return RTK_VERIFICATION_CONCLUSION_LABELS[verification.conclusionCode]
    || verification.conclusion
    || verification.conclusionCode
    || '等待验证结论'
}

export function rtkVerificationReasonLabel(reason) {
  const value = String(reason || '').trim()
  return RTK_VERIFICATION_REASON_LABELS[value] || value
}

export function formatRtkVerificationSummary(verification) {
  if (!verification) return '等待 RTK 固定解验证数据'
  const sample = verification.lastSample || {}
  const quality = sample.quality === 'fixed'
    ? '固定解'
    : (sample.quality === 'float' ? '浮点解' : (sample.quality || '未知解'))
  const parts = [
    `解状态 ${quality}`,
    `位置${sample.usable ? '可用' : '不可用'}`,
    `航向${sample.headingUsable ? '可用' : '不可用'}`,
    `稳定样本 ${verification.stableFrames}/${verification.requiredStableFrames}`,
  ]
  if (verification.spanM !== null || verification.thresholdM !== null) {
    parts.push(`位置跨度 ${formatAttemptMetric(verification.spanM)} / ${formatAttemptMetric(verification.thresholdM)} m`)
  }
  parts.push(`结论：${rtkVerificationConclusionLabel(verification)}`)
  return parts.join(' · ')
}

function firstTimestamp(...values) {
  return values.find(value => value !== null && value !== undefined && value !== '') || null
}

export function attemptSeedPose(attempt) {
  return finitePose(attempt?.seedPose || attempt?.seed_pose) || finitePose(attempt)
}

// Candidate markers represent the pose that was sent to NDT.  The matched
// pose is an output of verification and can be displaced from the candidate
// seed, so it must not change which map location the numbered marker denotes.
export function attemptMarkerPose(attempt) {
  return attemptSeedPose(attempt)
    || finitePose(attempt?.matchedPose || attempt?.matched_pose)
}

function commandResult(command) {
  return command?.result_payload && typeof command.result_payload === 'object'
    ? command.result_payload
    : {}
}

function commandMapIdentity(command, result = commandResult(command)) {
  const raw = result?.localization_attempts && typeof result.localization_attempts === 'object'
    ? result.localization_attempts
    : {}
  const payload = command?.payload && typeof command.payload === 'object' ? command.payload : {}
  const route = payload.route_snapshot && typeof payload.route_snapshot === 'object'
    ? payload.route_snapshot : {}
  const map = payload.map && typeof payload.map === 'object'
    ? payload.map
    : (route.map && typeof route.map === 'object' ? route.map : {})
  return {
    mapId: String(result.map_id ?? raw.map_id ?? map.map_id ?? payload.map_id ?? ''),
    mapVersion: String(result.map_version ?? raw.map_version ?? map.map_version ?? payload.map_version ?? ''),
  }
}

function normalizeAttempt(attempt, index) {
  const candidate = attempt?.ndt_candidate && typeof attempt.ndt_candidate === 'object'
    ? attempt.ndt_candidate
    : {}
  const seed = attemptSeedPose(attempt)
  const candidateNumber = Number(
    attempt?.candidate_number ?? attempt?.candidateNumber ?? attempt?.index,
  )
  const displayNumber = Number.isFinite(candidateNumber) ? candidateNumber : index + 1
  const attemptRejectReason = attempt?.reject_reason || attempt?.rejectReason || ''
  const candidateRejectReason = candidate.reject_reason
    || (Array.isArray(candidate.quality_failures) ? candidate.quality_failures[0] : '')
    || ''
  // Older Edge progress frames flattened every detailed safety rejection to
  // `quality_gate`.  Prefer the NDT candidate's causal gate when it exists:
  // e.g. a 78° seed-yaw correction is not an NDT-score failure.
  const rejectReason = attemptRejectReason === 'quality_gate' && candidateRejectReason
    ? candidateRejectReason
    : (attemptRejectReason || candidateRejectReason || attempt?.error_code || '')
  return {
    // Keep index for protocol compatibility; candidateNumber is the one
    // display identity shared by the map marker and the stage list.
    index: displayNumber,
    candidateNumber: displayNumber,
    candidateLabel: String(attempt?.candidate_label ?? attempt?.candidateLabel ?? ''),
    status: String(attempt?.status || 'waiting'),
    seedPose: seed,
    livePose: finitePose(attempt?.live_pose || attempt?.livePose),
    matchedPose: finitePose(attempt?.matched_pose || attempt?.matchedPose || candidate.matched_pose),
    matchingError: finiteNumber(candidate.matching_error ?? attempt?.matching_error),
    inlierFraction: finiteNumber(candidate.inlier_fraction ?? attempt?.inlier_fraction),
    geometricRmse: finiteNumber(candidate.geometric_rmse ?? attempt?.geometric_rmse),
    hasConverged: typeof (candidate.has_converged ?? attempt?.has_converged) === 'boolean'
      ? Boolean(candidate.has_converged ?? attempt?.has_converged)
      : null,
    stableFrames: Number(candidate.stable_frames || attempt?.stable_frames || 0),
    requiredStableFrames: Number(candidate.required_stable_frames || attempt?.required_stable_frames || 0),
    qualityFailures: Array.isArray(candidate.quality_failures ?? attempt?.quality_failures)
      ? [...(candidate.quality_failures ?? attempt?.quality_failures)]
      : [],
    rejectReason,
    eligible: Boolean(attempt?.eligible || candidate.eligible),
    accepted: attempt?.accepted === true || attempt?.status === 'accepted',
    stage: attempt?.stage || '',
    source: attempt?.source || '',
    waypointIndex: attempt?.waypoint_index,
    startedAt: firstTimestamp(attempt?.started_at, attempt?.startedAt),
    finishedAt: firstTimestamp(attempt?.finished_at, attempt?.finishedAt),
  }
}

function stageStatusFromEvidence(value, fallback = 'waiting') {
  if (!value || typeof value !== 'object') return fallback
  if (value.accepted === true || value.verified === true) return 'accepted'
  if (value.ready === true || value.navigation_allowed === true || value.active === true) return 'accepted'
  const status = String(value.status || value.state || '').trim().toLowerCase()
  if (['accepted', 'succeeded', 'completed', 'ready', 'passed'].includes(status)) return 'accepted'
  if (['skipped', 'unavailable'].includes(status)) return 'skipped'
  if (['failed', 'rejected', 'error', 'timed_out'].includes(status)) return 'failed'
  return status || fallback
}

function usesNavigationLifecycleStartupChain(session) {
  const commandType = String(session?.commandType || '').trim().toLowerCase()
  if (['task.start', 'nav.start', 'nav.restart', 'nav.recover', 'map.activate'].includes(commandType)) {
    return true
  }
  if (session?.initialNdtCommit || session?.finalLocalizationGate || session?.trustedRtkSeed) return true
  const lifecycleStages = new Set([
    'navigation_prepare',
    'fast_lio_readiness',
    'trusted_rtk_fixed',
    'final_localization_gate',
    'navigation_execution_activate',
  ])
  return (session?.stages || []).some(record => lifecycleStages.has(
    canonicalTimelineStage(record?.stage),
  ))
}

export function isAttemptSessionTerminal(session) {
  return ATTEMPT_TERMINAL_STATES.has(String(session?.status || '').toLowerCase())
}

export function withAttemptMarkerExpiry(session, now = Date.now()) {
  if (!session || !isAttemptSessionTerminal(session)) return session
  if (Number.isFinite(Number(session.markersVisibleUntil))) return session
  return {
    ...session,
    markersVisibleUntil: now + ATTEMPT_MARKER_VISIBLE_MS,
  }
}

export function localizationAttemptSessionFromCommand(command, extras = {}) {
  if (!command) return null
  const result = commandResult(command)
  const nestedLocalization = result.localization && typeof result.localization === 'object'
    ? result.localization
    : {}
  const mapIdentity = commandMapIdentity(command, result)
  const raw = result.localization_attempts && typeof result.localization_attempts === 'object'
    ? result.localization_attempts
    : (nestedLocalization.localization_attempts && typeof nestedLocalization.localization_attempts === 'object'
      ? nestedLocalization.localization_attempts
      : {})
  const attemptSource = Array.isArray(raw.attempts)
    ? raw.attempts
    : (Array.isArray(result.attempts) ? result.attempts : [])
  const rawStages = Array.isArray(raw.stages)
    ? raw.stages
    : (Array.isArray(result.stages) ? result.stages : [])
  const selectedLifecycleStage = canonicalTimelineStage(
    raw.selected_stage || result.selected_stage,
  )
  // Task-start progress packets report one current lifecycle stage rather
  // than a complete NDT attempt array. Preserve that evidence so the route
  // planner can show prepare/LIO/final-gate/execution activation live.
  const displayStages = [...rawStages]
  if (
    selectedLifecycleStage
    && TIMELINE_STAGE_META[selectedLifecycleStage]
    && !displayStages.some(record => canonicalTimelineStage(record?.stage) === selectedLifecycleStage)
  ) {
    displayStages.push({
      stage: selectedLifecycleStage,
      status: raw.state || result.state || command.status || 'running',
      started_at: firstTimestamp(command.started_at, command.issued_at),
      updated_at: firstTimestamp(command.updated_at, command.finished_at),
    })
  }
  const rtkStage = rawStages.find(record => canonicalTimelineStage(record?.stage) === 'rtk_fixed')
  const initialNdtCommit = result.initial_ndt_commit && typeof result.initial_ndt_commit === 'object'
    ? result.initial_ndt_commit
    : (nestedLocalization.initial_ndt_commit && typeof nestedLocalization.initial_ndt_commit === 'object'
      ? nestedLocalization.initial_ndt_commit
      : null)
  const trustedRtkSeed = raw.trusted_rtk_seed
    || result.trusted_rtk_seed
    || nestedLocalization.trusted_rtk_seed
    || initialNdtCommit?.trusted_rtk_seed
    || null
  if (trustedRtkSeed) {
    const existingSeedStageIndex = displayStages.findIndex(
      record => canonicalTimelineStage(record?.stage) === 'trusted_rtk_fixed',
    )
    const seedStage = {
      stage: 'trusted_rtk_fixed',
      status: trustedRtkSeed.status || (trustedRtkSeed.accepted ? 'accepted' : 'skipped'),
      started_at: firstTimestamp(trustedRtkSeed.started_at, command.started_at, command.issued_at),
      finished_at: firstTimestamp(trustedRtkSeed.finished_at, command.finished_at),
      message: trustedRtkSeed.reason || trustedRtkSeed.rtk_verification?.conclusion,
      rtk_verification: trustedRtkSeed.rtk_verification,
    }
    if (existingSeedStageIndex >= 0) displayStages[existingSeedStageIndex] = seedStage
    else displayStages.push(seedStage)
  }
  const rtkVerification = normalizeRtkVerification(
    raw.rtk_verification
      || result.rtk_verification
      || rtkStage?.rtk_verification
      || raw.rtk_stability
      || result.rtk_stability,
  )
  const secondaryCorrection = raw.secondary_correction
    || result.secondary_correction
    || nestedLocalization.secondary_correction
    || null
  const explicitHandoff = raw.handoff
    || result.handoff
    || result.fast_lio_imu_handoff
    || nestedLocalization.handoff
    || nestedLocalization.fast_lio_imu_handoff
    || initialNdtCommit?.handoff
    || initialNdtCommit?.fast_lio_imu_handoff
    || raw.handoff_diagnostics
    || result.handoff_diagnostics
    || rtkVerification?.handoff
    || null
  const acceptedSelectedStage = rawStages.find(record => (
    canonicalTimelineStage(record?.stage) === canonicalTimelineStage(
      raw.selected_stage || result.selected_stage,
    )
    && timelineStatusClass(record?.status) === 'done'
  ))
  const secondaryCompleted = timelineStatusClass(secondaryCorrection?.status) === 'done'
  // Releases before the unified handoff contract omitted the handoff object
  // on a successful global-search path. A completed secondary correction can
  // only follow accepted initialization, so retain that causal evidence
  // instead of leaving the preceding handoff stage permanently "waiting".
  const handoff = explicitHandoff || (secondaryCompleted
    ? {
        status: 'completed',
        conclusion_code: 'inferred_from_completed_secondary_correction',
        conclusion: 'FAST-LIO + IMU handoff confirmed by completed secondary correction',
        inferred: true,
        started_at: firstTimestamp(
          raw.best_candidate_commit_finished_at,
          result.best_candidate_commit_finished_at,
          acceptedSelectedStage?.finished_at,
          acceptedSelectedStage?.finishedAt,
          secondaryCorrection?.started_at,
          secondaryCorrection?.startedAt,
        ),
        finished_at: firstTimestamp(
          secondaryCorrection?.started_at,
          secondaryCorrection?.startedAt,
          secondaryCorrection?.finished_at,
          secondaryCorrection?.finishedAt,
        ),
      }
    : null)
  const phase = extras.phase
    || (String(command.command_type || extras.commandType || '') === 'map.activate' ? 'transfer' : 'localization')
  const showCandidates = extras.showCandidates !== false && phase !== 'transfer'
  const bestMatchPose = finitePose(result.best_match_pose)
    || finitePose(raw.best_match_pose)
    || finitePose(raw.best_ndt_candidate?.matched_pose)
    || finitePose(result.best_ndt_candidate?.matched_pose)
  const session = {
    mapId: mapIdentity.mapId,
    mapVersion: mapIdentity.mapVersion,
    commandId: String(command.id || extras.commandId || ''),
    commandType: command.command_type || extras.commandType || '',
    commandIssuedAt: firstTimestamp(command.issued_at, command.created_at),
    commandStartedAt: firstTimestamp(command.started_at),
    commandFinishedAt: firstTimestamp(command.finished_at),
    observedAt: firstTimestamp(
      extras.observedAt,
      command.updated_at,
      command.finished_at,
      command.started_at,
      command.issued_at,
      command.created_at,
    ) || new Date().toISOString(),
    phase,
    showCandidates,
    status: raw.state || result.state || nestedLocalization.state || command.status || '',
    livePose: finitePose(raw.live_pose) || finitePose(result.live_pose),
    attempts: attemptSource.map(normalizeAttempt),
    bestMatchPose,
    bestNdtCandidate: raw.best_ndt_candidate || result.best_ndt_candidate || null,
    source: raw.selected_stage || result.selected_stage || nestedLocalization.selected_stage || raw.source || extras.source || '',
    startupProgress: result.startup_progress && typeof result.startup_progress === 'object'
      ? result.startup_progress
      : null,
    initialNdtCommit,
    earlyStopped: Boolean(raw.early_stopped ?? result.early_stopped),
    stopReason: raw.stop_reason || result.stop_reason || '',
    candidateCount: Number(raw.candidate_count ?? result.candidate_count ?? attemptSource.length),
    evaluatedCandidateCount: Number(raw.evaluated_candidate_count ?? result.evaluated_candidate_count ?? attemptSource.length),
    globalSearchStarted: Boolean(raw.global_search_started ?? result.global_search_started),
    activeCandidateNumber: finiteNumber(
      raw.active_candidate_number ?? result.active_candidate_number,
    ),
    activeCandidateStage: raw.active_candidate_stage || result.active_candidate_stage || '',
    rtkDrift: raw.rtk_drift || result.rtk_drift || null,
    rtkVerification,
    trustedRtkSeed,
    handoff,
    secondaryCorrection,
    finalLocalizationGate: raw.final_localization_gate
      || result.final_localization_gate
      || nestedLocalization.final_localization_gate
      || null,
    sceneScope: String(
      raw.scene_scope
      || result.scene_scope
      || command?.payload?.scene_scope
      || command?.payload?.command?.scene_scope
      || '',
    ).trim().toLowerCase(),
    bestNdtCommitted: Boolean(raw.best_ndt_committed ?? result.best_ndt_committed),
    rtkFixedCommitted: Boolean(raw.rtk_fixed_committed ?? result.rtk_fixed_committed),
    strategy: Array.isArray(raw.strategy)
      ? raw.strategy
      : (Array.isArray(result.strategy)
        ? result.strategy
        : (extras.source === 'rtk'
          ? ['rtk_fixed']
          : (command.command_type === 'nav.initial_pose' ? ['operator_initial_pose'] : []))),
    stages: displayStages.length
      ? displayStages
      : (extras.source === 'rtk'
          ? [{ stage: 'rtk_fixed', status: command.status || 'searching' }]
          : (command.command_type === 'nav.initial_pose'
            ? [{ stage: 'operator_initial_pose', status: command.status || 'searching' }]
            : [])),
    localizationBootstrap: result.localization_bootstrap || raw.localization_bootstrap || null,
    navigationStart: result.navigation_start || raw.navigation_start || null,
    bestCandidateCommitStartedAt: firstTimestamp(
      raw.best_candidate_commit_started_at,
      result.best_candidate_commit_started_at,
    ),
    bestCandidateCommitFinishedAt: firstTimestamp(
      raw.best_candidate_commit_finished_at,
      result.best_candidate_commit_finished_at,
    ),
    bestCandidateIndex: finiteNumber(
      raw.best_candidate_index ?? result.best_candidate_index,
    ),
    bestCandidateStage: raw.best_candidate_stage || result.best_candidate_stage || '',
    bestCandidateSeedPose: finitePose(
      raw.best_candidate_seed_pose || result.best_candidate_seed_pose,
    ),
    bestCandidateLabel: String(
      raw.best_candidate_label
      ?? result.best_candidate_label
      ?? '',
    ),
    bestCandidateNdt: raw.best_candidate_ndt || result.best_candidate_ndt || null,
    bestCandidateHandoffPending: Boolean(
      raw.handoff_pending ?? result.handoff_pending,
    ),
    selectedStage: raw.selected_stage || result.selected_stage || '',
    timelineHistory: Array.isArray(extras.timelineHistory) ? extras.timelineHistory : [],
  }
  const score = finiteNumber(session.bestNdtCandidate?.matching_error)
  session.optimalVerified = Boolean(
    session.rtkVerification?.verified
    || session.rtkDrift?.verified
    || session.rtkFixedCommitted
    || (session.bestNdtCommitted && score !== null && score < 0.01),
  )
  return withAttemptMarkerExpiry(session)
}

export function emptyAttemptSession({ phase = 'localization', commandType = '', commandId = '' } = {}) {
  return {
    mapId: '',
    mapVersion: '',
    commandId: String(commandId || ''),
    commandType,
    commandIssuedAt: null,
    commandStartedAt: null,
    commandFinishedAt: null,
    observedAt: new Date().toISOString(),
    phase,
    showCandidates: phase !== 'transfer',
    status: '',
    livePose: null,
    attempts: [],
    bestMatchPose: null,
    bestNdtCandidate: null,
    source: '',
    earlyStopped: false,
    stopReason: '',
    candidateCount: 0,
    evaluatedCandidateCount: 0,
    globalSearchStarted: false,
    activeCandidateNumber: null,
    activeCandidateStage: '',
    rtkDrift: null,
    rtkVerification: null,
    trustedRtkSeed: null,
    handoff: null,
    secondaryCorrection: null,
    finalLocalizationGate: null,
    sceneScope: '',
    bestNdtCommitted: false,
    rtkFixedCommitted: false,
    optimalVerified: false,
    strategy: [],
    stages: [],
    localizationBootstrap: null,
    navigationStart: null,
    bestCandidateCommitStartedAt: null,
    bestCandidateCommitFinishedAt: null,
    bestCandidateIndex: null,
    bestCandidateStage: '',
    bestCandidateSeedPose: null,
    bestCandidateLabel: '',
    bestCandidateNdt: null,
    bestCandidateHandoffPending: false,
    selectedStage: '',
    timelineHistory: [],
  }
}

function canonicalTimelineStage(value) {
  const key = String(value || '').trim()
  return TIMELINE_STAGE_ALIASES[key] || key
}

function timelineStatusClass(status) {
  const normalized = String(status || '').trim().toLowerCase().replace(/[-\s]/g, '_')
  if (['accepted', 'succeeded', 'done', 'completed'].includes(normalized)) return 'done'
  if (['rejected', 'failed', 'unavailable', 'error'].includes(normalized)) return 'failed'
  if (['searching', 'running', 'verifying', 'executing', 'active', 'started', 'in_progress'].includes(normalized)) return 'active'
  if (['skipped', 'cancelled', 'canceled'].includes(normalized)) return 'skipped'
  return 'waiting'
}

function timelineStatusLabel(status, stageKey = '') {
  const labels = {
    waiting: '待执行',
    active: '执行中',
    done: '已完成',
    failed: '失败',
    skipped: '已跳过',
  }
  if (timelineStatusClass(status) === 'failed' && ['mapping_origin_bounded', 'route_waypoints'].includes(stageKey)) {
    return '未通过，已转下一阶段'
  }
  return labels[timelineStatusClass(status)]
}

function stageAttemptsFor(session, stageKey, stageRecord = null) {
  const aliases = new Set([stageKey])
  Object.entries(TIMELINE_STAGE_ALIASES).forEach(([alias, canonical]) => {
    if (canonical === stageKey) aliases.add(alias)
  })
  const recordAttemptIndexes = new Set(
    (stageRecord?.attempts || [])
      .map(attempt => Number(attempt?.candidate_number ?? attempt?.candidateNumber ?? attempt?.index))
      .filter(index => Number.isFinite(index)),
  )
  const attempts = (session?.attempts || []).filter(attempt => (
    aliases.has(canonicalTimelineStage(attempt.stage))
    || (stageKey === 'mapping_origin_bounded' && attempt.source === 'mapping_origin')
    || (stageKey === 'mapping_origin_bounded'
      && !attempt.stage
      && (session?.source === 'mapping_origin' || recordAttemptIndexes.has(Number(attempt.candidateNumber))))
  ))
  if (attempts.length || !Array.isArray(stageRecord?.attempts)) return attempts
  return stageRecord.attempts.map((attempt, index) => normalizeAttempt({
    ...attempt,
    stage: attempt?.stage || stageKey,
  }, index))
}

function inferredStageStatus(session, stageKey, attempts, stageRecord) {
  // A completed stage is authoritative. A delayed candidate progress packet
  // must not turn a rejected/failed stage back into "执行中".
  if (stageRecord?.status && ['done', 'failed', 'skipped'].includes(timelineStatusClass(stageRecord.status))) {
    return stageRecord.status
  }
  if (attempts.some(attempt => ['verifying', 'started', 'running', 'executing', 'in_progress', 'committing'].includes(attempt.status))) return 'searching'
  if (stageRecord?.status && timelineStatusClass(stageRecord.status) !== 'waiting') return stageRecord.status
  if (stageRecord?.status) return stageRecord.status
  if (stageKey === 'fast_lio_imu_handoff' && session?.handoff) {
    return session.handoff.status || (session.handoff.handoff_state === 'accepted' ? 'accepted' : 'failed')
  }
  if (stageKey === 'secondary_correction' && session?.secondaryCorrection) {
    return session.secondaryCorrection.status || 'searching'
  }
  if (stageKey === 'final_localization_gate' && session?.finalLocalizationGate) {
    return stageStatusFromEvidence(session.finalLocalizationGate, 'searching')
  }
  if (stageKey === 'trusted_rtk_fixed') {
    if (session?.trustedRtkSeed) return stageStatusFromEvidence(session.trustedRtkSeed, 'searching')
    if (session?.sceneScope === 'indoor') return 'skipped'
  }
  if (attempts.some(attempt => attempt.status === 'accepted')) return 'accepted'
  if (attempts.length && attempts.every(attempt => ['rejected', 'failed', 'skipped'].includes(attempt.status))) return 'rejected'
  const selectedStage = canonicalTimelineStage(session?.selectedStage)
  if (selectedStage === stageKey) {
    return timelineStatusClass(session?.status) === 'failed' ? 'failed' : 'searching'
  }
  // The command progress protocol advances one canonical stage at a time.
  // Once it has moved past a mandatory stage, that predecessor completed;
  // retain explicit failures/skips above instead of leaving its row forever
  // in "执行中" when the next packet arrives.
  const selectedIndex = TIMELINE_STAGE_ORDER.indexOf(selectedStage)
  const stageIndex = TIMELINE_STAGE_ORDER.indexOf(stageKey)
  if (selectedIndex >= 0 && stageIndex >= 0 && selectedIndex > stageIndex) {
    return stageKey === 'trusted_rtk_fixed' && !session?.trustedRtkSeed
      ? 'skipped'
      : 'accepted'
  }
  if (!isAttemptSessionTerminal(session)) {
    const firstStage = canonicalTimelineStage((session?.strategy || [])[0] || 'mapping_origin_bounded')
    if (firstStage === stageKey) return 'searching'
  }
  return 'waiting'
}

function timelineDetail(stageKey, status, attempts, session, stageRecord = null) {
  const meta = TIMELINE_STAGE_META[stageKey] || { title: stageKey, detail: '' }
  if (stageKey === 'map_transfer' && session?.phase === 'transfer') {
    return status === 'done' ? '地图下发并应用完成' : '正在等待机器狗确认地图命令'
  }
  if (stageKey === 'fast_lio_imu_handoff' && session?.handoff?.inferred) {
    return '兼容旧结果：后续二次定位校正已完成，确认 FAST-LIO + IMU 主定位接管已完成'
  }
  if (stageKey === 'rtk_fixed') {
    const verification = normalizeRtkVerification(stageRecord?.rtk_verification)
      || session?.rtkVerification
    return formatRtkVerificationSummary(verification)
  }
  if (stageKey === 'trusted_rtk_fixed') {
    const seed = session?.trustedRtkSeed
    if (!seed) return status === 'skipped'
      ? '室内地图或无合格 fixed RTK，本轮不使用 RTK 搜索种子'
      : meta.detail
    const verification = normalizeRtkVerification(seed.rtk_verification)
    const reason = seed.reason || verification?.conclusion || ''
    const confirmation = seed.ndt_confirmation || stageRecord?.ndt_confirmation || null
    if (seed.accepted && confirmation?.confirmed) {
      const candidateNumber = confirmation.candidate_number ?? 1
      const score = formatAttemptMetric(
        confirmation.ndt_score ?? confirmation.matching_error,
      )
      const pose = formatAttemptPose(
        confirmation.matched_pose || seed.confirmed_pose,
      )
      return [
        `fixed RTK 已经 NDT 确认 · 候选 #${candidateNumber}`,
        `NDT ${score}`,
        `定位结果 ${pose}`,
      ].join(' · ')
    }
    return [
      seed.accepted ? 'fixed RTK 已作为 NDT 候选种子' : 'fixed RTK 未作为搜索种子',
      reason,
    ].filter(Boolean).join(' · ')
  }
  if (stageKey === 'final_localization_gate') {
    const gate = session?.finalLocalizationGate
    if (!gate) return meta.detail
    const frames = Number(gate.stable_frames ?? gate.normal_samples ?? 0)
    const required = Number(gate.required_stable_frames ?? gate.required_samples ?? 3)
    const source = gate.continuous_source || gate.active_source || session?.continuousSource || 'lio_imu'
    const reason = gate.reason || gate.message || gate.failure_reason || ''
    return [
      `连续状态帧 ${frames}/${required}`,
      `主源 ${source}`,
      reason,
    ].filter(Boolean).join(' · ')
  }
  if (stageKey === 'best_candidate_commit') {
    if (timelineStatusClass(status) === 'waiting') return meta.detail
    const selected = (session?.attempts || []).find(attempt => (
      session?.bestCandidateIndex !== null
      && session?.bestCandidateIndex !== undefined
      && Number(attempt.candidateNumber) === Number(session.bestCandidateIndex)
    ))
    const candidateNumber = session?.bestCandidateIndex ?? selected?.candidateNumber
    const pose = session?.bestMatchPose || selected?.matchedPose
    const score = selected?.matchingError ?? finiteNumber(session?.bestNdtCandidate?.matching_error)
    const inlier = selected?.inlierFraction ?? finiteNumber(session?.bestNdtCandidate?.inlier_fraction)
    const source = session?.bestCandidateStage || selected?.stage || session?.source || 'NDT'
    const committed = timelineStatusClass(status) === 'done'
    const prefix = candidateNumber === null || candidateNumber === undefined
      ? (committed ? '已提交最优候选' : '正在提交最优候选')
      : `${committed ? '已提交' : '正在提交'}候选 #${candidateNumber}`
    const metrics = [
      `${prefix}（${source}）`,
      ...(session?.bestCandidateLabel ? [`候选点 ${session.bestCandidateLabel}`] : []),
      ...(session?.bestCandidateSeedPose ? [`种子 ${formatAttemptPose(session.bestCandidateSeedPose)}`] : []),
      `匹配位姿 ${formatAttemptPose(pose)}`,
      `NDT ${formatAttemptMetric(score)}`,
    ]
    if (inlier !== null && inlier !== undefined) metrics.push(`内点 ${(Number(inlier) * 100).toFixed(1)}%`)
    return metrics.join(' · ')
  }
  if (attempts.length) {
    const stageTerminal = ['done', 'failed', 'skipped'].includes(timelineStatusClass(status))
    const active = attempts.find(attempt => (
      ['verifying', 'started', 'running', 'executing', 'in_progress'].includes(attempt.status)
    ))
    const committing = attempts.find(attempt => attempt.status === 'committing')
    const evaluated = attempts.filter(attempt => (
      ['qualified', 'accepted', 'rejected', 'failed', 'committing'].includes(attempt.status)
    )).length
    const skipped = attempts.filter(attempt => attempt.status === 'skipped').length
    const activeText = stageTerminal ? '' : (active
      ? `正在尝试 #${active.candidateNumber}`
      : (committing ? `正在提交 #${committing.candidateNumber}` : ''))
    const skippedText = skipped ? ` · 已跳过 ${skipped}` : ''
    return `${meta.detail} · ${activeText ? `${activeText} · ` : ''}已评估 ${evaluated}/${attempts.length} 个候选${skippedText}`
  }
  return meta.detail
}

function timelineStageTimes(stageKey, status, record, session) {
  const historical = (session?.timelineHistory || []).find(item => item?.key === stageKey)
  let startedAt = firstTimestamp(record?.started_at, record?.startedAt)
  let finishedAt = firstTimestamp(record?.finished_at, record?.finishedAt)

  // A stage that has not started must not inherit the command timestamp. That
  // made future stages look as if they had started before the active stage.
  if (!startedAt && timelineStatusClass(status) === 'waiting') {
    return { startedAt: null, finishedAt: null }
  }

  if (stageKey === 'localization_bootstrap') {
    startedAt = firstTimestamp(
      startedAt,
      session?.localizationBootstrap?.started_at,
      session?.localizationBootstrap?.startedAt,
    )
    finishedAt = firstTimestamp(
      finishedAt,
      session?.localizationBootstrap?.finished_at,
      session?.localizationBootstrap?.finishedAt,
    )
  } else if (stageKey === 'best_candidate_commit') {
    startedAt = firstTimestamp(startedAt, session?.bestCandidateCommitStartedAt)
    finishedAt = firstTimestamp(finishedAt, session?.bestCandidateCommitFinishedAt)
  } else if (stageKey === 'navigation_start') {
    startedAt = firstTimestamp(
      startedAt,
      session?.navigationStart?.started_at,
      session?.navigationStart?.startedAt,
    )
    finishedAt = firstTimestamp(
      finishedAt,
      session?.navigationStart?.finished_at,
      session?.navigationStart?.finishedAt,
    )
  } else if (stageKey === 'fast_lio_imu_handoff') {
    startedAt = firstTimestamp(
      startedAt,
      session?.handoff?.started_at,
      session?.handoff?.startedAt,
    )
    finishedAt = firstTimestamp(
      finishedAt,
      session?.handoff?.finished_at,
      session?.handoff?.finishedAt,
    )
  } else if (stageKey === 'secondary_correction') {
    startedAt = firstTimestamp(
      startedAt,
      session?.secondaryCorrection?.started_at,
      session?.secondaryCorrection?.startedAt,
    )
    finishedAt = firstTimestamp(
      finishedAt,
      session?.secondaryCorrection?.finished_at,
      session?.secondaryCorrection?.finishedAt,
    )
  } else if (stageKey === 'map_transfer' && session?.phase !== 'transfer') {
    startedAt = firstTimestamp(startedAt, historical?.startedAt)
    finishedAt = firstTimestamp(finishedAt, historical?.finishedAt)
  }
  const hasStageEvidence = Boolean(
    record
    || (stageKey === 'localization_bootstrap' && session?.localizationBootstrap)
    || (stageKey === 'best_candidate_commit' && (session?.bestNdtCommitted || session?.bestMatchPose))
    || (stageKey === 'fast_lio_imu_handoff' && session?.handoff)
    || (stageKey === 'secondary_correction' && session?.secondaryCorrection)
    || (stageKey === 'navigation_start' && session?.navigationStart),
  )
  if (!startedAt && timelineStatusClass(status) !== 'waiting') {
    startedAt = firstTimestamp(
      historical?.startedAt,
      session?.commandStartedAt,
      session?.commandIssuedAt,
      session?.commandFinishedAt,
      session?.observedAt,
    )
  }
  if (
    !finishedAt
    && ['done', 'failed', 'skipped'].includes(timelineStatusClass(status))
  ) {
    // Synthetic prerequisite rows (for example map transfer on a direct
    // relocalize command) have no independent completion packet. Close them
    // at their inferred start instead of using the whole command's finish;
    // otherwise chronological clamping pushes every real NDT stage to the
    // command completion time.
    finishedAt = hasStageEvidence
      ? firstTimestamp(
          record?.updated_at,
          record?.updatedAt,
          historical?.finishedAt,
          session?.commandFinishedAt,
          session?.observedAt,
          startedAt,
        )
      : startedAt
  }
  return { startedAt, finishedAt }
}

function timestampMillis(value) {
  if (!value) return null
  const millis = Date.parse(value)
  return Number.isFinite(millis) ? millis : null
}

function orderedTimelineTimes(session, timeline) {
  let cursor = null
  return timeline.map(step => {
    let startedAt = step.startedAt
    let finishedAt = step.finishedAt
    let startedMillis = timestampMillis(startedAt)
    let finishedMillis = timestampMillis(finishedAt)

    // Direct relocalization has no independent map-transfer packet. Its
    // compatibility row receives an observation timestamp, which can be much
    // newer than the reported NDT/handoff evidence. Do not let that synthetic
    // prerequisite move real stage timestamps forward.
    const nonBlockingSynthetic = (
      (step.key === 'map_transfer' && session?.phase !== 'transfer')
      || (step.key === 'localization_bootstrap' && !session?.localizationBootstrap)
      // nav.initial_pose legacy payloads synthesize this row from the command
      // type.  A synthesized terminal row has no independent timestamps and
      // must not move the reported RTK/NDT handoff evidence forward.
      || (step.key === 'operator_initial_pose' && !(session?.stages || []).some(
        record => (
          canonicalTimelineStage(record?.stage) === 'operator_initial_pose'
          && firstTimestamp(record?.started_at, record?.startedAt, record?.finished_at, record?.finishedAt)
        )
      ))
    )
    if (!nonBlockingSynthetic && startedMillis !== null && cursor !== null && startedMillis < cursor) {
      startedMillis = cursor
      startedAt = new Date(startedMillis).toISOString()
    }
    if (finishedMillis !== null && startedMillis !== null && finishedMillis < startedMillis) {
      finishedMillis = startedMillis
      finishedAt = new Date(finishedMillis).toISOString()
    }
    if (!nonBlockingSynthetic) {
      if (finishedMillis !== null) cursor = finishedMillis
      else if (startedMillis !== null) cursor = Math.max(cursor ?? startedMillis, startedMillis)
    }

    return { ...step, startedAt, finishedAt }
  })
}

export function localizationAttemptTimeline(session) {
  if (!session) return []
  const timeline = []
  const add = (key, status, attempts = [], detail = '', record = null) => {
    const meta = TIMELINE_STAGE_META[key] || { title: key, detail: '' }
    const times = timelineStageTimes(key, status, record, session)
    timeline.push({
      key,
      title: meta.title,
      detail: detail || timelineDetail(key, status, attempts, session, record),
      status: timelineStatusClass(status),
      statusLabel: timelineStatusLabel(status, key),
      attempts,
      reported: Boolean(record)
        || attempts.length > 0
        || (key === 'trusted_rtk_fixed' && Boolean(session.trustedRtkSeed))
        || (key === 'fast_lio_imu_handoff' && Boolean(session.handoff))
        || (key === 'secondary_correction' && Boolean(session.secondaryCorrection))
        || (key === 'final_localization_gate' && Boolean(session.finalLocalizationGate))
        || (key === 'navigation_start' && Boolean(session.navigationStart)),
      startedAt: times.startedAt,
      finishedAt: times.finishedAt,
    })
  }

  const commandStatus = String(session.status || '').toLowerCase()
  const commandDone = isAttemptSessionTerminal(session)
  const transferStatus = session.phase === 'transfer'
    ? commandStatus
    : 'accepted'
  add('map_transfer', transferStatus)
  if (session.phase === 'transfer') {
    return orderedTimelineTimes(session, mergeTimelineHistory(session, timeline))
  }

  const bootstrapStatus = session.localizationBootstrap
    ? 'accepted'
    : (commandDone ? 'accepted' : 'searching')
  add('localization_bootstrap', bootstrapStatus)

  const stageRecords = new Map()
  ;(session.stages || []).forEach(record => {
    const key = canonicalTimelineStage(record?.stage)
    if (key && !stageRecords.has(key)) stageRecords.set(key, record)
  })
  const stageKeys = []
  const isLifecycleStartup = usesNavigationLifecycleStartupChain(session)
  const addStageKey = value => {
    const key = canonicalTimelineStage(value)
    if (!key || key === 'map_transfer' || key === 'localization_bootstrap' || stageKeys.includes(key)) return
    if (TIMELINE_STAGE_META[key] || stageRecords.has(key)) stageKeys.push(key)
  }
  // A navigation start always exposes every admission gate.  Direct/manual
  // localization remains concise: it only shows the search stages it ran.
  if (isLifecycleStartup) {
    ;[
      'navigation_prepare',
      'fast_lio_readiness',
      'trusted_rtk_fixed',
      'mapping_origin_bounded',
    ].forEach(addStageKey)
  }
  ;(session.strategy || []).forEach(addStageKey)
  ;(session.stages || []).forEach(record => addStageKey(record?.stage))
  ;(session.attempts || []).forEach(attempt => addStageKey(attempt?.stage))
  if ((session.rtkVerification || session.rtkDrift) && !stageKeys.includes('rtk_fixed')) stageKeys.unshift('rtk_fixed')
  if (!stageKeys.length) stageKeys.push('mapping_origin_bounded')
  // Handoff and secondary correction remain visible after direct localization
  // for compatibility.  The final admission/activation rows are specific to
  // a lifecycle-managed navigation start.
  addStageKey('fast_lio_imu_handoff')
  addStageKey('secondary_correction')
  if (isLifecycleStartup) {
    addStageKey('final_localization_gate')
    addStageKey('navigation_execution_activate')
  }
  stageKeys.forEach(key => {
    const record = stageRecords.get(key)
    const attempts = stageAttemptsFor(session, key, record)
    add(
      key,
      inferredStageStatus(session, key, attempts, record),
      attempts,
      ['rtk_fixed', 'trusted_rtk_fixed', 'final_localization_gate'].includes(key)
        ? ''
        : (record?.error_message || record?.message || ''),
      record,
    )
  })

  // The RTK transaction commits its authoritative anchor inside rtk_fixed;
  // adding a second "提交最优 NDT" node misrepresents the cross-check as the
  // source of the absolute pose. The global matcher likewise applies its
  // verified candidate inside keyframe_global_match, so it must not create a
  // synthetic commit row that pushes the real handoff time to command end.
  const selectedStageKey = canonicalTimelineStage(session.selectedStage)
  const globalCandidateApplied = selectedStageKey === 'keyframe_global_match'
    && timelineStatusClass(stageRecords.get('keyframe_global_match')?.status) === 'done'
  if (!session.rtkFixedCommitted && !globalCandidateApplied) {
    const commitStatus = session.bestNdtCommitted || (commandDone && session.bestMatchPose)
      ? 'accepted'
      : (session.bestMatchPose ? 'searching' : 'waiting')
    add('best_candidate_commit', commitStatus)
  }

  const shouldShowNavigation = session.commandType !== 'nav.initial_pose' || session.navigationStart
  if (shouldShowNavigation) {
    const navigationStatus = session.navigationStart
      ? stageStatusFromEvidence(session.navigationStart, 'searching')
      : (commandDone && session.status === 'accepted' ? 'searching' : 'waiting')
    add('navigation_start', navigationStatus)
  }
  return orderedTimelineTimes(session, mergeTimelineHistory(session, timeline))
}

function mergeTimelineHistory(session, currentTimeline) {
  const history = Array.isArray(session?.timelineHistory) ? session.timelineHistory : []
  const merged = history.map(step => ({ ...step }))
  currentTimeline.forEach(step => {
    const index = merged.findIndex(item => item.key === step.key)
    if (index >= 0) {
      const previous = merged[index]
      // A progress packet normally reports one lifecycle checkpoint.  Keep
      // a prior concrete checkpoint when this packet only renders the
      // structural placeholder for that row; otherwise an accepted RTK seed
      // or final gate would appear to regress on the following NDT packet.
      if (!step.reported && previous?.reported) return
      merged[index] = step
    }
    else merged.push(step)
  })
  return merged.sort((left, right) => {
    const leftIndex = TIMELINE_STAGE_ORDER.indexOf(left.key)
    const rightIndex = TIMELINE_STAGE_ORDER.indexOf(right.key)
    return (leftIndex < 0 ? TIMELINE_STAGE_ORDER.length : leftIndex)
      - (rightIndex < 0 ? TIMELINE_STAGE_ORDER.length : rightIndex)
  })
}

export function shouldShowAttemptMarkers(session, now = Date.now()) {
  if (!session?.showCandidates || !Array.isArray(session.attempts) || !session.attempts.length) {
    return false
  }
  const visibleUntil = Number(session.markersVisibleUntil)
  if (Number.isFinite(visibleUntil) && now >= visibleUntil) return false
  return true
}

export function attemptStatusClass(status) {
  if (status === 'accepted') return 'accepted'
  if (status === 'qualified') return 'qualified'
  if (['verifying', 'started', 'running', 'executing', 'in_progress', 'committing'].includes(status)) return 'active'
  if (status === 'rejected' || status === 'failed') return 'failed'
  if (status === 'skipped') return 'skipped'
  return 'waiting'
}

export function attemptStatusLabel(status) {
  return ATTEMPT_STATUS_LABELS[status] || status || '等待'
}

export function attemptRejectReasonLabel(reason) {
  const value = String(reason || '').trim()
  return ATTEMPT_REJECT_REASON_LABELS[value] || value
}

export function localizationAttemptFailureMessage(session, fallback = '定位初始化失败') {
  if (!session) return fallback
  const attempts = Array.isArray(session.attempts) ? session.attempts : []
  const backendAccepted = ['accepted', 'succeeded', 'completed'].includes(
    String(session.status || '').trim().toLowerCase(),
  )
  const acceptedAttempt = attempts.find(attempt => attempt.accepted || attempt.status === 'accepted')
  if (backendAccepted && acceptedAttempt) {
    const score = acceptedAttempt.matchingError === null
      ? 'NDT —'
      : `NDT ${acceptedAttempt.matchingError.toFixed(3)}`
    const inlier = acceptedAttempt.inlierFraction === null
      ? '内点 —'
      : `内点 ${(acceptedAttempt.inlierFraction * 100).toFixed(1)}%`
    return `定位初始化已完成：候选 #${acceptedAttempt.candidateNumber} · ${score} · ${inlier}${fallback ? `；后续导航操作失败：${fallback}` : ''}`
  }
  const measured = attempts.filter(attempt => (
    attempt.matchingError !== null
    || attempt.inlierFraction !== null
  ))
  const observed = measured.length
    ? measured
    : attempts.filter(attempt => attempt.rejectReason)
  const ranked = [...observed].sort((left, right) => {
    if (left.hasConverged !== right.hasConverged) {
      if (left.hasConverged === true) return -1
      if (right.hasConverged === true) return 1
    }
    const leftScore = left.matchingError === null ? Number.POSITIVE_INFINITY : left.matchingError
    const rightScore = right.matchingError === null ? Number.POSITIVE_INFINITY : right.matchingError
    if (leftScore !== rightScore) return leftScore - rightScore
    const leftInlier = left.inlierFraction === null ? -1 : left.inlierFraction
    const rightInlier = right.inlierFraction === null ? -1 : right.inlierFraction
    return rightInlier - leftInlier
  })
  let diagnostic = measured.length ? (ranked[0] || null) : null
  if (session.bestNdtCandidate && !diagnostic) {
    const candidate = session.bestNdtCandidate
    diagnostic = {
      candidateNumber: null,
      matchingError: finiteNumber(candidate.matching_error),
      inlierFraction: finiteNumber(candidate.inlier_fraction),
      rejectReason: candidate.reject_reason || '',
      hasConverged: typeof candidate.has_converged === 'boolean'
        ? candidate.has_converged
        : null,
    }
  }
  if (!diagnostic) diagnostic = ranked[0] || null
  if (!diagnostic) return fallback
  const number = diagnostic.candidateNumber ? ` #${diagnostic.candidateNumber}` : ''
  const score = diagnostic.matchingError === null
    ? 'NDT —'
    : `NDT ${diagnostic.matchingError.toFixed(3)}`
  const inlier = diagnostic.inlierFraction === null
    ? '内点 —'
    : `内点 ${(diagnostic.inlierFraction * 100).toFixed(1)}%`
  const reason = attemptRejectReasonLabel(
    diagnostic.rejectReason || (diagnostic.hasConverged === false ? 'ndt_not_converged' : ''),
  )
  return `定位初始化未通过：最佳失败候选${number} · ${score} · ${inlier}${reason ? ` · ${reason}` : ''}`
}

export function formatAttemptPose(pose) {
  if (!pose) return '—'
  return `x ${pose.x.toFixed(3)} / y ${pose.y.toFixed(3)} / yaw ${pose.yaw.toFixed(3)}`
}

export function formatAttemptMetric(value, digits = 3) {
  const number = finiteNumber(value)
  return number === null ? '—' : number.toFixed(digits)
}

export function readStoredAttemptSession(robotId) {
  if (typeof sessionStorage === 'undefined' || !robotId) return null
  try {
    const raw = sessionStorage.getItem(localizationAttemptStorageKey(robotId))
    if (!raw) return null
    const session = JSON.parse(raw)
    return withAttemptMarkerExpiry({
      ...session,
      observedAt: session.observedAt || new Date().toISOString(),
      attempts: Array.isArray(session.attempts)
        ? session.attempts.map(normalizeAttempt)
        : [],
    })
  } catch {
    return null
  }
}

export function writeStoredAttemptSession(robotId, session) {
  if (typeof sessionStorage === 'undefined' || !robotId || !session) return
  sessionStorage.setItem(localizationAttemptStorageKey(robotId), JSON.stringify(session))
}

export function clearStoredAttemptSession(robotId) {
  if (typeof sessionStorage === 'undefined' || !robotId) return
  sessionStorage.removeItem(localizationAttemptStorageKey(robotId))
}

/**
 * Start a localization operation in the shared route-planner attempt slot.
 * Other pages use this slot so returning to the route planner never renders
 * the previous operation while a new map transfer/localization is underway.
 */
export function beginStoredAttemptSession(robotId, options = {}) {
  if (!robotId) return null
  const session = emptyAttemptSession(options)
  writeStoredAttemptSession(robotId, session)
  return session
}

/** Persist a command snapshot while retaining stages emitted by earlier commands. */
export function updateStoredAttemptSession(robotId, command, extras = {}) {
  if (!robotId || !command) return null
  const previous = readStoredAttemptSession(robotId)
  const session = localizationAttemptSessionFromCommand(command, {
    ...extras,
    timelineHistory: previous ? localizationAttemptTimeline(previous) : [],
  })
  if (session) writeStoredAttemptSession(robotId, session)
  return session
}
