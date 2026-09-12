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
])

export const ATTEMPT_MARKER_VISIBLE_MS = 60_000

const ATTEMPT_TERMINAL_STATES = new Set([
  'accepted',
  'failed',
  'handoff_failed',
  'succeeded',
])

const TIMELINE_STAGE_META = {
  map_transfer: { title: '地图下发', detail: '确认目标地图已传输并应用到机器狗' },
  localization_bootstrap: { title: '定位节点准备', detail: '准备 /initialpose 接收器和定位服务' },
  rtk_fixed: { title: 'RTK 固定解验证', detail: '验证 RTK 固定解及 RTK/Fast-LIO 漂移' },
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
  'localization_bootstrap',
  'rtk_fixed',
  'last_trusted',
  'mapping_origin_bounded',
  'route_waypoints',
  'keyframe_global_match',
  'quick_initialization',
  'best_candidate_commit',
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

function normalizeAttempt(attempt, index) {
  const candidate = attempt?.ndt_candidate && typeof attempt.ndt_candidate === 'object'
    ? attempt.ndt_candidate
    : {}
  const seed = attemptSeedPose(attempt)
  const candidateNumber = Number(
    attempt?.candidate_number ?? attempt?.candidateNumber ?? attempt?.index,
  )
  const displayNumber = Number.isFinite(candidateNumber) ? candidateNumber : index + 1
  return {
    // Keep index for protocol compatibility; candidateNumber is the one
    // display identity shared by the map marker and the stage list.
    index: displayNumber,
    candidateNumber: displayNumber,
    status: String(attempt?.status || 'waiting'),
    seedPose: seed,
    livePose: finitePose(attempt?.live_pose || attempt?.livePose),
    matchedPose: finitePose(attempt?.matched_pose || attempt?.matchedPose || candidate.matched_pose),
    matchingError: finiteNumber(candidate.matching_error ?? attempt?.matching_error),
    inlierFraction: finiteNumber(candidate.inlier_fraction ?? attempt?.inlier_fraction),
    geometricRmse: finiteNumber(candidate.geometric_rmse ?? attempt?.geometric_rmse),
    stableFrames: Number(candidate.stable_frames || attempt?.stable_frames || 0),
    rejectReason: attempt?.reject_reason || attempt?.rejectReason || attempt?.error_code || '',
    eligible: Boolean(attempt?.eligible || candidate.eligible),
    accepted: attempt?.accepted === true || attempt?.status === 'accepted',
    stage: attempt?.stage || '',
    source: attempt?.source || '',
    waypointIndex: attempt?.waypoint_index,
    startedAt: firstTimestamp(attempt?.started_at, attempt?.startedAt),
    finishedAt: firstTimestamp(attempt?.finished_at, attempt?.finishedAt),
  }
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
  const raw = result.localization_attempts && typeof result.localization_attempts === 'object'
    ? result.localization_attempts
    : {}
  const attemptSource = Array.isArray(raw.attempts)
    ? raw.attempts
    : (Array.isArray(result.attempts) ? result.attempts : [])
  const phase = extras.phase
    || (String(command.command_type || extras.commandType || '') === 'map.activate' ? 'transfer' : 'localization')
  const showCandidates = extras.showCandidates !== false && phase !== 'transfer'
  const bestMatchPose = finitePose(result.best_match_pose)
    || finitePose(raw.best_match_pose)
    || finitePose(raw.best_ndt_candidate?.matched_pose)
    || finitePose(result.best_ndt_candidate?.matched_pose)
  const session = {
    commandId: String(command.id || extras.commandId || ''),
    commandType: command.command_type || extras.commandType || '',
    commandIssuedAt: firstTimestamp(command.issued_at, command.created_at),
    commandStartedAt: firstTimestamp(command.started_at),
    commandFinishedAt: firstTimestamp(command.finished_at),
    phase,
    showCandidates,
    status: raw.state || command.status || '',
    livePose: finitePose(raw.live_pose) || finitePose(result.live_pose),
    attempts: attemptSource.map(normalizeAttempt),
    bestMatchPose,
    bestNdtCandidate: raw.best_ndt_candidate || result.best_ndt_candidate || null,
    source: raw.selected_stage || raw.source || extras.source || '',
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
    bestNdtCommitted: Boolean(raw.best_ndt_committed ?? result.best_ndt_committed),
    strategy: Array.isArray(raw.strategy)
      ? raw.strategy
      : (Array.isArray(result.strategy)
        ? result.strategy
        : (extras.source === 'rtk'
          ? ['rtk_fixed']
          : (command.command_type === 'nav.initial_pose' ? ['operator_initial_pose'] : []))),
    stages: Array.isArray(raw.stages)
      ? raw.stages
      : (Array.isArray(result.stages)
        ? result.stages
        : (extras.source === 'rtk'
          ? [{ stage: 'rtk_fixed', status: command.status || 'searching' }]
          : (command.command_type === 'nav.initial_pose'
            ? [{ stage: 'operator_initial_pose', status: command.status || 'searching' }]
            : []))),
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
    selectedStage: raw.selected_stage || result.selected_stage || '',
    timelineHistory: Array.isArray(extras.timelineHistory) ? extras.timelineHistory : [],
  }
  const score = finiteNumber(session.bestNdtCandidate?.matching_error)
  session.optimalVerified = Boolean(
    session.rtkDrift?.verified
    || (session.bestNdtCommitted && score !== null && score < 0.01),
  )
  return withAttemptMarkerExpiry(session)
}

export function emptyAttemptSession({ phase = 'localization', commandType = '', commandId = '' } = {}) {
  return {
    commandId: String(commandId || ''),
    commandType,
    commandIssuedAt: null,
    commandStartedAt: null,
    commandFinishedAt: null,
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
    bestNdtCommitted: false,
    optimalVerified: false,
    strategy: [],
    stages: [],
    localizationBootstrap: null,
    navigationStart: null,
    bestCandidateCommitStartedAt: null,
    bestCandidateCommitFinishedAt: null,
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
  if (attempts.some(attempt => ['verifying', 'started', 'running', 'executing', 'in_progress', 'committing'].includes(attempt.status))) return 'searching'
  if (stageRecord?.status && timelineStatusClass(stageRecord.status) !== 'waiting') return stageRecord.status
  if (stageRecord?.status) return stageRecord.status
  if (attempts.some(attempt => attempt.status === 'accepted')) return 'accepted'
  if (attempts.length && attempts.every(attempt => ['rejected', 'failed', 'skipped'].includes(attempt.status))) return 'rejected'
  const selectedStage = canonicalTimelineStage(session?.selectedStage)
  if (selectedStage === stageKey) return 'searching'
  if (!isAttemptSessionTerminal(session)) {
    const firstStage = canonicalTimelineStage((session?.strategy || [])[0] || 'mapping_origin_bounded')
    if (firstStage === stageKey) return 'searching'
  }
  return 'waiting'
}

function timelineDetail(stageKey, status, attempts, session) {
  const meta = TIMELINE_STAGE_META[stageKey] || { title: stageKey, detail: '' }
  if (stageKey === 'map_transfer' && session?.phase === 'transfer') {
    return status === 'done' ? '地图下发并应用完成' : '正在等待机器狗确认地图命令'
  }
  if (attempts.length) {
    const active = attempts.find(attempt => (
      ['verifying', 'started', 'running', 'executing', 'in_progress'].includes(attempt.status)
    ))
    const committing = attempts.find(attempt => attempt.status === 'committing')
    const evaluated = attempts.filter(attempt => !['waiting', 'verifying', 'started', 'running', 'executing', 'in_progress', 'committing'].includes(attempt.status)).length
    const activeText = active
      ? `正在尝试 #${active.candidateNumber}`
      : (committing ? `正在提交 #${committing.candidateNumber}` : '')
    return `${meta.detail} · ${activeText ? `${activeText} · ` : ''}已完成 ${evaluated}/${attempts.length} 个候选`
  }
  return meta.detail
}

function timelineStageTimes(stageKey, status, record, session) {
  const commandStart = firstTimestamp(session?.commandStartedAt, session?.commandIssuedAt)
  const commandFinish = firstTimestamp(session?.commandFinishedAt)
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
      commandStart,
    )
    finishedAt = firstTimestamp(
      finishedAt,
      session?.localizationBootstrap?.finished_at,
      session?.localizationBootstrap?.finishedAt,
      session?.localizationBootstrap ? startedAt : null,
    )
  } else if (stageKey === 'best_candidate_commit') {
    startedAt = firstTimestamp(startedAt, session?.bestCandidateCommitStartedAt, commandStart)
    finishedAt = firstTimestamp(finishedAt, session?.bestCandidateCommitFinishedAt, commandFinish)
  } else if (stageKey === 'navigation_start') {
    startedAt = firstTimestamp(
      startedAt,
      session?.navigationStart?.started_at,
      session?.navigationStart?.startedAt,
      commandStart,
    )
    finishedAt = firstTimestamp(
      finishedAt,
      session?.navigationStart?.finished_at,
      session?.navigationStart?.finishedAt,
      session?.navigationStart ? commandFinish : null,
    )
  } else if (stageKey === 'map_transfer' && session?.phase !== 'transfer') {
    startedAt = firstTimestamp(startedAt, historical?.startedAt, commandStart)
    finishedAt = firstTimestamp(finishedAt, historical?.finishedAt, commandStart)
  } else {
    startedAt = firstTimestamp(startedAt, commandStart)
  }
  if (!finishedAt && ['done', 'failed', 'skipped'].includes(timelineStatusClass(status))) {
    finishedAt = startedAt
  }
  return { startedAt, finishedAt }
}

function timestampMillis(value) {
  if (!value) return null
  const millis = Date.parse(value)
  return Number.isFinite(millis) ? millis : null
}

function orderedTimelineTimes(session, timeline) {
  let cursor = timestampMillis(firstTimestamp(session?.commandStartedAt, session?.commandIssuedAt))
  return timeline.map(step => {
    let startedAt = step.startedAt
    let finishedAt = step.finishedAt
    let startedMillis = timestampMillis(startedAt)
    let finishedMillis = timestampMillis(finishedAt)
    const status = timelineStatusClass(step.status)

    if (startedMillis === null && status !== 'waiting') {
      startedMillis = cursor
      startedAt = startedMillis === null ? null : new Date(startedMillis).toISOString()
    }
    if (startedMillis !== null && cursor !== null && startedMillis < cursor) {
      startedMillis = cursor
      startedAt = new Date(startedMillis).toISOString()
    }
    if (finishedMillis !== null && startedMillis !== null && finishedMillis < startedMillis) {
      finishedMillis = startedMillis
      finishedAt = new Date(finishedMillis).toISOString()
    }
    if (
      finishedMillis === null
      && startedMillis !== null
      && ['done', 'failed', 'skipped'].includes(status)
    ) {
      const commandFinish = timestampMillis(session?.commandFinishedAt)
      finishedMillis = commandFinish === null ? startedMillis : Math.max(commandFinish, startedMillis)
      finishedAt = new Date(finishedMillis).toISOString()
    }

    if (finishedMillis !== null) cursor = finishedMillis
    else if (startedMillis !== null) cursor = Math.max(cursor ?? startedMillis, startedMillis)

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
      detail: detail || timelineDetail(key, status, attempts, session),
      status: timelineStatusClass(status),
      statusLabel: timelineStatusLabel(status, key),
      attempts,
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
  const addStageKey = value => {
    const key = canonicalTimelineStage(value)
    if (!key || key === 'map_transfer' || key === 'localization_bootstrap' || stageKeys.includes(key)) return
    if (TIMELINE_STAGE_META[key] || stageRecords.has(key)) stageKeys.push(key)
  }
  ;(session.strategy || []).forEach(addStageKey)
  ;(session.stages || []).forEach(record => addStageKey(record?.stage))
  ;(session.attempts || []).forEach(attempt => addStageKey(attempt?.stage))
  if (session.rtkDrift && !stageKeys.includes('rtk_fixed')) stageKeys.unshift('rtk_fixed')
  if (!stageKeys.length) stageKeys.push('mapping_origin_bounded', 'route_waypoints', 'keyframe_global_match')

  stageKeys.forEach(key => {
    const record = stageRecords.get(key)
    const attempts = stageAttemptsFor(session, key, record)
    add(
      key,
      inferredStageStatus(session, key, attempts, record),
      attempts,
      record?.error_message || record?.message || '',
      record,
    )
  })

  const commitStatus = session.bestNdtCommitted || (commandDone && session.bestMatchPose)
    ? 'accepted'
    : (session.bestMatchPose ? 'searching' : 'waiting')
  add('best_candidate_commit', commitStatus)

  const shouldShowNavigation = session.commandType !== 'nav.initial_pose' || session.navigationStart
  if (shouldShowNavigation) {
    const navigationStatus = session.navigationStart
      ? 'accepted'
      : (commandDone && session.status === 'accepted' ? 'searching' : 'waiting')
    add('navigation_start', navigationStatus)
  }
  return orderedTimelineTimes(session, mergeTimelineHistory(session, timeline))
}

function mergeTimelineHistory(session, currentTimeline) {
  const history = Array.isArray(session?.timelineHistory) ? session.timelineHistory : []
  if (!history.length) return currentTimeline
  const merged = history.map(step => ({ ...step }))
  currentTimeline.forEach(step => {
    const index = merged.findIndex(item => item.key === step.key)
    if (index >= 0) merged[index] = step
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
