export const ATTEMPT_STATUS_LABELS = {
  waiting: '等待',
  started: '开始',
  verifying: '验证中',
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

export function attemptSeedPose(attempt) {
  return finitePose(attempt?.seedPose || attempt?.seed_pose) || finitePose(attempt)
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
  return {
    index: Number.isFinite(Number(attempt?.index)) ? Number(attempt.index) : index + 1,
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
    waypointIndex: attempt?.waypoint_index,
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
    rtkDrift: null,
    bestNdtCommitted: false,
    optimalVerified: false,
    strategy: [],
    stages: [],
    localizationBootstrap: null,
    navigationStart: null,
    selectedStage: '',
    timelineHistory: [],
  }
}

function canonicalTimelineStage(value) {
  const key = String(value || '').trim()
  return TIMELINE_STAGE_ALIASES[key] || key
}

function timelineStatusClass(status) {
  if (status === 'accepted' || status === 'succeeded' || status === 'done') return 'done'
  if (status === 'rejected' || status === 'failed' || status === 'unavailable') return 'failed'
  if (status === 'searching' || status === 'running' || status === 'verifying' || status === 'executing') return 'active'
  if (status === 'skipped') return 'skipped'
  return 'waiting'
}

function timelineStatusLabel(status) {
  const labels = {
    waiting: '待执行',
    active: '执行中',
    done: '已完成',
    failed: '失败',
    skipped: '已跳过',
  }
  return labels[timelineStatusClass(status)]
}

function stageAttemptsFor(session, stageKey) {
  const aliases = new Set([stageKey])
  Object.entries(TIMELINE_STAGE_ALIASES).forEach(([alias, canonical]) => {
    if (canonical === stageKey) aliases.add(alias)
  })
  return (session?.attempts || []).filter(attempt => aliases.has(canonicalTimelineStage(attempt.stage)))
}

function inferredStageStatus(session, stageKey, attempts, stageRecord) {
  if (stageRecord?.status) return stageRecord.status
  if (attempts.some(attempt => attempt.status === 'verifying' || attempt.status === 'started')) return 'searching'
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
    const evaluated = attempts.filter(attempt => attempt.status !== 'waiting').length
    return `${meta.detail} · 已评估 ${evaluated}/${attempts.length} 个候选`
  }
  return meta.detail
}

export function localizationAttemptTimeline(session) {
  if (!session) return []
  const timeline = []
  const add = (key, status, attempts = [], detail = '') => {
    const meta = TIMELINE_STAGE_META[key] || { title: key, detail: '' }
    timeline.push({
      key,
      title: meta.title,
      detail: detail || timelineDetail(key, status, attempts, session),
      status: timelineStatusClass(status),
      statusLabel: timelineStatusLabel(status),
      attempts,
    })
  }

  const commandStatus = String(session.status || '').toLowerCase()
  const commandDone = isAttemptSessionTerminal(session)
  const transferStatus = session.phase === 'transfer'
    ? commandStatus
    : 'accepted'
  add('map_transfer', transferStatus)
  if (session.phase === 'transfer') return mergeTimelineHistory(session, timeline)

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
    const attempts = stageAttemptsFor(session, key)
    const record = stageRecords.get(key)
    add(
      key,
      inferredStageStatus(session, key, attempts, record),
      attempts,
      record?.error_message || record?.message || '',
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
  return mergeTimelineHistory(session, timeline)
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
  if (status === 'verifying' || status === 'started') return 'active'
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
    return withAttemptMarkerExpiry(session)
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
