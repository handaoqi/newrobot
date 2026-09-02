export const ATTEMPT_STATUS_LABELS = {
  waiting: '等待',
  started: '开始',
  verifying: '验证中',
  accepted: '成功',
  rejected: '失败',
  failed: '失败',
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
  }
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
  }
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
