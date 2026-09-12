export const LOOP_TRAJECTORY_SYNC_GRACE_MS = 15_000

export function calculateTrajectoryDistance(points = []) {
  let distance = 0
  for (let index = 1; index < points.length; index += 1) {
    const previous = points[index - 1]
    const current = points[index]
    if (previous.map_id && current.map_id && String(previous.map_id) !== String(current.map_id)) continue
    const segment = Math.hypot(Number(current.x) - Number(previous.x), Number(current.y) - Number(previous.y))
    if (Number.isFinite(segment) && segment >= 0 && segment <= 10) distance += segment
  }
  return distance
}

export function guardDutyTrajectoryCaptureState({
  execution = null,
  points = [],
  distance = 0,
  now = Date.now(),
  graceMilliseconds = LOOP_TRAJECTORY_SYNC_GRACE_MS,
} = {}) {
  const pointCount = Array.isArray(points) ? points.length : 0
  const numericDistance = Number(distance)
  if (pointCount >= 2 || (Number.isFinite(numericDistance) && numericDistance > 0)) {
    return { ready: true, reason: 'trajectory_ready', retryAfterMilliseconds: 0 }
  }

  // A task rejected during startup cannot ever produce trajectory samples.
  // Treat it as a zero-distance round instead of presenting an endless sync.
  if (!execution?.started_at) {
    return { ready: true, reason: 'execution_not_started', retryAfterMilliseconds: 0 }
  }

  const terminalTimestamp = Date.parse(
    execution.finished_at || execution.updated_at || execution.created_at || '',
  )
  if (!Number.isFinite(terminalTimestamp)) {
    return { ready: true, reason: 'terminal_time_unavailable', retryAfterMilliseconds: 0 }
  }

  const safeGrace = Math.max(0, Number(graceMilliseconds) || 0)
  const retryAfterMilliseconds = Math.max(0, terminalTimestamp + safeGrace - Number(now))
  return retryAfterMilliseconds > 0
    ? { ready: false, reason: 'sync_pending', retryAfterMilliseconds }
    : { ready: true, reason: 'sync_grace_expired', retryAfterMilliseconds: 0 }
}

export function displayedGuardDutyDistance({
  currentDistance = 0,
  currentExecutionId = '',
  executionLoopSessionId = '',
  loopStartedAt = 0,
  loopSessionId = '',
  loopActive = false,
  loopAccumulatedDistance = 0,
  loopCountedExecutionIds = [],
  loopCurrentExecutionId = '',
  serverLoopSessionId = '',
  serverTotalDistance = null,
} = {}) {
  const current = Number(currentDistance)
  const safeCurrent = Number.isFinite(current) && current >= 0 ? current : 0
  if (!loopStartedAt) return safeCurrent

  const authoritative = Number(serverTotalDistance)
  const hasAuthoritativeDistance = serverTotalDistance !== null
    && serverTotalDistance !== ''
    && Number.isFinite(authoritative)
    && authoritative >= 0
    && loopSessionId
    && String(serverLoopSessionId || '') === String(loopSessionId)
  if (hasAuthoritativeDistance) return authoritative

  const executionId = String(currentExecutionId || '')
  const counted = loopCountedExecutionIds.map(String).includes(executionId)
  const sameLoopSession = Boolean(
    executionLoopSessionId
    && loopSessionId
    && String(executionLoopSessionId) === String(loopSessionId),
  )
  const currentBelongsToLoop = sameLoopSession
    || executionId === String(loopCurrentExecutionId || '')
    || counted

  // A stale completed-loop cache must not replace the distance of a normal
  // one-shot execution selected after the loop ended.
  if (!loopActive && !currentBelongsToLoop) return safeCurrent

  const accumulated = Number(loopAccumulatedDistance)
  const safeAccumulated = Number.isFinite(accumulated) && accumulated >= 0 ? accumulated : 0
  const total = safeAccumulated + (currentBelongsToLoop && !counted ? safeCurrent : 0)

  // If the loop was marked counted while the cloud trajectory was still
  // empty, expose the now-arrived current trajectory instead of keeping 0 m.
  return total > 0 || safeCurrent <= 0 ? total : safeCurrent
}
