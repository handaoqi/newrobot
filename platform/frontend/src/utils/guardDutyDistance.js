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
} = {}) {
  const current = Number(currentDistance)
  const safeCurrent = Number.isFinite(current) && current >= 0 ? current : 0
  if (!loopStartedAt) return safeCurrent

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
