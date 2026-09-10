const ACTIVE_EXECUTION_STATES = new Set(['running', 'resuming'])

export const LIVE_STATUS_MAX_AGE_MS = 8_000
export const LIVE_VELOCITY_MAX_AGE_SECONDS = 3
export const TRAJECTORY_SPEED_MAX_AGE_MS = 8_000

function finiteNumber(value) {
  if (value === null || value === undefined || value === '') return null
  const number = Number(value)
  return Number.isFinite(number) ? number : null
}

function timestampMs(value) {
  const timestamp = new Date(value || '').getTime()
  return Number.isFinite(timestamp) ? timestamp : null
}

function boundedSpeed(value) {
  return Math.min(Math.abs(value), 5)
}

function statusPayload(navigationStatus) {
  return navigationStatus?.status || navigationStatus || null
}

function statusIsFresh(status, nowMs) {
  const receivedAt = timestampMs(status?.received_at || status?.sampled_at)
  return receivedAt !== null
    && nowMs - receivedAt >= -LIVE_STATUS_MAX_AGE_MS
    && nowMs - receivedAt <= LIVE_STATUS_MAX_AGE_MS
}

function trajectorySpeed(points, nowMs) {
  if (!Array.isArray(points) || points.length < 2) {
    return { available: false, reason: '轨迹点不足' }
  }
  const current = points[points.length - 1]
  const currentSampleTime = timestampMs(current.sampled_at || current.received_at)
  const currentReceivedTime = timestampMs(current.received_at || current.sampled_at)
  if (currentSampleTime === null || currentReceivedTime === null) {
    return { available: false, reason: '轨迹时间无效' }
  }
  const visibleAgeMs = nowMs - currentReceivedTime
  if (visibleAgeMs < -TRAJECTORY_SPEED_MAX_AGE_MS || visibleAgeMs > TRAJECTORY_SPEED_MAX_AGE_MS) {
    return { available: false, reason: '轨迹速度数据已过期' }
  }
  for (let index = points.length - 2; index >= 0; index -= 1) {
    const previous = points[index]
    const previousTime = timestampMs(previous.sampled_at || previous.received_at)
    if (previousTime === null) continue
    const elapsedSeconds = (currentSampleTime - previousTime) / 1000
    if (elapsedSeconds < 0.2) continue
    if (previous.map_id && current.map_id && String(previous.map_id) !== String(current.map_id)) {
      return { available: false, reason: '轨迹地图已切换' }
    }
    const x = finiteNumber(current.x)
    const y = finiteNumber(current.y)
    const previousX = finiteNumber(previous.x)
    const previousY = finiteNumber(previous.y)
    if ([x, y, previousX, previousY].some((value) => value === null)) {
      return { available: false, reason: '轨迹坐标无效' }
    }
    const distance = Math.hypot(x - previousX, y - previousY)
    if (!Number.isFinite(distance) || distance > 10) {
      return { available: false, reason: '轨迹坐标发生跳变' }
    }
    return {
      available: true,
      valueMps: boundedSpeed(distance / elapsedSeconds),
      turnRps: null,
      source: 'trajectory',
      reason: '使用轨迹坐标估算',
    }
  }
  return { available: false, reason: '轨迹采样间隔不足' }
}

export function guardDutySpeedReading({
  executionState,
  navigationStatus,
  trajectory = [],
  nowMs = Date.now(),
} = {}) {
  if (!ACTIVE_EXECUTION_STATES.has(String(executionState || ''))) {
    return {
      available: true,
      valueMps: 0,
      turnRps: 0,
      source: 'task_inactive',
      reason: '任务当前未运行',
    }
  }

  const status = statusPayload(navigationStatus)
  const navigation = status?.navigation || {}
  if (statusIsFresh(status, nowMs)) {
    const actualSpeed = finiteNumber(navigation.actual_planar_speed_mps)
    const actualTurn = finiteNumber(navigation.actual_turn_speed_rps)
    const actualAge = finiteNumber(navigation.actual_velocity_sample_age_seconds)
    if (actualSpeed !== null && actualAge !== null && actualAge <= LIVE_VELOCITY_MAX_AGE_SECONDS) {
      return {
        available: true,
        valueMps: boundedSpeed(actualSpeed),
        turnRps: actualTurn,
        source: 'actual_velocity',
        reason: '设备实际控制速度',
      }
    }

    const localization = status.localization_quality || {}
    const localizedSpeed = finiteNumber(status.speed_mps ?? navigation.localized_speed_mps)
    const localizationAge = finiteNumber(localization.localization_sample_age_seconds)
    if (
      localizedSpeed !== null
      && localization.localization_fresh === true
      && localizationAge !== null
      && localizationAge <= LIVE_VELOCITY_MAX_AGE_SECONDS
    ) {
      return {
        available: true,
        valueMps: boundedSpeed(localizedSpeed),
        turnRps: actualTurn,
        source: 'localization_velocity',
        reason: '设备定位速度',
      }
    }
  }

  const fallback = trajectorySpeed(trajectory, nowMs)
  if (fallback.available) return fallback
  return {
    available: false,
    valueMps: null,
    turnRps: null,
    source: 'unavailable',
    reason: fallback.reason || '速度数据不可用',
  }
}

export function guardDutySpeedLabel(reading) {
  return reading?.available && Number.isFinite(reading.valueMps)
    ? `${reading.valueMps.toFixed(2)} m/s`
    : '-- m/s'
}

export function guardDutySpeedTitle(reading) {
  if (!reading?.available) return reading?.reason || '速度数据不可用'
  const turn = Number.isFinite(reading.turnRps)
    ? `；角速度 ${reading.turnRps.toFixed(2)} rad/s`
    : ''
  return `${reading.reason || '速度正常'}${turn}`
}

export function isLatestTrajectoryResponse({
  requestGeneration,
  latestGeneration,
  requestedExecutionId,
  currentExecutionId,
} = {}) {
  return Number(requestGeneration) === Number(latestGeneration)
    && String(requestedExecutionId || '') === String(currentExecutionId || '')
}
