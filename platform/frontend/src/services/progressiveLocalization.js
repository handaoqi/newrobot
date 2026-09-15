function normalizeWaypoint(point, index) {
  const rawX = Array.isArray(point) ? point[0] : point?.x
  const rawY = Array.isArray(point) ? point[1] : point?.y
  const rawYaw = Array.isArray(point) ? point[2] : point?.yaw
  const validCoordinate = value => value !== null
    && value !== undefined
    && value !== ''
    && typeof value !== 'boolean'
  const x = validCoordinate(rawX) ? Number(rawX) : Number.NaN
  const y = validCoordinate(rawY) ? Number(rawY) : Number.NaN
  const yaw = rawYaw === null || rawYaw === undefined || rawYaw === ''
    ? 0
    : (typeof rawYaw === 'boolean' ? Number.NaN : Number(rawYaw))
  if (![x, y, yaw].every(Number.isFinite)) {
    throw new Error(`第 ${index + 1} 个路线航点坐标无效`)
  }
  return { x, y, yaw }
}

export function buildProgressiveLocalizationPayload({
  mapId,
  mapVersion,
  waypoints = [],
  sceneScope = 'indoor',
  coordinateMode = 'local_only',
  localizationMode = '',
}) {
  const normalizedWaypoints = waypoints.map(normalizeWaypoint)
  const waypointLocalizationMode = Array.isArray(waypoints[0])
    ? ''
    : String(waypoints[0]?.localization_mode || '')
  const waitSeconds = Math.min(
    900,
    Math.max(180, 60 + (normalizedWaypoints.length + 1) * 8),
  )
  return {
    seed_source: 'progressive',
    map_id: mapId,
    map_version: mapVersion,
    scene_scope: String(sceneScope || 'indoor').trim().toLowerCase(),
    coordinate_mode: String(coordinateMode || 'local_only').trim().toLowerCase(),
    // The waypoint mode is a secondary-correction policy only.  Keeping it
    // on the command lets Edge apply the same policy after the NDT anchor is
    // committed, without allowing the UI to reorder initialization sources.
    localization_mode: String(localizationMode || waypointLocalizationMode).trim().toLowerCase(),
    waypoints: normalizedWaypoints,
    wait_seconds: waitSeconds,
  }
}

export function progressiveLocalizationTimeoutMs(payload) {
  return (Number(payload?.wait_seconds || 180) + 240) * 1000
}

/**
 * Edge returns a successful localization command only after its initial-pose
 * contract has verified the continuous FAST-LIO handoff and (when requested)
 * Nav2 readiness.  Do not turn that authoritative result into a 70 s UI-only
 * "waiting convergence" failure because telemetry replication lags.
 */
export function localizationCommandVerified(command) {
  const status = String(command?.status || '').trim().toLowerCase()
  if (!['succeeded', 'accepted', 'completed'].includes(status)) return false
  const result = command?.result_payload
  if (!result || typeof result !== 'object') return false
  if (result.handoff_pending === true) return false
  const attempts = result.localization_attempts
  if (attempts?.state === 'handoff_failed') return false
  return true
}

export async function initializeProgressiveLocalization({
  mapId,
  robotId,
  mapVersion,
  waypoints = [],
  sceneScope = 'indoor',
  coordinateMode = '',
  localizationMode = '',
  onProgress = () => {},
  onCommand = () => {},
  dependencies = {},
  traceId = '',
  existingActivation = null,
}) {
  const activateMap = dependencies.activateRouteMap
  const sendCommand = dependencies.sendRobotNavigationCommand
  const waitCommand = dependencies.waitForRobotCommand
  if (![sendCommand, waitCommand].every(item => typeof item === 'function')) {
    throw new Error('渐进定位编排缺少地图激活、命令下发或命令等待实现')
  }

  const activation = existingActivation || await (async () => {
    if (typeof activateMap !== 'function') {
      throw new Error('渐进定位编排缺少地图激活实现')
    }
    return activateMap({
      mapId,
      robotId,
      mapVersion,
      onProgress,
      onCommand,
      traceId,
    })
  })()

  const payload = buildProgressiveLocalizationPayload({
    mapId, mapVersion, waypoints, sceneScope, coordinateMode,
    localizationMode,
  })
  onProgress('地图已下发，先搜索建图原点及附近候选并提交最优 NDT，再由 FAST-LIO + IMU 接管，最后执行 RTK/UKF/NDT 二次校正')
  const createdCommand = await sendCommand(robotId, 'relocalize', payload, { traceId })
  const command = await waitCommand(robotId, createdCommand, {
    timeoutMs: progressiveLocalizationTimeoutMs(payload),
    onProgress: latest => {
      onProgress(`原点/航点候选搜索 · ${latest.status || 'created'}`)
      onCommand({ phase: 'localization', command: latest, showCandidates: true, source: 'progressive' })
    },
  })
  return {
    activation,
    payload,
    command,
    selectedSource: 'progressive',
    rtkAttempted: false,
    rtkAttempt: null,
  }
}
