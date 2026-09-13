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
}) {
  const normalizedWaypoints = waypoints.map(normalizeWaypoint)
  const waitSeconds = Math.min(
    900,
    Math.max(180, 60 + (normalizedWaypoints.length + 1) * 8),
  )
  return {
    seed_source: 'progressive',
    map_id: mapId,
    map_version: mapVersion,
    waypoints: normalizedWaypoints,
    wait_seconds: waitSeconds,
  }
}

export function progressiveLocalizationTimeoutMs(payload) {
  return (Number(payload?.wait_seconds || 180) + 240) * 1000
}

const OUTDOOR_SCENES = new Set(['outdoor', 'transition'])
const RTK_FALLBACK_CODES = new Set([
  // A fixed-quality flag is not enough to seed the map: Edge also requires
  // three fresh position-and-heading samples inside its self-stability gate.
  // A timeout there is a normal localization fallback, not an operator error.
  'RTK_FIXED_NOT_STABLE',
  'RTK_INITIAL_POSE_UNAVAILABLE',
  'RTK_INITIAL_POSE_TIMEOUT',
  'RTK_POSE_UNAVAILABLE',
  'RTK_INITIAL_POSE_NOT_CONVERGED',
  'LIO_HANDOFF_TIMEOUT',
])

export function shouldInitializeFromRtk({ sceneScope, coordinateMode } = {}) {
  const scene = String(sceneScope || '').trim().toLowerCase()
  const coordinates = String(coordinateMode || '').trim().toLowerCase()
  // Only a map explicitly built with a fixed RTK origin may start the RTK
  // initialization transaction.  Unknown metadata must fail closed to the
  // NDT progressive path, and indoor maps always skip this stage.
  return OUTDOOR_SCENES.has(scene) && coordinates === 'rtk_fixed'
}

function localizationDecision(navigationStatus) {
  const status = navigationStatus?.status || {}
  const quality = status.localization_quality || {}
  const decision = quality.decision || status.localization?.decision || {}
  return decision && typeof decision === 'object' ? decision : {}
}

/**
 * A map configured for RTK does not imply that the live receiver has a
 * usable fixed solution.  Only start the RTK command when the latest Edge
 * decision says that position *and* heading passed its navigation gate.
 * Unknown/stale status deliberately takes the deterministic NDT search path.
 */
export function rtkFixedForInitialization(navigationStatus) {
  const decision = localizationDecision(navigationStatus)
  if (decision.rtk_good_for_navigation === true) return true
  return decision.rtk_usable === true
    && String(decision.rtk_quality || '').trim().toLowerCase() === 'fixed'
    && decision.rtk_heading_usable === true
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

function commandErrorCode(error) {
  return String(
    error?.command?.error_code
    || error?.command?.ack_reason_code
    || error?.error_code
    || error?.code
    || '',
  ).trim()
}

export async function initializeProgressiveLocalization({
  mapId,
  robotId,
  mapVersion,
  waypoints = [],
  sceneScope = 'indoor',
  coordinateMode = '',
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

  let rtkAttempt = null
  const rtkConfigured = shouldInitializeFromRtk({ sceneScope, coordinateMode })
  const rtkFixed = rtkFixedForInitialization(activation.navigationStatus)
  if (rtkConfigured && !rtkFixed) {
    rtkAttempt = { status: 'skipped', errorCode: 'RTK_NOT_FIXED' }
    onProgress('RTK当前不是可用固定解，跳过RTK初始位姿，直接搜索建图原点、附近候选和航点')
  }
  if (rtkConfigured && rtkFixed) {
    onProgress('室外地图已下发，正在使用RTK固定解设置初始姿态并进行本地NDT验证')
    try {
      const rtkPayload = {
        seed_source: 'rtk',
        map_id: mapId,
        map_version: mapVersion,
        wait_seconds: 30,
        start_navigation: true,
      }
      const createdRtkCommand = await sendCommand(robotId, 'initial-pose', rtkPayload, { traceId })
      const command = await waitCommand(robotId, createdRtkCommand, {
        timeoutMs: 90_000,
        onProgress: latest => {
          onProgress(`RTK固定解与本地NDT验证 · ${latest.status || 'created'}`)
          onCommand({ phase: 'localization', command: latest, showCandidates: true, source: 'rtk' })
        },
      })
      onProgress('RTK固定解与本地NDT验证通过')
      return {
        activation,
        payload: rtkPayload,
        command,
        selectedSource: 'rtk_fixed',
        rtkAttempted: true,
      }
    } catch (error) {
      const errorCode = commandErrorCode(error)
      if (!RTK_FALLBACK_CODES.has(errorCode)) throw error
      rtkAttempt = { status: 'failed', errorCode }
      onProgress(`RTK固定解不可用、稳定性不足或FAST-LIO交接未完成（${errorCode}），转入渐进定位`)
    }
  }

  const payload = buildProgressiveLocalizationPayload({
    mapId, mapVersion, waypoints,
  })
  onProgress('地图已下发，依次尝试建图原点、原点周边候选和路线航点，失败后进入全局搜索')
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
    rtkAttempted: rtkAttempt?.status !== 'skipped',
    rtkAttempt,
  }
}
