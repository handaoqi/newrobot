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
    localization_mode: String(localizationMode || '').trim().toLowerCase(),
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
 * Unknown/stale status is distinct from an explicit non-fixed result so Edge
 * can verify fresh RTK samples instead of skipping the authoritative source.
 */
export function rtkFixedForInitialization(navigationStatus) {
  const decision = localizationDecision(navigationStatus)
  if (decision.rtk_good_for_navigation === true) return true
  return decision.rtk_usable === true
    && String(decision.rtk_quality || '').trim().toLowerCase() === 'fixed'
    && decision.rtk_heading_usable === true
}

export function rtkInitializationSnapshotState(navigationStatus) {
  const decision = localizationDecision(navigationStatus)
  if (rtkFixedForInitialization(navigationStatus)) return 'fixed'
  const quality = String(decision.rtk_quality || '').trim().toLowerCase()
  const hasExplicitEvidence = quality.length > 0
    || typeof decision.rtk_usable === 'boolean'
    || typeof decision.rtk_heading_usable === 'boolean'
    || typeof decision.rtk_good_for_navigation === 'boolean'
  return hasExplicitEvidence ? 'not_fixed' : 'unknown'
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
