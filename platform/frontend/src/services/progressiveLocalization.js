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
  'RTK_INITIAL_POSE_UNAVAILABLE',
  'RTK_INITIAL_POSE_TIMEOUT',
  'RTK_POSE_UNAVAILABLE',
  'RTK_INITIAL_POSE_NOT_CONVERGED',
])

export function shouldInitializeFromRtk({ sceneScope, coordinateMode } = {}) {
  const scene = String(sceneScope || '').trim().toLowerCase()
  const coordinates = String(coordinateMode || '').trim().toLowerCase()
  return OUTDOOR_SCENES.has(scene) && coordinates !== 'local_only'
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
  if (shouldInitializeFromRtk({ sceneScope, coordinateMode })) {
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
      onProgress(`RTK固定解不可用或漂移未达标（${errorCode}），转入快速定位`)
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
    rtkAttempted: Boolean(rtkAttempt),
    rtkAttempt,
  }
}
