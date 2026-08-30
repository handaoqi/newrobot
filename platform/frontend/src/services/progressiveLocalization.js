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

export function buildProgressiveLocalizationPayload({ mapId, mapVersion, waypoints = [] }) {
  const normalizedWaypoints = waypoints.map(normalizeWaypoint)
  const waitSeconds = Math.min(900, Math.max(180, 60 + (normalizedWaypoints.length + 1) * 8))
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

export async function initializeProgressiveLocalization({
  mapId,
  robotId,
  mapVersion,
  waypoints = [],
  onProgress = () => {},
  dependencies = {},
}) {
  const activateMap = dependencies.activateRouteMap
  const sendCommand = dependencies.sendRobotNavigationCommand
  const waitCommand = dependencies.waitForRobotCommand
  if (![activateMap, sendCommand, waitCommand].every(item => typeof item === 'function')) {
    throw new Error('渐进定位编排缺少地图激活、命令下发或命令等待实现')
  }

  const activation = await activateMap({
    mapId,
    robotId,
    mapVersion,
    onProgress,
  })
  const payload = buildProgressiveLocalizationPayload({ mapId, mapVersion, waypoints })
  onProgress('地图已下发，正在依次尝试建图原点、静态航向、1米范围、路线航点和全局匹配')
  const createdCommand = await sendCommand(robotId, 'relocalize', payload)
  const command = await waitCommand(robotId, createdCommand, {
    timeoutMs: progressiveLocalizationTimeoutMs(payload),
    onProgress: latest => onProgress(`渐进定位 · ${latest.status || 'created'}`),
  })
  return { activation, payload, command }
}
