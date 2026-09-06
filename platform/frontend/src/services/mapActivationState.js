export function expectedLegacyMapVersion(mapId) {
  return mapId === null || mapId === undefined || mapId === ''
    ? ''
    : `legacy-mapdata-${mapId}`
}

export function navigationMapIdentity(payload = {}) {
  const safe = payload && typeof payload === 'object' ? payload : {}
  const status = safe.status || {}
  const currentMap = status.current_map || {}
  return {
    mapId: String(currentMap.map_id || status.map_id || safe.current_map_id || ''),
    mapVersion: String(currentMap.map_version || status.map_version || safe.current_map_version || ''),
  }
}

export function navigationStatusFresh(payload = {}, {
  nowMs = Date.now(),
  maxAgeMs = 10_000,
} = {}) {
  const status = payload?.status || {}
  const quality = status.localization_quality || {}
  if (quality.localization_fresh === false) return false
  const sampledAt = quality.localization_sampled_at
    || quality.sampled_at
    || status.sampled_at
    || status.received_at
  if (!sampledAt) return false
  const sampledAtMs = new Date(sampledAt).getTime()
  const ageMs = nowMs - sampledAtMs
  return Number.isFinite(sampledAtMs)
    && ageMs >= -maxAgeMs
    && ageMs <= maxAgeMs
}

export function shouldFallbackToGlobalRelocalization(errorCode) {
  return new Set([
    'RELOCALIZATION_SEED_UNAVAILABLE',
    'ACTIVE_RELOCALIZATION_FAILED',
  ]).has(String(errorCode || ''))
}

export function navigationReadyForMap(payload, mapId, mapVersion = expectedLegacyMapVersion(mapId)) {
  return navigationUnreadinessReason(payload, mapId, mapVersion) === null
}

export function navigationUnreadinessReason(payload, mapId, mapVersion = expectedLegacyMapVersion(mapId)) {
  if (!payload) return '尚未获取定位状态'
  if (payload.connection_status !== 'online') return '机器人不在线'
  const identity = navigationMapIdentity(payload)
  if (!identity.mapId) return '尚未加载任务地图'
  if (identity.mapId !== String(mapId || '')) return '当前地图与任务地图不一致'
  if (identity.mapVersion !== String(mapVersion || '')) return '当前地图版本与任务地图不一致'
  const status = payload.status || {}
  const localizationStatus = status.localization_status || payload.localization_status
  if (localizationStatus && localizationStatus !== 'normal') {
    if (localizationStatus === 'lost') return '定位丢失，待恢复'
    if (localizationStatus === 'initializing') return '定位初始化中'
    return `定位状态：${localizationStatus}`
  }
  const navReady = status.nav_ready ?? payload.nav_ready
  if (!navReady) return '导航栈未就绪'
  if (!navigationStatusFresh(payload)) return '定位数据未刷新'
  return localizationStatus === 'normal' && Boolean(navReady)
    ? null
    : '定位或导航栈未就绪'
}
